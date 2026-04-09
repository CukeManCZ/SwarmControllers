#include "swarm_controllers/boid_controller.hpp" 

namespace boid_controller {
  //Initialization //{
  BoidController::BoidController(rclcpp::NodeOptions options)
    : Node("boid_controller_node", options)
  {
    init_timer_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&BoidController::InitializationCallback, this)); 
  }
  
  void BoidController::InitializationCallback(){
    node_ = shared_from_this();
    RCLCPP_INFO(node_->get_logger(), "Initializing node.."); 

    std::string _leader_name;
    //Parameters from config //{ 
    mrs_lib::ParamLoader param_loader(node_, "BoidController");
    param_loader.loadParam("config", _config_file_);
    if(_config_file_ != ""){
      RCLCPP_INFO_STREAM(node_->get_logger(), "This is the path:" << _config_file_); 
      param_loader.addYamlFile(_config_file_);
    }

    param_loader.loadParam("uav_names", _uav_names_);
    param_loader.loadParam("uav_name", _uav_name_);
    param_loader.loadParam("odometry_topic", _odometry_topic_name_);
    param_loader.loadParam("set_reference_timer/rate", _rate_timer_set_reference_);
    param_loader.loadParam("control_frame", _control_frame_);
    param_loader.loadParam("odom_msg_max_latency", _odom_msg_max_latency_);
    param_loader.loadParam("diagnostics/odom_timeout", _odom_timeout_);
    param_loader.loadParam("diagnostics/rate", _rate_timer_diagnostics_);
    param_loader.loadParam("controlled_dimensions", _c_dimensions_);
    param_loader.loadParam("boid_constants/flocking", FLOCKING_CONSTANT);
    param_loader.loadParam("boid_constants/avoidance", AVOIDANCE_CONSTANT);
    param_loader.loadParam("boid_constants/velocity", VELOCITY_M_CONSTANT);
    param_loader.loadParam("boid_constants/waypoint", WAYPOINT_CONSTANT);
    //}
    
    //Find this UAV
    auto it = std::find(_uav_names_.begin(), _uav_names_.end(), _uav_name_); 
    
    //Remove from uav which we track
    if (it != _uav_names_.end()) {
      this_uav_idx_ = it - _uav_names_.begin();
      _uav_names_.erase(it);
    } 
    else {
      RCLCPP_ERROR(node_->get_logger(), "[BoidController]: This UAV is not part of the formation! Check the config file. Shutting down node.");
      rclcpp::shutdown();
    }
    n_drones_ = _uav_names_.size();
    uav_positions_.resize(n_drones_);
    uav_velocities_.resize(n_drones_);
    last_odom_msg_time_.resize(n_drones_);

    //Subscribe to information of UAVs except this drones//{
    for (size_t i = 0; i < _uav_names_.size(); i++) {
      std::string topic_name = "/" + _uav_names_[i] + "/" + _odometry_topic_name_; 

      auto callback = [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->OdomCallback(msg, i);
      };

      other_uav_odom_subscribers_.push_back(
        create_subscription<nav_msgs::msg::Odometry>(
          topic_name, rclcpp::QoS(1), callback
        )
      );

      RCLCPP_INFO(node_->get_logger(), "Subscribing to %s", topic_name.c_str());
    }    
    //}
  
    //This UAV odom
    sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>("/" + _uav_name_ + "/" + _odometry_topic_name_, rclcpp::QoS(1) , std::bind(&BoidController::ThisOdomCallback,this,std::placeholders::_1 ));
    sub_waypoint_ = create_subscription<geometry_msgs::msg::PointStamped>("/waypoint",rclcpp::QoS(1), std::bind(&BoidController::WaypointCallback, this, std::placeholders::_1));
    pub_centroid_ = create_publisher<geometry_msgs::msg::PointStamped>("/centroid", 1);
    sub_boid_constants_ = create_subscription<myc_messages::msg::KeyValueFloatArray>("/boidConstants", rclcpp::QoS(1), std::bind(&BoidController::BoidConstantsCallback, this, std::placeholders::_1));

    mrs_lib::SubscriberHandlerOptions shopts;
    shopts.node                 = node_;
    shopts.node_name          = "BoidController";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.qos        = 10;

    sh_position_command_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>(shopts, "tracker_cmd_in");

    //Send reference command to UAV 
    timer_set_reference_ = create_wall_timer(std::chrono::duration<double>(1.0/_rate_timer_set_reference_), std::bind(&BoidController::SetReferenceCallback, this));
    timer_diagnostics_ = create_wall_timer(std::chrono::duration<double>(1.0/_rate_timer_diagnostics_), std::bind(&BoidController::TimerDiagnosticCallback, this));
    //activate uav control
    ser_activate_control_ = create_service<std_srvs::srv::Trigger>("control_activation_in", std::bind(&BoidController::ActivateControlCallback,this,  std::placeholders::_1, std::placeholders::_2));
    
    cli_set_position_ = create_client<mrs_msgs::srv::ReferenceStampedSrv>("ref_pos_out");

    transformer_ = std::make_shared<mrs_lib::Transformer>(node_);
    transformer_->retryLookupNewest(true);

    is_initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "[BoidController]: Initialization completed.");

    init_timer_->cancel();
  }
  //}
  
  //Miscell //{
  bool BoidController::IsInitialized(std::string functionName){
    if (!is_initialized_){
      RCLCPP_INFO_STREAM(node_->get_logger(), "Node not initialized" << "| Call from: |" << functionName);
      return false; 
    }
    return true;
  }
  //}

  
  // Reference Callback //{
  void BoidController::SetReferenceCallback(){
    if(!IsInitialized(__func__)) 
        return; 

    if(!all_robots_positions_valid_ && !got_position_command_){
      RCLCPP_WARN(get_logger(), "[BoidController]: Waiting for valid robots."); 
      GetPositionCmd();
      return;
    }

    if (!control_allowed_){
      RCLCPP_WARN(get_logger(), "[BoidController]: Waiting for activation");
      return;
    }

    GetPositionCmd();

    // Calculate velocity/position reference
    mrs_msgs::msg::Reference p_ref;
    {
      std::scoped_lock lock(mutex_uav_odoms_, mutex_position_command_);

      Eigen::Vector3d accel = CalculateReferenceAcceleration();
      double ts = 1.0 / double(_rate_timer_set_reference_);
      double multiplier = 1.0;

      // ---- 1) Compute next position (unclamped) ----
      double nx = uav_position_.x() + accel.x() * ts * multiplier;
      double ny = uav_position_.y() + accel.y() * ts * multiplier;
      double nz;

      if (_c_dimensions_ == 3){
        nz = uav_position_.z() + accel.z() * ts;
        if (nz < 0.2)
          nz = 0.2;   // minimum
      } else {
        nz = 2.0;
      }

      // ---- 2) Clamp BEFORE assigning to p_ref ----
      const float x_size = 24.0f;
      const float y_size = 24.0f;
      const float z_size = 14.0f;

      if (nx > x_size) nx = x_size;
      if (nx < -x_size) nx = -x_size;

      if (ny > y_size) ny = y_size;
      if (ny < -y_size) ny = -y_size;

      if (nz > z_size) nz = z_size;

      // ---- 3) NOW set the reference ----
      p_ref.position.x = nx;
      p_ref.position.y = ny;
      p_ref.position.z = nz;
      p_ref.heading = 0.0;

      RCLCPP_INFO_STREAM(get_logger(), "[BoidController] p_ref (control_frame=" << _control_frame_ 
  << ") x=" << p_ref.position.x << " y=" << p_ref.position.y << " z=" << p_ref.position.z);
    }


    // Set drone velocity
    auto request = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();
    request -> reference = p_ref;
    request -> header.frame_id = _control_frame_;
    request -> header.stamp = get_clock()->now();
 
    if(cli_set_position_->service_is_ready())
    {
      cli_set_position_->async_send_request(request, [&](const rclcpp::Client<mrs_msgs::srv::ReferenceStampedSrv>::SharedFuture fut)
      {
        const auto result = fut.get();
        if(result->success){
          //RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3.0,  "[BoidController]: Reference set success");
        }
        else{
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3.0,  "[BoidController] Reference set response error [%s].", result->message.c_str());
        }

      });   
    }
    else
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3.0,  "[BoidController] Service SendReference is not ready.");
    }
  }
  //}

  // Timer Diagnostic Callback //{
  void BoidController::TimerDiagnosticCallback(){
    if(!IsInitialized(__func__))
      return;

    bool timeout_exceeded = false;
    std::stringstream msg;
    msg.precision(2);
    for(size_t i = 0; i < last_odom_msg_time_.size(); i++)
    {
      double time_since_last_message = (get_clock()->now() - last_odom_msg_time_[i]).seconds();
      msg << _uav_names_[i] << "(" << time_since_last_message << "s), ";
      if(time_since_last_message > _odom_timeout_)
        timeout_exceeded = true;
    }

    if(timeout_exceeded)
      RCLCPP_WARN(node_->get_logger(), "[BoidController] %s", msg.str().c_str());

    all_robots_positions_valid_ = !timeout_exceeded;
  }
  //}
  
  // ActivateControllCallback //{
  void BoidController::ActivateControlCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res){
    if(!IsInitialized(__func__))
      return;

    RCLCPP_INFO(get_logger(), "[BoidController]: Activation service called.");
    res->success = true; 
    if(control_allowed_){
      res->message = "Control was already allowed.";
      RCLCPP_WARN(get_logger(), "[BoidController]: %s", res->message.c_str());
    }
    else if (!all_robots_positions_valid_){
      res->message = "Robots are not ready, control not activated.";
      RCLCPP_WARN(get_logger(), "[BoidController]: %s", res->message.c_str());
      res->success = false;
    }
    else{
      control_allowed_ = true;
      res->message = "Control allowed";
      RCLCPP_WARN(get_logger(), "[BoidController]: %s", res->message.c_str());
    }
  }
  //}
  
  //OdomCallback for UAVs //{ 
  void BoidController::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, size_t idx){
    if(!IsInitialized(__func__))
        return;
    
    
    if ((get_clock()->now() - msg->header.stamp).seconds() > _odom_msg_max_latency_)
    {
      RCLCPP_WARN(get_logger(), "[BoidController]: The latency of odom message for %s exceeds the threshold (latency = %.2f s).", _uav_names_[idx].c_str(),
               (get_clock()->now() - msg->header.stamp).seconds());
    }

    geometry_msgs::msg::PointStamped new_point;
    new_point.header = msg->header;
    new_point.header.frame_id = _uav_names_[idx]+"/gps_garmin_origin";
    new_point.point.x = msg->pose.pose.position.x;
    new_point.point.y = msg->pose.pose.position.y;
    new_point.point.z = msg->pose.pose.position.z;

    //Position transform
    auto res = transformer_->transformSingle(new_point, _control_frame_);
    if (res) {
      new_point = res.value();
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "[BoidController]: Could not transform odometry msg to control frame.");
      return;
    }

    Eigen::Vector3d transformed_position;
    if (_c_dimensions_ == 3) {
      transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, new_point.point.z);
    } else {
      transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, 0.0);
    }
   
    //Velocity transform
    Eigen::Vector3d velocity_in(msg->twist.twist.linear.x,
                              msg->twist.twist.linear.y,
                              msg->twist.twist.linear.z);

    auto vel_res = transformer_->transformAsVector(
    _uav_names_[this_uav_idx_] + "/gps_garmin_origin",  // from frame
    velocity_in,
    _control_frame_,                                     // to frame
    msg->header.stamp);

    if (!vel_res) {
      RCLCPP_ERROR_STREAM(get_logger(), "[BoidController]: Could not transform odometry msg velocity to control frame.");
      return;
    }

    Eigen::Vector3d transformed_velocity = *vel_res;
    if (_c_dimensions_ != 3)
     transformed_velocity.z() = 0.0;

    mrs_lib::set_mutexed(mutex_uav_odoms_, transformed_position, uav_positions_[idx]);
    mrs_lib::set_mutexed(mutex_uav_odoms_, transformed_velocity, uav_velocities_[idx]);

    last_odom_msg_time_[idx] = get_clock()->now();
  }
  //}
  

  //This uav odometry //{
  void BoidController::ThisOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    if(!IsInitialized(__func__))
        return;
    
    geometry_msgs::msg::PointStamped new_point;
    new_point.header = msg->header;
    new_point.header.frame_id = _uav_name_ + "/gps_garmin_origin";
    new_point.point.x = msg->pose.pose.position.x;
    new_point.point.y = msg->pose.pose.position.y;
    new_point.point.z = msg->pose.pose.position.z;

    //Position transform
    auto res = transformer_->transformSingle(new_point, _control_frame_);
    if (res) {
      new_point = res.value();
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "[BoidController]: Could not transform odometry msg to control frame.");
      return;
    }

    Eigen::Vector3d transformed_position;
    if (_c_dimensions_ == 3) {
      transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, new_point.point.z);
    } else {
      transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, 0.0);
    }
   
    //Velocity transform
    Eigen::Vector3d velocity_in(msg->twist.twist.linear.x,
                              msg->twist.twist.linear.y,
                              msg->twist.twist.linear.z);

    auto vel_res = transformer_->transformAsVector(
    _uav_name_ + "/gps_garmin_origin",  // from frame
    velocity_in,
    _control_frame_,                                     // to frame
    msg->header.stamp);

    if (!vel_res) {
      RCLCPP_ERROR_STREAM(get_logger(), "[BoidController]: Could not transform odometry msg velocity to control frame.");
      return;
    }

    Eigen::Vector3d transformed_velocity = *vel_res;
    if (_c_dimensions_ != 3)
     transformed_velocity.z() = 0.0;
    
    uav_position_ = transformed_position;
    uav_velocity_ = transformed_velocity;

  }
  //}

  //Waypoint command //{
  void BoidController::WaypointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
    if(!IsInitialized(__func__))
      return;

    RCLCPP_INFO_STREAM(get_logger(), "[BoidController]: Got position command X:" << msg->point.x << " Y:" << msg->point.y << "Z: " << msg->point.z);
    mrs_lib::set_mutexed(mutex_position_command_, msg->point, position_command_);
  }
  //}

  //Boid constants callback //{
  void BoidController::BoidConstantsCallback(const myc_messages::msg::KeyValueFloatArray::SharedPtr msg){
    if(!IsInitialized(__func__))
        return;

    //Boid constants names:
    std::vector<std::string> boidConstantNames = {"flocking", "avoidance", "velocity", "waypoint"};
    
    for(size_t i = 0; i < msg->data.size(); i++){
      std::string key = msg->data[i].key;
      double value = msg->data[i].value;
      if(key == "flocking"){
        FLOCKING_CONSTANT = value;
      }
      else if(key == "avoidance"){
        AVOIDANCE_CONSTANT = value;
      }
      else if(key == "velocity"){
        VELOCITY_M_CONSTANT = value;
      }
      else if(key == "waypoint"){
        WAYPOINT_CONSTANT = value;
      }else{
        RCLCPP_INFO_STREAM(get_logger(), "[BoidController]: Nonidenfiable key[ " << key << ":" << value << "]");
      }
    }
  }
  //}

  //PositionCmd from tracker //{
  void BoidController::GetPositionCmd(){
    if(!IsInitialized(__func__))
      return;

    if(!sh_position_command_.hasMsg()){
      return; 
    }

    mrs_msgs::msg::TrackerCommand msg = *sh_position_command_.getMsg();

    geometry_msgs::msg::PointStamped new_point;

    new_point.header = msg.header;
    new_point.point  = msg.position;

    auto res = transformer_->transformSingle(new_point, _control_frame_);

    if (res) {
      new_point = res.value();
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "[BoidController]: Could not transform position command to control frame.");
      return;
    }

    got_position_command_ = true;
  }
  //}

  // Boid controll //{
  Eigen::Vector3d BoidController::GetBoidCenteroid(){
    if(uav_positions_.size() != size_t(n_drones_))
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3.0, "[BoidController]: Number of drones is not consistent with number of positions.");

    if(uav_positions_.size() > 0){
      //std::scoped_lock lock(mutex_uav_odoms_);
      Eigen::Vector3d sum = Eigen::Vector3d::Zero();

      for(size_t i = 0; i < uav_positions_.size(); i++){
        sum += uav_positions_[i]; 
      }

      sum += uav_position_;

      Eigen::Vector3d centroid = sum / (double(uav_positions_.size()) + 1.0);

      geometry_msgs::msg::PointStamped ps;
      ps.point.set__x(centroid.x());
      ps.point.set__y(centroid.y());
      ps.point.set__z(centroid.z());
      ps.header.frame_id = _control_frame_;
      ps.header.stamp = get_clock()->now();

      pub_centroid_->publish(ps);
      RCLCPP_INFO(get_logger(), "[BoidController]: Publishing");
      return centroid;
    }

    return Eigen::Vector3d();
  }

  Eigen::Vector3d BoidController::GetNearestNeighbourPosition(){
    return uav_positions_[GetNearestNeighbour()];
  }

  Eigen::Vector3d BoidController::GetNearestNeighbourVelocity(){
    return uav_velocities_[GetNearestNeighbour()];
  }

  size_t BoidController::GetNearestNeighbour(){
      if (uav_positions_.empty()) {
      return 0;
    }

    double best_dist = std::numeric_limits<double>::infinity();
    size_t best_idx = 0;

    for (size_t i = 0; i < uav_positions_.size(); i++) {
      double dist = ComputeDistance(uav_positions_[i], uav_position_);
      if (dist < best_dist) {
        best_dist = dist;
        best_idx = i;
      }
    }

    return best_idx;
  }

  double BoidController::ComputeDistance(Eigen::Vector3d a, Eigen::Vector3d b){
    return sqrt( pow(a.x() - b.x(), 2)
        + pow(a.y() - b.y(), 2)
        + pow(a.z() - b.z(), 2));
  }

  Eigen::Vector3d BoidController::CalculateReferenceAcceleration(){

    Eigen::Vector3d flocking = CalculateFlockingRule()* FLOCKING_CONSTANT;
    Eigen::Vector3d avoidance = CalculateAvoidanceRule()* AVOIDANCE_CONSTANT;
    Eigen::Vector3d velocity = CalculateVelocityMatchRule()* VELOCITY_M_CONSTANT;
    Eigen::Vector3d waypoint = CalculateWaypointRule()* WAYPOINT_CONSTANT;

    RCLCPP_INFO_STREAM(get_logger(), "[BoidController]" << 
      " Flock:" << FLOCKING_CONSTANT <<  
      " Avoidance:" << AVOIDANCE_CONSTANT << 
      " Velocity:" << VELOCITY_M_CONSTANT << 
      " Waypoint:" << WAYPOINT_CONSTANT);
    RCLCPP_INFO_STREAM(get_logger(), "[BoidController] Flock: X:" << flocking.x() <<  " Y:" << flocking.y() << " Z:" << flocking.z());
    RCLCPP_INFO_STREAM(get_logger(), "[BoidController] Avoid: X:" << avoidance.x() <<  " Y:" << avoidance.y() << " Z:" << avoidance.z());
    RCLCPP_INFO_STREAM(get_logger(), "[BoidController] Veloc: X:" << velocity.x() <<  " Y:" << velocity.y() << " Z:" << velocity.z());
    RCLCPP_INFO_STREAM(get_logger(), "[BoidController] Waypo: X:" << waypoint.x() <<  " Y:" << waypoint.y() << " Z:" << waypoint.z());

    return flocking
      + avoidance
      + velocity
      + waypoint;
  }
  
  Eigen::Vector3d BoidController::CalculateFlockingRule(){
    if (uav_positions_.empty()) return Eigen::Vector3d::Zero();
    Eigen::Vector3d diff = GetBoidCenteroid() - uav_position_;
    double d = diff.norm();
    if (d < 1e-6) 
      return Eigen::Vector3d::Zero();
    
    return diff / d;
  }

  Eigen::Vector3d BoidController::CalculateAvoidanceRule(){
    Eigen::Vector3d diff = GetNearestNeighbourPosition() - uav_position_;
    double d = diff.norm();
    if (d < 1e-6) return Eigen::Vector3d::Zero();
    return -diff / d;
  }

  Eigen::Vector3d BoidController::CalculateVelocityMatchRule(){
    Eigen::Vector3d diff = GetNearestNeighbourVelocity() - uav_velocity_;
    if (!diff.allFinite()) 
      return Eigen::Vector3d::Zero();
    return diff;
  }

  Eigen::Vector3d BoidController::CalculateWaypointRule(){
    if (!got_position_command_) return Eigen::Vector3d::Zero();
    Eigen::Vector3d command(position_command_.x, position_command_.y, position_command_.z);
    Eigen::Vector3d diff = command - uav_position_;
    double d = diff.norm();
    if (d < 1e-6) 
      return Eigen::Vector3d::Zero();
    return diff;
  }

  //}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(boid_controller::BoidController);
