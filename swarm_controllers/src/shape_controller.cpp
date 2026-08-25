#include "swarm_controllers/shape_controller.hpp" 

namespace shape_controller {
  //Initialization //{
  ShapeController::ShapeController(rclcpp::NodeOptions options)
    : Node("shape_controller_node", options)
  {
    this->initialize();
  }
  
  void ShapeController::initialize(){
    node_ = this_node_ptr();
    clock_ = node_->get_clock();

    cbkgrp_subs_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cbkgrp_ss_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cbkgrp_sc_     = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::on_shutdown([this]() {this->shutdown(); });

    RCLCPP_INFO(node_->get_logger(), "Initializing node.."); 

    std::string _leader_name;
    // | ----------------------- load paramas -----------------| //{ 
    mrs_lib::ParamLoader param_loader(node_, "ShapeController");
    param_loader.loadParam("config", _config_file_);
    if(_config_file_ != ""){
      RCLCPP_INFO_STREAM(node_->get_logger(), "This is the path:" << _config_file_); 
      param_loader.addYamlFile(_config_file_);
    }

    param_loader.loadParam("uav_names", _uav_names_);
    param_loader.loadParam("uav_name", _uav_name_);
    param_loader.loadParam("odometry_topic", _odometry_topic_name_);
    param_loader.loadParam("set_reference_timer/rate", _rate_timer_set_reference_);
    param_loader.loadParam("feedback/rate", _rate_timer_feedback_);
    param_loader.loadParam("control_frame", _control_frame_);
    param_loader.loadParam("odom_msg_max_latency", _odom_msg_max_latency_);
    param_loader.loadParam("diagnostics/odom_timeout", _odom_timeout_);
    param_loader.loadParam("diagnostics/rate", _rate_timer_diagnostics_);
    param_loader.loadParam("controlled_dimensions", _c_dimensions_);
    param_loader.loadParam("shape_constants/avoidance", AVOIDANCE_CONSTANT);
    param_loader.loadParam("shape_constants/waypoint", WAYPOINT_CONSTANT);
    param_loader.loadParam("shape_constants/avoidance_distance", AVOIDANCE_DISTANCE);
    param_loader.loadParam("shape_constants/cell_size", _cellSize_);
    param_loader.loadParam("shape_constants/threshold", _threshold_);
    param_loader.loadParam("intent_topic", _intent_topic_name_);
    param_loader.loadParam("shape_constants/default_rate", _rate_timer_default_);
    param_loader.loadParam("shape_constants/travel", TRAVEL_CONSTANT);
    param_loader.loadParam("shape_constants/min_height", _minHeight_);
    //}
    
    rateScale = _rate_timer_set_reference_ / _rate_timer_default_;

    // | ----------------------- UAV Swarm initialization -----------------|
    auto it = std::find(_uav_names_.begin(), _uav_names_.end(), _uav_name_); 
    
    //Remove from uav which we track
    if (it != _uav_names_.end()) {
      this_uav_idx_ = it - _uav_names_.begin();
      _uav_names_.erase(it);
    } 
    else {
      RCLCPP_ERROR(node_->get_logger(), "[ShapeController]: This UAV is not part of the formation! Check the config file. Shutting down node.");
      rclcpp::shutdown();
    }
    n_drones_ = _uav_names_.size();
    uav_positions_.resize(n_drones_);
    uav_velocities_.resize(n_drones_);
    last_odom_msg_time_.resize(n_drones_, clock_->now());
    uav_target_intents_.resize(n_drones_);

    // | ---------------------- subscribers ---------------------|
    mrs_lib::SubscriberHandlerOptions shopts;
    shopts.node                 = node_;
    shopts.node_name          = "ShapeController";
    shopts.no_message_timeout = mrs_lib::no_timeout;
    shopts.threadsafe         = true;
    shopts.autostart          = true;
    shopts.qos        = 10;
    shopts.subscription_options.callback_group = cbkgrp_subs_;

    //Subscribe UAVs informations except this drone//{
    for (size_t i = 0; i < _uav_names_.size(); i++) {
      //| ------------------------- odometry ---------------------|
      {
      std::string odom_topic_name = "/" + _uav_names_[i] + "/" + _odometry_topic_name_; 
      other_uav_odom_subscribers_.push_back(
        mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(
            shopts,
            odom_topic_name,
            std::bind(&ShapeController::OdomCallback, this, std::placeholders::_1, i)
            )
      );
      RCLCPP_INFO(node_->get_logger(), "Subscribing to %s", odom_topic_name.c_str());
      }
      //| ------------------------- intent ---------------------|
      {
      std::string intent_topic_name = "/" + _uav_names_[i] + "/" + _intent_topic_name_; 
      other_uav_target_intents_subscribers_.push_back(
        mrs_lib::SubscriberHandler<geometry_msgs::msg::PoseStamped>(
        shopts,
        intent_topic_name,
        std::bind(&ShapeController::IntentCallback, this, std::placeholders::_1, i)
        )
      );
      RCLCPP_INFO(node_->get_logger(), "Subscribing to %s", intent_topic_name.c_str());
      }
    }    
    //}
    //This UAV odom
    sub_odometry_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(
      shopts,
      "/" + _uav_name_ + "/" + _odometry_topic_name_,
      std::bind(&ShapeController::ThisOdomCallback,this,std::placeholders::_1 )
    );
    sub_waypoint_ = mrs_lib::SubscriberHandler<geometry_msgs::msg::PointStamped>(
      shopts,
      "/waypoint",
      std::bind(&ShapeController::WaypointCallback, this, std::placeholders::_1)
    );
    sh_position_command_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>(
      shopts,
      "tracker_cmd_in");
    //Octomap incoming data
    sub_octomap_shape_ = mrs_lib::SubscriberHandler<octomap_msgs::msg::OctomapWithPose>(
      shopts,
      "/octomap",
      std::bind(&ShapeController::OctomapShapeCallback, this, std::placeholders::_1));
    
    // | ---------------------- publishers ---------------------|
    // Rviz Visualization
    {
      mrs_lib::PublisherHandlerOptions opts;
      opts.node = node_;
      pub_octomac_rviz_ = mrs_lib::PublisherHandler<octomap_msgs::msg::Octomap>(opts, "/plain_octomap");
    }
    {
      mrs_lib::PublisherHandlerOptions opts;
      opts.node = node_;
      pub_shape_markers_ = mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>(opts, "shape_nodes");
    }
    //Shape control visualization
    {
      mrs_lib::PublisherHandlerOptions opts;
      opts.node = node_;
      pub_target_marker_ = mrs_lib::PublisherHandler<visualization_msgs::msg::Marker>(opts, "/waypoint");
    }
    {
      mrs_lib::PublisherHandlerOptions opts;
      opts.node = node_;
      uav_intent_publisher_ = mrs_lib::PublisherHandler<geometry_msgs::msg::PoseStamped>(opts, "/" + _uav_name_ + "/" + _intent_topic_name_);
    }
    {
      mrs_lib::PublisherHandlerOptions opts;
      opts.node = node_;
      pub_feedback_ = mrs_lib::PublisherHandler<std_msgs::msg::Float64>(opts, "/" + _uav_name_ + "/" + "feedback");
    }

    // | ---------------------- timers ---------------------|
    mrs_lib::TimerHandlerOptions opts;
        opts.node = node_;
        opts.autostart = true;
        opts.callback_group = cbkgrp_timers_;
    //Send reference command to UAV 
    {
      std::function<void()> callback_fcn = std::bind(&ShapeController::SetReferenceCallback, this);
      timer_set_reference_ = std::make_shared<TimerType>(opts, rclcpp::Rate(_rate_timer_set_reference_, clock_), callback_fcn);
    }
    {
      std::function<void()> callback_fcn = std::bind(&ShapeController::TimerDiagnosticCallback, this);
      timer_diagnostics_ = std::make_shared<TimerType>(opts, rclcpp::Rate(_rate_timer_diagnostics_, clock_), callback_fcn);
    }
    //Feedback timer
    {
      std::function<void()> callback_fcn = std::bind(&ShapeController::TimerFeedbackCallback, this);
      timer_feedback_ = std::make_shared<TimerType>(opts, rclcpp::Rate(_rate_timer_feedback_, clock_), callback_fcn);
    }
    // | ---------------------- services ---------------------|
    ser_activate_control_ = mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger>(
      node_,
      "control_activation_in",
      std::bind(&ShapeController::ActivateControlCallback,this,  std::placeholders::_1, std::placeholders::_2)
    );
    // | ---------------------- client ---------------------|
    cli_set_position_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(
      node_, 
      "ref_pos_out",
      cbkgrp_sc_
    );

    lastTarget_ = Eigen::Vector3d::Zero();
    previousScore_ = std::numeric_limits<double>::lowest();
    previousScoreTime_ = clock_->now();

    transformer_ = std::make_shared<mrs_lib::Transformer>(node_);
    transformer_->retryLookupNewest(true);

    //Octomap shape data receiving an visualization
    shapeBuilder_ = shape_lib::ShapeBuilder();
    InitOctoTree();
    wasShapeReseted = true;
    wasOctomapReceived = false;

    is_initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), "[ShapeController]: Initialization completed.");
  }

  void ShapeController::shutdown(){
    std::cout << "[ShapeController]: shutdown(): called" << std::endl;
  }
  //}
  
  void ShapeController::OctomapShapeCallback(const octomap_msgs::msg::OctomapWithPose::ConstSharedPtr msg)
  {
      if (!msg) return;

      auto* abstract_tree = octomap_msgs::msgToMap(msg->octomap);
      if (!abstract_tree) return;

      auto* octree = dynamic_cast<octomap::OcTree*>(abstract_tree);
      if (!octree) {
          delete abstract_tree;
          return;
      }

      tree_.reset(octree);
      shapeNodes_ =  shapeBuilder_.ExtractShape(shapeBuilder_.BuildGrid(*tree_, _cellSize_));
      previousScore_ = std::numeric_limits<double>::lowest();
      previousScoreTime_ = clock_->now();
      wasShapeReseted = true;
      wasOctomapReceived = true;

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 20.0, "[ShapeController] Incoming octomap.");

      // --- Publish octomap ---
      octomap_msgs::msg::Octomap plain;
      if (octomap_msgs::binaryMapToMsg(*tree_, plain)) {
        plain.header.frame_id = _uav_name_  + "/world_origin";
        //plain.header.stamp = tf_msg.header.stamp;
        pub_octomac_rviz_.publish(plain);
        PublishShapeNodes(shapeNodes_);
      }
  }

  void ShapeController::InitOctoTree(){
    tree_ = std::make_shared<octomap::OcTree>(1.0);

    int size = 20;

    for(int x = -size; x < size; ++x){
      for(int y = -size; y < size; ++y){
        for(int z = -size; z < size; ++z){
          octomap::point3d endpoint ((float) x*0.5f, (float) y*0.5f, (float) z*0.5f);
          tree_->updateNode(endpoint, true); 
        }
      }
    }

    tree_->updateInnerOccupancy();
    shapeNodes_ = shapeBuilder_.ExtractShape(shapeBuilder_.BuildGrid(*tree_, _cellSize_));
    // --- Publish octomap ---
    octomap_msgs::msg::Octomap plain;
    if (octomap_msgs::binaryMapToMsg(*tree_, plain)) {
      plain.header.frame_id = "simulator_origin";
      //plain.header.stamp = tf_msg.header.stamp;
      pub_octomac_rviz_.publish(plain);
      
      PublishShapeNodes(shapeNodes_);
    }
  }
  
  //Miscell //{
  bool ShapeController::IsInitialized(std::string functionName){
    if (!is_initialized_){
      RCLCPP_INFO_STREAM(node_->get_logger(), "Node not initialized" << "| Call from: |" << functionName);
      return false; 
    }
    return true;
  }
  //}
  
  // Reference Callback //{
  void ShapeController::SetReferenceCallback(){
    if(!IsInitialized(__func__)) 
        return; 

    if(!all_robots_positions_valid_ && !got_position_command_){
      RCLCPP_WARN(node_->get_logger(), "[ShapeController]: Waiting for valid robots.");
      GetPositionCmd();
      return;
    }

    if (!control_allowed_){
      RCLCPP_WARN(node_->get_logger(), "[ShapeController]: Waiting for activation");
      return;
    }

    if (!wasOctomapReceived){
      RCLCPP_WARN(node_->get_logger(), "[ShapeController] Waiting for valid octomap");
      return;

    }else{
      //Average print for debug
      {
        Eigen::Vector3d sum(0.0, 0.0, 0.0);

        for (const auto& node : shapeNodes_) {
            sum += node.position;
        }

        Eigen::Vector3d avg = sum / static_cast<double>(shapeNodes_.size());

        RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *clock_, 2.0,
            "[Octomap] Average position: [x=%.3f, y=%.3f, z=%.3f]",
            avg.x(), avg.y(), avg.z()
        );
      }
    }

    GetPositionCmd();

    // Calculate velocity/position reference
    mrs_msgs::msg::Reference p_ref;
    {
      std::scoped_lock lock(mutex_uav_odoms_, mutex_position_command_);

      Eigen::Vector3d accel = CalculateReferenceAcceleration();
      double ts = 1.0 / double(_rate_timer_set_reference_);

      // ---- Compute next position ----
      double nx =  accel.x() * ts * rateScale + uav_position_.x();
      double ny =  accel.y() * ts * rateScale + uav_position_.y();
      double nz;

      if (_c_dimensions_ == 3){
        nz = accel.z() * ts * rateScale + uav_position_.z();
        if (nz < _minHeight_)
          nz = _minHeight_;   // minimum
      } else {
        nz = 2.0;
      }
      // ---- Set the reference ----
      p_ref.position.x = nx;
      p_ref.position.y = ny;
      p_ref.position.z = nz;
      p_ref.heading = 0.0;

      RCLCPP_INFO_STREAM(node_->get_logger(), "[Reference] [" 
        << p_ref.position.x << "," << p_ref.position.y << "," << p_ref.position.z << "]-control_frame=" << _control_frame_ << ")");
    }

    auto request = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();
    request -> reference = p_ref;
    request -> header.frame_id = _control_frame_;
    request -> header.stamp = clock_->now();

    auto future_opt = cli_set_position_.callAsync(request);

    if (!future_opt) {
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 3.0,
        "[ShapeController] Service call failed (not ready)");
      return;
    }

    auto future = future_opt.value();

    if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      auto result = future.get();

      if (result->success) {
        // success
      } else {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 3.0,
          "[ShapeController] Reference set error [%s]", result->message.c_str());
      }
    }
  }
  //}

  // Timer Diagnostic Callback //{
  void ShapeController::TimerDiagnosticCallback(){
    if(!IsInitialized(__func__))
      return;

    bool timeout_exceeded = false;
    std::stringstream msg;
    msg.precision(2);
    for(size_t i = 0; i < last_odom_msg_time_.size(); i++)
    {
      double time_since_last_message = (clock_->now() - last_odom_msg_time_[i]).seconds();
      msg << _uav_names_[i] << "(" << time_since_last_message << "s), ";
      if(time_since_last_message > _odom_timeout_)
        timeout_exceeded = true;
    }

    if(timeout_exceeded)
      RCLCPP_WARN(node_->get_logger(), "[ShapeController] %s", msg.str().c_str());

    all_robots_positions_valid_ = !timeout_exceeded;
  }
  //}
  
  //Timer Feedback Callback //{
  void ShapeController::TimerFeedbackCallback(){
    if(!IsInitialized(__func__))
      return;

    std_msgs::msg::Float64 msg;
    
    float distance = ComputeDistance(uav_position_ , lastTarget_);
    msg.data = distance;
    pub_feedback_.publish(msg);
  }
  //}

  // ActivateControllCallback //{
  void ShapeController::ActivateControlCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res){
    if(!IsInitialized(__func__))
      return;

    RCLCPP_INFO(node_->get_logger(), "[ShapeController]: Activation service called.");
    res->success = true; 
    if(control_allowed_){
      res->message = "Control was already allowed.";
      RCLCPP_WARN(node_->get_logger(), "[ShapeController]: %s", res->message.c_str());
    }
    else if (!all_robots_positions_valid_){
      res->message = "Robots are not ready, control not activated.";
      RCLCPP_WARN(node_->get_logger(), "[ShapeController]: %s", res->message.c_str());
      res->success = false;
    }
    else{
      control_allowed_ = true;
      res->message = "Control allowed";
      RCLCPP_WARN(node_->get_logger(), "[ShapeController]: %s", res->message.c_str());
    }
  }
  //}
  
  //OdomCallback for UAVs //{ 
  void ShapeController::OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg, size_t idx){
    if(!IsInitialized(__func__))
        return;
    
    
    if ((clock_->now() - msg->header.stamp).seconds() > _odom_msg_max_latency_)
    {
      RCLCPP_WARN(node_->get_logger(), "[ShapeController]: The latency of odom message for %s exceeds the threshold (latency = %.2f s).", _uav_names_[idx].c_str(),
               (clock_->now() - msg->header.stamp).seconds());
    }

    geometry_msgs::msg::PointStamped new_point;

    new_point.header = msg->header;
    new_point.header.frame_id = _uav_names_[idx]+"/gps_garmin_origin";
    new_point.point.x = msg->pose.pose.position.x;
    new_point.point.y = msg->pose.pose.position.y;
    new_point.point.z = msg->pose.pose.position.z;

    Eigen::Vector3d transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, new_point.point.z);
    Eigen::Vector3d transformed_velocity = Eigen::Vector3d (msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    
    //Position transform -> this is needed only if drones are not in the same world frame, now we ignore it
    /*
    auto res = transformer_->transformSingle(new_point, _control_frame_);
    if (res) {
      new_point = res.value();
    } else {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "[ShapeController]: Could not transform odometry msg to control frame.");
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
    _uav_names_[idx] + "/gps_garmin_origin",  // from frame
    velocity_in,
    _control_frame_,                                     // to frame
    msg->header.stamp);

    if (!vel_res) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "[ShapeController]: Could not transform odometry msg velocity to control frame.");
      return;
    }

    Eigen::Vector3d transformed_velocity = *vel_res;
    if (_c_dimensions_ != 3)
     transformed_velocity.z() = 0.0;
    
    */

    mrs_lib::set_mutexed(mutex_uav_odoms_, transformed_position, uav_positions_[idx]);
    mrs_lib::set_mutexed(mutex_uav_odoms_, transformed_velocity, uav_velocities_[idx]);

    last_odom_msg_time_[idx] = clock_->now();
  }
  //}
  
  //IntentCallback for UAVs //{
  void ShapeController::IntentCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg, size_t idx){
    if(!IsInitialized(__func__))
        return;

    
    geometry_msgs::msg::PointStamped new_point;
    rclcpp::Time msg_time(msg->header.stamp);

    new_point.header = msg->header;
    new_point.header.stamp = msg_time;
    new_point.header.frame_id = _uav_names_[idx]+"/gps_garmin_origin";
    new_point.point.x = msg->pose.position.x;
    new_point.point.y = msg->pose.position.y;
    new_point.point.z = msg->pose.position.z;

    //Position transform -> this is needed only if drones are not in the same world frame, now we ignore it
    /*
    auto res = transformer_->transformSingle(new_point, _control_frame_);
    if (res) {
      new_point = res.value();
    } else {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "[ShapeController]: Could not transform odometry msg to control frame.");
      return;
    }

    Eigen::Vector3d transformed_position;
    if (_c_dimensions_ == 3) {
      transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, new_point.point.z);
    } else {
      transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, 0.0);
    }
    */
    geometry_msgs::msg::PoseStamped intentPose;
    intentPose.header = new_point.header;
    intentPose.pose.position.x = new_point.point.x;
    intentPose.pose.position.y = new_point.point.y;
    intentPose.pose.position.z = new_point.point.z;

    mrs_lib::set_mutexed(mutex_uav_intent_, intentPose, uav_target_intents_[idx]);
  }
  //}

  //This uav odometry //{
  void ShapeController::ThisOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg){
    if(!IsInitialized(__func__))
        return;
    
    geometry_msgs::msg::PointStamped new_point;
    rclcpp::Time msg_time(msg->header.stamp);

    new_point.header = msg->header;
    new_point.header.stamp = msg_time;
    new_point.header.frame_id = _uav_name_ + "/gps_garmin_origin";
    new_point.point.x = msg->pose.pose.position.x;
    new_point.point.y = msg->pose.pose.position.y;
    new_point.point.z = msg->pose.pose.position.z;

    Eigen::Vector3d transformed_position = Eigen::Vector3d(new_point.point.x, new_point.point.y, new_point.point.z);
    Eigen::Vector3d transformed_velocity = Eigen::Vector3d (msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

    //Position transform
    /*
    auto res = transformer_->transformSingle(new_point, _control_frame_);
    if (res) {
      new_point = res.value();
    } else {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "[ShapeController]: Could not transform odometry msg to control frame.");
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
      RCLCPP_ERROR_STREAM(node_->get_logger(), "[ShapeController]: Could not transform odometry msg velocity to control frame.");
      return;
    }

    Eigen::Vector3d transformed_velocity = *vel_res;
    if (_c_dimensions_ != 3)
     transformed_velocity.z() = 0.0;
    */
    
    uav_position_ = transformed_position;
    uav_velocity_ = transformed_velocity;
  }
  //}

  //Waypoint command //{
  void ShapeController::WaypointCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg){
    if(!IsInitialized(__func__))
      return;

    RCLCPP_INFO_STREAM(node_->get_logger(), "[ShapeController]: Got position command X:" << msg->point.x << " Y:" << msg->point.y << "Z: " << msg->point.z);
    mrs_lib::set_mutexed(mutex_position_command_, msg->point, position_command_);
  }
  //}

  //PositionCmd from tracker //{
  void ShapeController::GetPositionCmd(){
    if(!IsInitialized(__func__))
      return;

    if(!sh_position_command_.hasMsg()){
      return; 
    }

    mrs_msgs::msg::TrackerCommand msg = *sh_position_command_.getMsg();

    geometry_msgs::msg::PointStamped new_point;

    rclcpp::Time msg_time(msg.header.stamp);

    new_point.header = msg.header;
    new_point.header.stamp = msg_time;
    new_point.point  = msg.position;

    auto res = transformer_->transformSingle(new_point, _control_frame_);

    if (res) {
      new_point = res.value();
    } else {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "[ShapeController]: Could not transform position command to control frame.");
      return;
    }

    got_position_command_ = true;
  }
  //}

  // Shape controll //{
  void ShapeController::PublishShapeNodes(std::vector<shape_lib::ShapeNode> nodes){
    visualization_msgs::msg::MarkerArray array;

    int id = 0;
    for (const auto& n : nodes)
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "simulator_origin";
      m.header.stamp = clock_->now();
      m.ns = "shape_nodes";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = visualization_msgs::msg::Marker::ADD;

      // position
      m.pose.position.x = n.position.x();
      m.pose.position.y = n.position.y();
      m.pose.position.z = n.position.z();
      m.pose.orientation.w = 1.0;

      // size
      m.scale.x = n.size;
      m.scale.y = n.size;
      m.scale.z = n.size;

      // color (simple heatmap)
      float v = std::min(6, n.value) / 6.0f;
      m.color.r = v;
      m.color.g = 1.0f - v;
      m.color.b = 0.0f;
      m.color.a = 0.4f;

      array.markers.push_back(m);
    }

    pub_shape_markers_.publish(array);
  }

  Eigen::Vector3d ShapeController::GetNearestOccupiedVoxel(){
    double best_dist = std::numeric_limits<double>::infinity();
    Eigen::Vector3d bestVoxelPos;

    for (auto it = tree_->begin(); it != tree_->end(); ++it) {

        if (!tree_->isNodeOccupied(*it))
            continue;
        Eigen::Vector3d center(it.getX(), it.getY(), it.getZ());
        double size = it.getSize();

        double dist = DistanceToVoxel(uav_position_, center, size);

        if (dist < best_dist) {
            best_dist = dist;
            bestVoxelPos = center; // or closest point on voxel
        }
    }

    return bestVoxelPos;
  }

  double ShapeController::DistanceToVoxel(const Eigen::Vector3d& p, const Eigen::Vector3d& center, double size)
  {
    double half = size * 0.5;

    Eigen::Vector3d min = center - Eigen::Vector3d::Constant(half);
    Eigen::Vector3d max = center + Eigen::Vector3d::Constant(half);

    Eigen::Vector3d d(0,0,0);

    //Computes distance based on sides of voxel
    for (int i = 0; i < 3; ++i) {
        if (p[i] < min[i])
          d[i] = min[i] - p[i];
        else if (p[i] > max[i]) 
          d[i] = p[i] - max[i];
    }

    
    return d.norm();
  }

  Eigen::Vector3d ShapeController::GetNearestNeighbourPosition(){
    if (uav_positions_.empty()) {
      return uav_position_;  // Return self position if no neighbors
    }
    return uav_positions_[GetNearestNeighbour()];
  }

  Eigen::Vector3d ShapeController::GetNearestNeighbourVelocity(){
    if (uav_velocities_.empty()) {
      return Eigen::Vector3d::Zero();  // Return zero velocity if no neighbors
    }
    return uav_velocities_[GetNearestNeighbour()];
  }

  size_t ShapeController::GetNearestNeighbour(){
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

  double ShapeController::ComputeDistance(Eigen::Vector3d a, Eigen::Vector3d b){
    return sqrt( pow(a.x() - b.x(), 2)
        + pow(a.y() - b.y(), 2)
        + pow(a.z() - b.z(), 2));
  }

  Eigen::Vector3d ShapeController::CalculateReferenceAcceleration(){

    Eigen::Vector3d avoidance = CalculateAvoidanceRule();
    Eigen::Vector3d waypoint = CalculateUavToShapeRule()* WAYPOINT_CONSTANT;

    //RCLCPP_INFO_STREAM(get_logger(), "[ShapeController] Avoid: Size("<< avoidance.norm() << ")" << "X(" << avoidance.x() <<  ") Y(" << avoidance.y() << " ) Z(" << avoidance.z() << ")");
    //RCLCPP_INFO_STREAM(get_logger(), "[ShapeController] Waypo: Size("<< waypoint.norm() << ")" << "X(" << waypoint.x() <<  ") Y(" << waypoint.y() << " ) Z(" << waypoint.z() << ")");

    return avoidance +
      waypoint;
  }

  Eigen::Vector3d ShapeController::CalculateAvoidanceRule()
  {
      // For single UAV (no neighbors), return zero avoidance
      if (uav_positions_.empty()) {
          return Eigen::Vector3d::Zero();
      }

      Eigen::Vector3d diff = GetNearestNeighbourPosition() - uav_position_;
      double d = diff.norm();

      if (d < 1e-6)
          return Eigen::Vector3d::Zero();

      //if (d >= AVOIDANCE_DISTANCE)
      //    return Eigen::Vector3d::Zero();

      double k = AVOIDANCE_CONSTANT;  // tuning gain
      double force = k * (1.0 / d - 1.0 / AVOIDANCE_DISTANCE);
      force = std::max(0.0, force);

      return -diff.normalized() * force;
  }

  double ShapeController::ComputeScore(Eigen::Vector3d nodePosition, std::vector<geometry_msgs::msg::PoseStamped> intents){
    // Handle single UAV case (no other drones)
    if(intents.empty()){
      double travelCost = ComputeDistance(uav_position_, nodePosition);
      return -TRAVEL_CONSTANT * travelCost;  // Just minimize travel cost
    }

    double minDist = std::numeric_limits<double>::max();
    if(uav_positions_.size() == intents.size()){
      for(size_t i = 0; i < uav_positions_.size(); ++i){
        Eigen::Vector3d other(
        intents[i].pose.position.x,
        intents[i].pose.position.y,
        intents[i].pose.position.z
        );

        double d = ComputeDistance(nodePosition, other);
        minDist = std::min(minDist, d);
      }

      double travelCost = ComputeDistance(uav_position_, nodePosition);
      //Can comment
      return (minDist - TRAVEL_CONSTANT * travelCost);
    }

    return std::numeric_limits<double>::max();
  }

  Eigen::Vector3d ShapeController::CalculateUavToShapeRule(){
    if (!got_position_command_) return Eigen::Vector3d::Zero();

    Eigen::Vector3d targetPosition = uav_position_;

    //TODO: Check incoming intent
    double bestScore = std::numeric_limits<double>::lowest();
    Eigen::Vector3d nodePosition = uav_position_;

    std::vector<geometry_msgs::msg::PoseStamped> intents_copy;
    {
      std::lock_guard<std::mutex> lock(mutex_uav_intent_);
      intents_copy = uav_target_intents_;
    }

    const double exlusion_radius = 1.0 * _cellSize_;

    // Single UAV case: skip exclusion check since no other intents
    bool isSingleUAV = intents_copy.empty();

    // Min maxing -> Find node which has maximal minimal distance from each drone/their intents
    for (const shape_lib::ShapeNode& node : shapeNodes_){
      // Hard distance check from intents //{
      bool forbidden = false;
      if (!isSingleUAV) {  // Only check exclusion for multi-UAV
        for(const auto& intent : intents_copy) {
          Eigen::Vector3d other(
            intent.pose.position.x,
            intent.pose.position.y,
            intent.pose.position.z
          );

          if(ComputeDistance(node.position, other) < exlusion_radius){
            forbidden = true;
            break;
          }
        }
      }

      if(forbidden)
        continue;
      //}

      double minDist = std::numeric_limits<double>::max();

      // For single UAV, just use travel cost; for multi-UAV, check against intents
      if(isSingleUAV || uav_positions_.size() == intents_copy.size()){
        if(!isSingleUAV) {
          for(size_t i = 0; i < uav_positions_.size(); ++i){
            Eigen::Vector3d other(
            intents_copy[i].pose.position.x,
            intents_copy[i].pose.position.y,
            intents_copy[i].pose.position.z
            );

            double d = ComputeDistance(node.position, other);
            minDist = std::min(minDist, d);
          }
        } else {
          minDist = 0.0;  // Single UAV: no distance to check against others
        }

        double travelCost = ComputeDistance(uav_position_, node.position);
        //Can comment
        double score = (isSingleUAV) ? -travelCost : (minDist - TRAVEL_CONSTANT * travelCost);
        if(score > bestScore){
          bestScore = score;
          nodePosition = node.position;
        }
      }
    }

    //Has to be at least + threshold so it changes voxel (without it it just blinking between similar score nodes)
    double currentScore = ComputeScore(lastTarget_, intents_copy);
    if (bestScore > currentScore + _threshold_ || wasShapeReseted) {
      RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *clock_, 10.0,
      "[NEW NODE] [%.2f, %.2f, %.2f] bestScore (%.2f) > currentScore (%.2f) + Thr(%.2f)",
      targetPosition.x(), targetPosition.y(), targetPosition.z(), bestScore, currentScore, _threshold_
      );
      if(wasShapeReseted)
        wasShapeReseted = false;

      targetPosition = nodePosition;
      //previousScore_ = bestScore;
      lastTarget_ = nodePosition;
    } else {
      RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *clock_, 3.0,
      "[OLD NODE] [%.2f, %.2f, %.2f] prevScore (%.2f) + Thr(%.2f) > bestScore (%.2f)",
      lastTarget_.x(), lastTarget_.y(), lastTarget_.z(),
      currentScore, _threshold_, bestScore
      );

      targetPosition = lastTarget_;
    }

    PublishWaypoint(targetPosition);

    Eigen::Vector3d diff = targetPosition - uav_position_;
    double d = diff.norm();
    if (d < 1e-6)
      return Eigen::Vector3d::Zero();
    return diff;
  }

  void ShapeController::PublishWaypoint(Eigen::Vector3d target){
    geometry_msgs::msg::PoseStamped intentPose;
    intentPose.header.frame_id = _control_frame_;
    intentPose.header.stamp = clock_->now();
    intentPose.pose.position.x = target.x();
    intentPose.pose.position.y = target.y();
    intentPose.pose.position.z = target.z();
    uav_intent_publisher_.publish(intentPose);

    visualization_msgs::msg::Marker m;
    m.header.frame_id = "simulator_origin";
    m.header.stamp = clock_->now();
    m.ns = "shape_nodes";
    m.id = this_uav_idx_;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;

    // position
    m.pose.position.x = target.x();
    m.pose.position.y = target.y();
    m.pose.position.z = target.z();
    m.pose.orientation.w = 1.0;

    // size
    m.scale.x = _cellSize_;
    m.scale.y = _cellSize_;
    m.scale.z = _cellSize_;

    // color (simple heatmap)
    m.color.r = 1.0f;
    m.color.g = 0.0f;
    m.color.b = 0.0f;
    m.color.a = 0.8f;

    pub_target_marker_.publish(m);
    RCLCPP_INFO_THROTTLE(
            node_->get_logger(),
            *clock_, 2.0,
            "[TargetPosition]: [x=%.3f, y=%.3f, z=%.3f]",
            target.x(), target.y(), target.z()
    );
  }
  //}
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(shape_controller::ShapeController);
