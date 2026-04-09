#include "swarm_controllers/data_gatherer.hpp"

namespace data_gatherer {
    DataGatherer::DataGatherer(rclcpp::NodeOptions options)
        :mrs_lib::Node("data_gather_node", options)
    {
        this->initialize();
    };

    void DataGatherer::initialize(){
        node_ = this_node_ptr();
        clock_ = node_->get_clock();

        cbkgrp_subs_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cbkgrp_ss_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        rclcpp::on_shutdown([this]() {this->shutdown(); });

        // | ----------------------- load paramas -----------------|
        mrs_lib::ParamLoader param_loader(node_, "ShapeController");
        param_loader.loadParam("config", _config_file_);
        if(_config_file_ != ""){
        RCLCPP_INFO_STREAM(node_->get_logger(), "This is the path:" << _config_file_); 
        param_loader.addYamlFile(_config_file_);
        }

        param_loader.loadParam("uav_names", _uav_names_);
        param_loader.loadParam("odometry_topic", _odometry_topic_name_);
        param_loader.loadParam("reference_service_name", _reference_service_name_);
        param_loader.loadParam("feedback_topic", _feedback_topic_name_);
        param_loader.loadParam("file_path", _file_path_);
        param_loader.loadParam("set_reference_timer/rate", _rate_timer_data_);

        n_drones_ = _uav_names_.size();
        uav_positions_.resize(n_drones_, Eigen::Vector3d::Zero());
        uav_velocities_.resize(n_drones_, Eigen::Vector3d::Zero());
        uav_feedbacks_.resize(n_drones_, 0.0);
        uav_references_.resize(n_drones_, Eigen::Vector3d::Zero());

        // | --------------------- tf transformer --------------------- |
        /*
        transformer_ = std::make_shared<mrs_lib::Transformer>(node_);
        transformer_->setDefaultPrefix(_uav_name_);
        transformer_->retryLookupNewest(true);*/

        // | ---------------------- subscribers ---------------------|
        mrs_lib::SubscriberHandlerOptions shopts;
        shopts.node = node_;
        shopts.node_name  = "DataGatherer";
        shopts.no_message_timeout = mrs_lib::no_timeout;
        shopts.threadsafe = true;
        shopts.autostart = true;
        shopts.subscription_options.callback_group = cbkgrp_subs_;
    
        sub_octomap_shape_ = mrs_lib::SubscriberHandler<octomap_msgs::msg::OctomapWithPose>(shopts, "octomap",&DataGatherer::callbackOctomap, this);

        //Subscribe to UAVs //{
        for (size_t i = 0; i < _uav_names_.size(); i++) {
            //| ------------------------- odometry ---------------------|
            std::string topic_odometry_name = "/" + _uav_names_[i] + "/" + _odometry_topic_name_;
            uav_sub_odom_.push_back(
                mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(
                    shopts,
                    topic_odometry_name,
                    std::bind(&DataGatherer::callbackOdom, this, std::placeholders::_1, i)
                    )
            );
            RCLCPP_INFO(node_->get_logger(), "Subscribing to %s", topic_odometry_name.c_str());
            //| ------------------------- feedback ---------------------|
            std::string topic_feedback_name = "/" + _uav_names_[i] + "/" + _feedback_topic_name_;
            uav_sub_feedback_.push_back(
                mrs_lib::SubscriberHandler<std_msgs::msg::Float64>(
                    shopts,
                    topic_feedback_name,
                    std::bind(&DataGatherer::callbackFeedback, this, std::placeholders::_1, i)
                )
            );
            RCLCPP_INFO(node_->get_logger(), "Subscribing to %s", topic_feedback_name.c_str());
        }

        // | ---------------------- servers ---------------------|
        for(size_t i = 0; i < _uav_names_.size(); i++){
            std::string server_reference_name = "/" + _uav_names_[i] + "/" + _reference_service_name_;
            uav_ss_reference_.push_back(
                mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv>(
                    node_,
                    server_reference_name,
                    std::bind(&DataGatherer::callbackReference,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2,
                            i),
                    cbkgrp_ss_
                )
            );
            RCLCPP_INFO(node_->get_logger(), "Service active to %s", server_reference_name.c_str());
        }

        //|------------------------ timers --------------------------|
        mrs_lib::TimerHandlerOptions opts;
        opts.node = node_;
        opts.autostart = true;
        opts.callback_group = cbkgrp_timers_;

        {
            std::function<void()> callback_fcn = std::bind(&DataGatherer::timerDataWrite, this);

            timer_data_write_ = std::make_shared<TimerType>(opts, rclcpp::Rate(_rate_timer_data_, clock_), callback_fcn);
        }

        //|------------------------- writing -------------------------|
        octomapReceived_ = false;
        newOctomap_ = false;
        octomap_counter_ = 0;

        auto now = std::chrono::system_clock::now();
        std::time_t tt = std::chrono::system_clock::to_time_t(now);
        std::tm tm = *std::localtime(&tt);

        std::ostringstream ss;
        ss << "run_"
        << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");

        run_folder_ = _file_path_ + ss.str() + "/";

        std::filesystem::create_directories(run_folder_);

        RCLCPP_INFO(node_->get_logger(),
                    "Logging directory: %s",
                    run_folder_.c_str());

        is_initialized_ = true;
    }

    void DataGatherer::shutdown(){
        std::cout << "DataGatherer: shutdown(): called" << std::endl;
    }

    // | ---------------------- topic callbacks ------------------ |
    void DataGatherer::callbackOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg, size_t idx){
        if(!IsInitialized(__func__)) 
            return; 

        Eigen::Vector3d pos = Eigen::Vector3d(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z
        );

        Eigen::Vector3d vel = Eigen::Vector3d(
            msg->twist.twist.linear.x,
            msg->twist.twist.linear.y,
            msg->twist.twist.linear.z
        );

        uav_positions_[idx] = pos;
        uav_velocities_[idx] = vel;
    }

    void DataGatherer::callbackFeedback(const std_msgs::msg::Float64::ConstSharedPtr msg, size_t idx){
        if(!IsInitialized(__func__)) 
            return;

        uav_feedbacks_[idx] = msg->data;
    }

    void DataGatherer::callbackOctomap(const octomap_msgs::msg::OctomapWithPose::ConstSharedPtr msg){
        if(!IsInitialized(__func__)) 
            return; 

        if (!msg) 
            return;

        auto* abstract_tree = octomap_msgs::msgToMap(msg->octomap);
        if (!abstract_tree) return;

        auto* octree = dynamic_cast<octomap::OcTree*>(abstract_tree);
        if (!octree) {
            delete abstract_tree;
            return;
        }

        tree_.reset(octree);
        SaveOctomap();
        {
            std::lock_guard<std::mutex> lock(mutex_octomap_received);
            octomapReceived_ = true;
            newOctomap_ = true;
        }
    }
    
    // | ---------------------- service callbacks ------------------- |
    bool DataGatherer::callbackReference(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request,
    const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response,
        size_t idx){
            return true;
    }

    // | ------------------------- timer callbacks --------------------- |
    void DataGatherer::timerDataWrite(){
        if(!IsInitialized(__func__)) 
            return;

          bool start_new_file = false;

        {
            std::lock_guard<std::mutex> lock(mutex_octomap_received);

            if (newOctomap_) {
                octomap_counter_++;
                start_new_file = true;

                newOctomap_ = false;
            }

            if (!octomapReceived_)
            return;   // still waiting for first octomap
        }

        // Open new file if needed
        if (start_new_file) {
            OpenNewCSV();
        }

        if (!csv_file_.is_open())
            return;

        double t = std::chrono::duration<double>
            (std::chrono::steady_clock::now() - file_start_time_).count();

        for (size_t i = 0; i < n_drones_; i++) {

            const auto& p = uav_positions_[i];
            const auto& v = uav_velocities_[i];
            double fb = uav_feedbacks_[i];
            const auto& r = uav_references_[i];

            csv_file_
            << t << ","
            << i << ","
            << p.x() << "," << p.y() << "," << p.z() << ","
            << v.x() << "," << v.y() << "," << v.z() << ","
            << fb << ","
            << r.x() << "," << r.y() << "," << r.z()
            << "\n";
        }

        csv_file_.flush();
    }

    //|--------------------------------miscel ----------------------------|
    bool DataGatherer::IsInitialized(std::string functionName){
        if (!is_initialized_){
            RCLCPP_INFO_STREAM(node_->get_logger(), "Node not initialized" << "| Call from: |" << functionName);
            return false; 
        }
        return true;
    }

    void DataGatherer::OpenNewCSV()
    {
        if (csv_file_.is_open())
            csv_file_.close();

        std::string filename = run_folder_ + std::to_string(octomap_counter_) + ".csv";

        csv_file_.open(filename);
        file_start_time_ = std::chrono::steady_clock::now();
        // header
        csv_file_ <<
            "time,"
            "uav_id,"
            "px,py,pz,"
            "vx,vy,vz,"
            "feedback,"
            "ref_x,ref_y,ref_z\n";

        RCLCPP_INFO(node_->get_logger(),"Opened CSV: %s", filename.c_str());
    }

    void DataGatherer::SaveOctomap()
    {
        if (!tree_) return;

        std::string filename = run_folder_ + "octomap_" + std::to_string(octomap_counter_) + ".bt";

        if (tree_->writeBinary(filename)) {
            RCLCPP_INFO(node_->get_logger(), "Saved Octomap: %s", filename.c_str());
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to save Octomap!");
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(data_gatherer::DataGatherer);