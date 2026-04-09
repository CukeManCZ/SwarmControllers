#ifndef SWARM_CONTROLLERS_PKG__DATA_GATHERER_HPP_
#define SWARM_CONTROLLERS_PKG__DATA_GATHERER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/timer_handler.h>
#include <mrs_lib/service_server_handler.h>

#include <mrs_msgs/srv/reference_stamped_srv.hpp> 

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap_with_pose.hpp>

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <algorithm>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

#include <fstream>
#include <iomanip>
#include <filesystem>

/* using //{ */
using namespace std::chrono_literals;
//}

/* typedefs //{ */
#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif
//}

namespace data_gatherer
{
    class DataGatherer : public mrs_lib::Node
    {
        public:
        DataGatherer(rclcpp::NodeOptions options);

        private:
        rclcpp::Node::SharedPtr  node_;
        rclcpp::Clock::SharedPtr clock_;
        std::atomic<bool>        is_initialized_ = false;

        rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
        rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
        rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

        void initialize();
        void shutdown();

        // | ---------------------- param loader ---------------------- |
        std::shared_ptr<mrs_lib::ParamLoader> param_loader_;
        // | ----------------------- transformer ---------------------- |
        std::shared_ptr<mrs_lib::Transformer> transformer_;
        
        // | ---------------------- parameters ---------------------- |
        std::string _config_file_;
        std::string _file_path_;
        size_t n_drones_;
        std::vector<std::string> _uav_names_;
        std::string _odometry_topic_name_;
        std::string _feedback_topic_name_;
        std::string _reference_service_name_;
        double _rate_timer_data_;

        //| ----------------------- writing ------------------------- |
        bool octomapReceived_;
        bool newOctomap_;
        std::mutex mutex_octomap_received;
        std::chrono::steady_clock::time_point file_start_time_;

        std::ofstream csv_file_;
        int octomap_counter_;
        std::string run_folder_;

        //Odom
        std::mutex mutex_uav_odoms_;
        std::vector<mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>> uav_sub_odom_;
        void callbackOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg, size_t idx); 
        std::vector<Eigen::Vector3d> uav_positions_;
        std::vector<Eigen::Vector3d> uav_velocities_;

        //Reference
        std::vector<mrs_lib::ServiceServerHandler<mrs_msgs::srv::ReferenceStampedSrv>> uav_ss_reference_;
        bool callbackReference(const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request,
            const std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Response> response,
            size_t idx);
        std::vector<Eigen::Vector3d> uav_references_;

        //...
        //Feedback
        std::vector<mrs_lib::SubscriberHandler<std_msgs::msg::Float64>> uav_sub_feedback_;
        void callbackFeedback(const std_msgs::msg::Float64::ConstSharedPtr, size_t idx);
        std::vector<float> uav_feedbacks_;
        //...
        //Algo (wap, avoid)
        //...

        //Octomap -> Reseting after new incoming
        std::shared_ptr<octomap::OcTree> tree_;
        mrs_lib::SubscriberHandler<octomap_msgs::msg::OctomapWithPose> sub_octomap_shape_;
        void callbackOctomap(const octomap_msgs::msg::OctomapWithPose::ConstSharedPtr msg);

        //|---------------------- timers ----------------------|
        std::shared_ptr<TimerType> timer_data_write_;
        void timerDataWrite(void);

        //|----------------------- miscel ---------------------|
        bool IsInitialized(std::string functionName);
        void OpenNewCSV();
        void SaveOctomap();
    };
}
#endif