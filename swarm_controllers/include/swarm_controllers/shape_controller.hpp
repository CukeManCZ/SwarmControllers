#ifndef SWARM_CONTROLLERS_PKG__SHAPE_CONTROLLER_HPP_
#define SWARM_CONTROLLERS_PKG__SHAPE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/msg/tracker_command.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

//custom for settings
#include <myc_messages/msg/key_value_float.hpp>
#include <myc_messages/msg/key_value_float_array.hpp>

#include <mrs_lib/node.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/timer_handler.h>

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <algorithm>

#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap_with_pose.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <shape_lib/shape_builder.hpp>

//CV
#include <fstream>
#include <iomanip>

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

namespace shape_controller
{
  class ShapeController : public mrs_lib::Node  
  {
    public:
    ShapeController(rclcpp::NodeOptions options);

    private:
    //Initialization
    rclcpp::Node::SharedPtr node_;
    rclcpp::Clock::SharedPtr clock_;
    std::atomic<bool> is_initialized_ = false;

    rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
    rclcpp::CallbackGroup::SharedPtr cbkgrp_ss_;
    rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;
    rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;

    void initialize();
    void shutdown();
     
    // config definitions from file
    std::string _config_file_;
    int n_drones_;
    std::vector<std::string> _uav_names_;
    std::string _uav_name_;
    std::string _target_uav_name_;
    std::string _control_frame_;
    int this_uav_idx_;
    double _target_gain_; //?
    int _c_dimensions_;
    double _cellSize_;
    double _minHeight_;

    // set reference client
    mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv> cli_set_position_;
    std::shared_ptr<TimerType> timer_set_reference_;
    void SetReferenceCallback();
    double _rate_timer_set_reference_;
    double _rate_timer_default_;
    double rateScale;

    // diagnostic timer subscriber
    std::shared_ptr<TimerType> timer_diagnostics_;
    double _rate_timer_diagnostics_;
    bool all_robots_positions_valid_ = false;
    void TimerDiagnosticCallback();
    double _odom_timeout_; //??

    // activation trigger service
    mrs_lib::ServiceServerHandler<std_srvs::srv::Trigger> ser_activate_control_;
    void ActivateControlCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>, const std::shared_ptr<std_srvs::srv::Trigger::Response>); 
    bool control_allowed_ = false;

    // all UAV odometry information subscription
    std::mutex mutex_uav_odoms_;
    std::string _odometry_topic_name_;
    std::vector<mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>> other_uav_odom_subscribers_;
    void OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg, size_t idx); 
    std::vector<Eigen::Vector3d> uav_positions_;
    std::vector<Eigen::Vector3d> uav_velocities_;
    std::vector<rclcpp::Time> last_odom_msg_time_; 
    double _odom_msg_max_latency_;

    std::mutex mutex_uav_intent_;
    std::string _intent_topic_name_;
    std::vector<mrs_lib::SubscriberHandler<geometry_msgs::msg::PoseStamped>> other_uav_target_intents_subscribers_;
    void IntentCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg, size_t idx);
    std::vector<geometry_msgs::msg::PoseStamped> uav_target_intents_;
    mrs_lib::PublisherHandler<geometry_msgs::msg::PoseStamped> uav_intent_publisher_;
    Eigen::Vector3d lastTarget_;
    double previousScore_;
    double _threshold_;
    rclcpp::Time previousScoreTime_;

    //this UAV odometry
    mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> sub_odometry_;
    void ThisOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    Eigen::Vector3d uav_position_;
    Eigen::Vector3d uav_velocity_;

    // command for UAV
    std::mutex mutex_position_command_;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand> sh_position_command_;
    void GetPositionCmd();
    geometry_msgs::msg::Point position_command_;
    bool got_position_command_ = false;

    // octomap
    std::shared_ptr<octomap::OcTree> tree_;
    mrs_lib::SubscriberHandler<octomap_msgs::msg::OctomapWithPose> sub_octomap_shape_;
    void OctomapShapeCallback(const octomap_msgs::msg::OctomapWithPose::ConstSharedPtr msg);
    void InitOctoTree();
    bool wasShapeReseted;
    bool wasOctomapReceived;

    mrs_lib::PublisherHandler<octomap_msgs::msg::Octomap> pub_octomac_rviz_;

    // TODO: Use for precise drone control //{
    // waypoint
    mrs_lib::SubscriberHandler<geometry_msgs::msg::PointStamped> sub_waypoint_;
    void WaypointCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg);
    //}

    //Miscel
    std::shared_ptr<mrs_lib::Transformer> transformer_;
    shape_lib::ShapeBuilder shapeBuilder_;
    bool IsInitialized(std::string functionName);
    double ComputeDistance(Eigen::Vector3d, Eigen::Vector3d);
    
    // Shape controller
    Eigen::Vector3d GetNearestOccupiedVoxel();
    double DistanceToVoxel(const Eigen::Vector3d& p, const Eigen::Vector3d& center, double size);   
    Eigen::Vector3d CalculateUavToShapeRule();

    std::vector<shape_lib::ShapeNode> shapeNodes_; 
    mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray> pub_shape_markers_;
    void PublishShapeNodes(std::vector<shape_lib::ShapeNode>);

    size_t GetNearestNeighbour();
    Eigen::Vector3d GetNearestNeighbourPosition(); 
    Eigen::Vector3d GetNearestNeighbourVelocity(); 

    mrs_lib::PublisherHandler<visualization_msgs::msg::Marker> pub_target_marker_;
    void PublishWaypoint(Eigen::Vector3d target);
    Eigen::Vector3d CalculateReferenceAcceleration();
    Eigen::Vector3d CalculateAvoidanceRule();
    double ComputeScore(Eigen::Vector3d, std::vector<geometry_msgs::msg::PoseStamped>);

    double AVOIDANCE_CONSTANT;
    double WAYPOINT_CONSTANT;
    double AVOIDANCE_DISTANCE;
    double TRAVEL_CONSTANT;

    // Feedback
    mrs_lib::PublisherHandler<std_msgs::msg::Float64> pub_feedback_;
    std::shared_ptr<TimerType> timer_feedback_;
    double _rate_timer_feedback_;
    void TimerFeedbackCallback();
  };
}

#endif
