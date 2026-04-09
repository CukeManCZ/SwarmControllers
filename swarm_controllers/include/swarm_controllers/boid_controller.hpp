#ifndef SWARM_CONTROLLERS_PKG__BOID_CONTROLLER_HPP_
#define SWARM_CONTROLLERS_PKG__BOID_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/msg/tracker_command.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

//custom for settings
#include <myc_messages/msg/key_value_float.hpp>
#include <myc_messages/msg/key_value_float_array.hpp>

#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <algorithm>

namespace boid_controller
{
  class BoidController : public rclcpp::Node  
  {
    public:
    BoidController(rclcpp::NodeOptions options);

    private:
    //Initialization
    rclcpp::Node::SharedPtr node_;
    bool is_initialized_ = false;
    rclcpp::TimerBase::SharedPtr init_timer_;
    void InitializationCallback();
     
    // config definitions from file
    std::string _config_file_;
    int n_drones_;
    std::vector<std::string> _uav_names_;
    std::string _uav_name_;
    std::string _target_uav_name_;
    std::string _control_frame_;
    int this_uav_idx_;
    double _target_gain_; //?
    int _c_dimensions_; //?
    
    // set reference client
    rclcpp::Client<mrs_msgs::srv::ReferenceStampedSrv>::SharedPtr cli_set_position_;
    rclcpp::TimerBase::SharedPtr timer_set_reference_;
    void SetReferenceCallback();
    double _rate_timer_set_reference_;

    // diagnostic timer subscriber
    rclcpp::TimerBase::SharedPtr timer_diagnostics_;
    double _rate_timer_diagnostics_;
    bool all_robots_positions_valid_ = false;
    void TimerDiagnosticCallback();
    double _odom_timeout_; //??

    // activation trigger service
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ser_activate_control_;
    void ActivateControlCallback([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request>, const std::shared_ptr<std_srvs::srv::Trigger::Response>); 
    bool control_allowed_ = false;

    // all UAV odometry information subscription
    std::mutex mutex_uav_odoms_;
    std::string _odometry_topic_name_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> other_uav_odom_subscribers_;
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, size_t idx); 
    std::vector<Eigen::Vector3d> uav_positions_;
    std::vector<Eigen::Vector3d> uav_velocities_;
    std::vector<rclcpp::Time> last_odom_msg_time_; 
    double _odom_msg_max_latency_;

    //this UAV odometry
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
    void ThisOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    Eigen::Vector3d uav_position_;
    Eigen::Vector3d uav_velocity_;

    // command for UAV
    std::mutex mutex_position_command_;
    mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand> sh_position_command_;
    void GetPositionCmd();
    geometry_msgs::msg::Point position_command_;
    bool got_position_command_ = false;


    // Boids setting //{
    // waypoint
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_waypoint_;
    void WaypointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    // centroid
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_centroid_;
    rclcpp::Subscription<myc_messages::msg::KeyValueFloatArray>::SharedPtr sub_boid_constants_;
    void BoidConstantsCallback(const myc_messages::msg::KeyValueFloatArray::SharedPtr msg);
    //}

    //Miscel
    std::shared_ptr<mrs_lib::Transformer> transformer_;
    bool IsInitialized(std::string functionName);
    double ComputeDistance(Eigen::Vector3d, Eigen::Vector3d);
    
    //Boid controlled
    Eigen::Vector3d GetBoidCenteroid();  
    size_t GetNearestNeighbour();
    Eigen::Vector3d GetNearestNeighbourPosition(); 
    Eigen::Vector3d GetNearestNeighbourVelocity(); 

    Eigen::Vector3d CalculateReferenceAcceleration();
    Eigen::Vector3d CalculateFlockingRule();
    Eigen::Vector3d CalculateAvoidanceRule();
    Eigen::Vector3d CalculateVelocityMatchRule();
    Eigen::Vector3d CalculateWaypointRule();

    double FLOCKING_CONSTANT;
    double AVOIDANCE_CONSTANT;
    double VELOCITY_M_CONSTANT;
    double WAYPOINT_CONSTANT;
  };
}

#endif
