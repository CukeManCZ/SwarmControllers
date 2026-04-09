import launch
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = launch.LaunchDescription()
    
    pkg_name = "swarm_controllers"
    this_pkg_path = get_package_share_directory(pkg_name) 
    namespace = "swarm_controllers"

    uav_name = os.getenv("UAV_NAME", "uav1")
    use_sim_time=os.getenv("USE_SIM_TIME", "false") == "true"
   

    boid_controller_node = ComposableNode(
                    package=pkg_name,
                    plugin="boid_controller::BoidController",
                    name="BoidController",
                    namespace=uav_name + "/",
                    parameters=[
                        {"uav_name": uav_name},
                        {"topic_prefix":"/"+uav_name},
                        {"enable_profiler": False},
                        {"use_sim_time":use_sim_time},
                        {"control_frame":uav_name + "/world_origin"},
                        {"config": this_pkg_path + "/config/boid.yaml"}
                    ],
                    remappings=[
                        ("tracker_cmd_in", "control_manager/tracker_cmd"),
                        ("ref_pos_out", "control_manager/reference"),
                        ("control_activation_in", "activation"),
                    ]
    )
  
    standalone_container = ComposableNodeContainer(
            namespace=uav_name, 
            name=namespace+"_container",
            package="rclcpp_components",
            executable="component_container_mt", 
            output="screen",
            composable_node_descriptions=[boid_controller_node],
    )

    ld.add_action(standalone_container)

    return ld
