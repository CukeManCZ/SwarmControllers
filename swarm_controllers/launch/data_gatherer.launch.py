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

    use_sim_time=os.getenv("USE_SIM_TIME", "false") == "true"
   

    data_gatherer_node = ComposableNode(
                    package=pkg_name,
                    plugin="data_gatherer::DataGatherer",
                    name="DataGatherer",
                    namespace="",
                    parameters=[
                        {"enable_profiler": False},
                        {"use_sim_time":use_sim_time},
                        {"config": this_pkg_path + "/config/dataGatherer.yaml"}
                    ]
    )
  
    standalone_container = ComposableNodeContainer(
            namespace="DataGatherer", 
            name=namespace+"_container",
            package="rclcpp_components",
            executable="component_container_mt", 
            output="screen",
            composable_node_descriptions=[data_gatherer_node],
    )

    ld.add_action(standalone_container)

    return ld
