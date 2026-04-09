from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    UAV_NAME = LaunchConfiguration("UAV_NAME")
    global_frame = LaunchConfiguration("global_frame")

    declared_arguments = [
        DeclareLaunchArgument(
            "UAV_NAME",
            default_value=EnvironmentVariable("UAV_NAME", default_value="uav1")
        ),
        DeclareLaunchArgument("global_frame", default_value="gps_garmin_origin"),
    ]

    # List of UAVs to connect
    uav_list = ["uav1", "uav2", "uav3"]

    nodes = []
    for other in uav_list:
        if other == "uav1":
            continue

        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=[UAV_NAME, "_to_", other],
                arguments=[
                    "--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", "0.0",
                    "--frame-id", [UAV_NAME, "/", global_frame],
                    "--child-frame-id", [other, "/", global_frame],
                ]
            )
        )

    return LaunchDescription(declared_arguments + nodes)

