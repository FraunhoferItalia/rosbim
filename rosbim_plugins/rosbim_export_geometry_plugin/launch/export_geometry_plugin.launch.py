import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    log_level = LaunchConfiguration("log_level").perform(context)
    config_file = LaunchConfiguration("bim_file")

    rosbim_manager = Node(
        package="rosbim_manager",
        executable="rosbim_manager.py",
        output="screen",
        parameters=[{"bim_file": config_file}],
        arguments=["--ros-args", "--log-level", log_level],
        emulate_tty=True,
    )
    spawner = Node(
        package="rosbim_manager",
        executable="spawner.py",
        output="screen",
        arguments=["--name", "rosbim_export_geometry_plugin"],
        emulate_tty=True,
    )

    return [rosbim_manager, spawner]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level",
            ),
            DeclareLaunchArgument(
                "bim_file",
                default_value="/rosbim/src/rosbim_example_models/models/crane_hall_v10.ifc",
                description="Absolute path of the bim file",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
