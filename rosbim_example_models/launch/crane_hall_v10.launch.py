import os

import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_path
from lifecycle_msgs.msg import Transition, State
from launch.actions import EmitEvent
from launch.actions import EmitEvent
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg


def launch_setup(context, *args, **kwargs):
    log_level = LaunchConfiguration("log_level").perform(context)
    config_file = LaunchConfiguration("bim_file")
    rviz_file = LaunchConfiguration("rviz_file").perform(context)

    rosbim_manager = Node(
        package="rosbim_manager",
        executable="rosbim_manager.py",
        output="screen",
        parameters=[{"bim_file": config_file}],
        arguments=["--ros-args", "--log-level", log_level],
        emulate_tty=True,
    )
    spawner_export_geometries = Node(
        package="rosbim_manager",
        executable="spawner.py",
        output="screen",
        arguments=["--name", "rosbim_export_geometry_plugin"],
        emulate_tty=True,
    )
    spawner_export_map = Node(
        package="rosbim_manager",
        executable="spawner.py",
        output="screen",
        arguments=["--name", "rosbim_export_map_plugin"],
        emulate_tty=True,
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[
            "src/rosbim_example_models/config/map_server.yaml",
        ],
        emulate_tty=True,
    )

    rviz2 = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                output="both",
                arguments=["-d", rviz_file],
            )
        ],
    )

    map_server_trans_event = TimerAction(
        period=4.0,
        actions=[
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=launch.events.process.matches_executable(
                        map_server
                    ),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )
            )
        ],
    )

    return [
        rosbim_manager,
        spawner_export_geometries,
        spawner_export_map,
        map_server,
        rviz2,
        map_server_trans_event,
    ]


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
            DeclareLaunchArgument(
                "rviz_file",
                default_value="/rosbim/src/rosbim_example_models/config/rosbim.rviz",
                description="Rviz GUI to load.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
