import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("direct_joy"), "config", "direct_joy.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("start_joy", default_value="true"),
        DeclareLaunchArgument("device_id", default_value="0"),
        DeclareLaunchArgument("joy_topic", default_value="/joy1/joy"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("params_file", default_value=config),
        DeclareLaunchArgument("mode_controls", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("mode_service_prefix", default_value="/tron2_control_mode"),
        Node(
            package="joy",
            executable="joy_node",
            name="joy",
            namespace="joy1",
            condition=IfCondition(LaunchConfiguration("start_joy")),
            parameters=[{
                "device_id": ParameterValue(
                    LaunchConfiguration("device_id"), value_type=int
                ),
                "autorepeat_rate": 20.0,
            }],
            remappings=[("joy", LaunchConfiguration("joy_topic"))],
            output="screen",
        ),
        Node(
            package="direct_joy",
            executable="direct_joy_node",
            name="direct_joy",
            parameters=[
                LaunchConfiguration("params_file"),
                {
                    "mode_controls": ParameterValue(
                        LaunchConfiguration("mode_controls"), value_type=bool
                    ),
                    "mode_service_prefix": LaunchConfiguration("mode_service_prefix"),
                },
            ],
            remappings=[
                ("joy1/joy", LaunchConfiguration("joy_topic")),
                ("cmd_vel", LaunchConfiguration("cmd_vel_topic")),
            ],
            output="screen",
        ),
    ])
