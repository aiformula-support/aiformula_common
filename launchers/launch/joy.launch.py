import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    PACKAGE_NAME = "joy"
    NODE_NAME = "joy_node"
    launch_args = (
        DeclareLaunchArgument(
            "pub_game_pad_output",
            default_value="/aiformula_control/joy_node/joy",
            description="Topic name of GamePad output.",
        ),
    )

    ROS_PARAM_CONFIG = (
        osp.join(get_package_share_directory(
            "launchers"), "config", "joy.yaml"),
    )
    joy_node = Node(
        package=PACKAGE_NAME,
        executable=NODE_NAME,
        name=NODE_NAME,
        namespace="/aiformula_control",
        parameters=[*ROS_PARAM_CONFIG],
        remappings=[
            ("joy", LaunchConfiguration("pub_game_pad_output")),
        ],
    )

    return LaunchDescription([
        *launch_args,
        joy_node,
    ])
