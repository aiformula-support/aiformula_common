import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    PACKAGE_NAME = "teleop_twist_joy"
    NODE_NAME = "teleop_node"
    launch_args = (
        DeclareLaunchArgument(
            "sub_game_pad_output",
            default_value="/aiformula_control/joy_node/joy",
            description="Topic name of GamePad output.",
        ),
        DeclareLaunchArgument(
            "pub_speed_command",
            default_value="/aiformula_control/game_pad/cmd_vel",
            description="Topic name of speed command.",
        ),
        DeclareLaunchArgument(
            "game_pad",
            default_value="dualshock4",
            description="GamePad type.",
        ),
        DeclareLaunchArgument(
            "button_layout_config",
            default_value=[
                osp.join(get_package_share_directory("launchers"),
                         "config/gamepad", ""), LaunchConfiguration("game_pad"), ".yaml"
            ],
            description="GamePad button layout configuration file.",
        ),
    )

    nodes = (
        Node(
            package=PACKAGE_NAME,
            executable=NODE_NAME,
            name=NODE_NAME,
            namespace="/aiformula_control",
            parameters=[LaunchConfiguration("button_layout_config")],
            remappings=[
                ("joy", LaunchConfiguration("sub_game_pad_output")),
                ("cmd_vel", LaunchConfiguration("pub_speed_command")),
            ],
        ),
    )
    return LaunchDescription([
        *launch_args,
        *nodes,
    ])
