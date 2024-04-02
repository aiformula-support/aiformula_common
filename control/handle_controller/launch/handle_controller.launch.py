import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    PACKAGE_NAME = "handle_controller"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)

    launch_args = (
        DeclareLaunchArgument(
            "pub_handle_controller",
            default_value="/cmd_vel",
            description="handle controller topic name",
        ),

        DeclareLaunchArgument(
            "sub_joy_frame",
            default_value="/joy",
            description="Joy topic name",
        ),

        DeclareLaunchArgument(
            "handle_controller_frame_id",
            default_value="handle_controller",
            description="Frame id of handle_controller",
        ),

        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0'
        ),
    )

    ROS_PARAM_CONFIG = os.path.join(PACKAGE_DIR, "config", "handle_controller.yaml")
    nodes = (
        Node(
            package=PACKAGE_NAME,
            executable=PACKAGE_NAME,
            name=PACKAGE_NAME,
            namespace="/aiformula_control",
            output="screen",
            emulate_tty=True,
            parameters=[ROS_PARAM_CONFIG,
                        {
                            "handle_controller_frame_id": LaunchConfiguration("handle_controller_frame_id"),
                        }],
            remappings=[
                ("sub_joy_frame", LaunchConfiguration("sub_joy_frame")),
                ("pub_handle_controller", LaunchConfiguration("pub_handle_controller")),
            ],
        ),

        Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': LaunchConfiguration("joy_dev"),
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]
        ),
    )

    return LaunchDescription([
        *launch_args,
        *nodes,
    ])
