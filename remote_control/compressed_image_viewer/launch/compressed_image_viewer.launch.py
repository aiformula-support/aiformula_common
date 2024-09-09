from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    PACKAGE_NAME = "compressed_image_viewer"
    _, TOPIC_NAMES = get_frame_ids_and_topic_names()

    launch_args = (
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logger level (debug, info, warn, error, fatal)",
        ),
    )

    compressed_image_viewer = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_visualization/" + PACKAGE_NAME + "/",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        remappings=[
            ("sub_compressed_image", TOPIC_NAMES["visualization"]["aiformula_pilot"]),
        ],
    )

    return LaunchDescription([
        *launch_args,
        compressed_image_viewer,
    ])
