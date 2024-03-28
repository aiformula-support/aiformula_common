from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    PACKAGE_NAME = "zedx_image_publisher"
    launch_args = (
        DeclareLaunchArgument(
            "pub_left_image",
            default_value="/aiformula_sensing/zedx_image_publisher/left_image_raw",
            description="Topic name of ZED left camera raw image",
        ),
        DeclareLaunchArgument(
            "pub_left_compressed_image",
            default_value="/aiformula_sensing/zedx_image_publisher/left_image_raw/compressed",
            description="Topic name of ZED left camera compressed image",
        ),
        DeclareLaunchArgument(
            "zed_left_frame_id",
            default_value="zedX_left_link",
            description="Frame id of ZED left camera",
        ),
    )

    nodes = (
        Node(
            package=PACKAGE_NAME,
            executable=PACKAGE_NAME,
            name=PACKAGE_NAME,
            namespace="/aiformula_sensing",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "zed_left_frame_id": LaunchConfiguration("zed_left_frame_id"),
            }],
            remappings=[
                ("pub_left_image", LaunchConfiguration("pub_left_image")),
                ("pub_left_compressed_image", LaunchConfiguration(
                    "pub_left_compressed_image")),
            ],
        ),
    )
    return LaunchDescription([
        *launch_args,
        *nodes,
    ])
