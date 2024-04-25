import os.path as osp
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
from launch_ros.actions import Node
from common_python.launch_util import get_frame_ids_and_topic_names, check_zedx_available_fps


def get_zedx_image_publisher(context):
    _, TOPIC_NAMES = get_frame_ids_and_topic_names()
    # Launch configuration variables
    grab_resolution_val = LaunchConfiguration("grab_resolution").perform(context)
    grab_frame_rate_val = LaunchConfiguration("grab_frame_rate").perform(context)
    is_valid_fps = check_zedx_available_fps(grab_resolution_val, grab_frame_rate_val)
    return (
        # ZED Wrapper node
        Node(
            package="zed_wrapper",
            namespace="/aiformula_sensing",
            executable="zed_wrapper",
            name="zedx_image_publisher",
            output="screen",
            condition=IfCondition(str(is_valid_fps)),
            parameters=[
                # YAML files
                LaunchConfiguration("config_common_path"),  # Common parameters
                LaunchConfiguration("config_camera_path"),  # Camera related parameters
                # Overriding
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "general.grab_resolution": LaunchConfiguration("grab_resolution"),
                    "general.grab_frame_rate": int(grab_frame_rate_val),
                },
            ],
            remappings=[
                ("~/left/image_rect_color",
                 TOPIC_NAMES["sensing"]["zedx"]["left_image"]["raw"]),
                ("~/right_raw/image_rect_color",
                 TOPIC_NAMES["sensing"]["zedx"]["right_image"]["raw"]),
            ],
        ),
    )


def generate_launch_description():
    launch_args = (
        DeclareLaunchArgument(
            "grab_resolution",
            default_value=TextSubstitution(text="HD1080"),
            description="The native camera grab resolution. HD1200, HD1080, SVGA",
            choices=["HD1200", "HD1080", "SVGA"]),
        DeclareLaunchArgument(
            "grab_frame_rate",
            default_value=TextSubstitution(text="60"),
            description="grabbing rate (HD1200/HD1080: 60, 30, 15 - SVGA: 120, 60, 30, 15)"),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Enable simulation time mode.",
            choices=["true", "false"]),
        DeclareLaunchArgument(
            "config_common_path",
            default_value=osp.join(get_package_share_directory("vehicle"), "config", "zedx", "common.yaml"),
            description="Path to the YAML configuration common file for the camera."),
        DeclareLaunchArgument(
            "config_camera_path",
            default_value=osp.join(get_package_share_directory("vehicle"), "config", "zedx", "zedx.yaml"),
            description="Path to the YAML configuration zedx file for the camera."),
    )
    zedx_image_publisher = OpaqueFunction(function=get_zedx_image_publisher)
    return LaunchDescription([
        SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
        *launch_args,
        zedx_image_publisher,
    ])
