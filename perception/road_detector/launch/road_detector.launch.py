import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from common_python.launch_util import get_frame_ids_and_topic_names
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    PACKAGE_NAME = "road_detector"
    _, TOPIC_NAMES = get_frame_ids_and_topic_names()

    launch_args = (
        DeclareLaunchArgument(
            "weight_path",
            default_value=osp.join(get_package_share_directory("road_detector"), "weights", "End-to-end.pth"),
            description="Path to the weight pth file."),
        DeclareLaunchArgument(
            "use_device",
            default_value="0",
            description="cuda device, i.e. 0 or 0,1,2,3 or cpu",),
    )

    road_detector = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_perception",
        parameters=[
            # Overriding
            {
                "weight_path": LaunchConfiguration("weight_path"),
                "use_device": LaunchConfiguration("use_device"),
            },
        ],
        remappings=[
            ("sub_image",
             TOPIC_NAMES["sensing"]["zedx"]["left_image"]["undistorted"]),
            ("pub_mask_image",
             TOPIC_NAMES["perception"]["mask_image"]),
            ("pub_annotated_mask_image",
             TOPIC_NAMES["visualization"]["mask_image"]),
        ],
    )

    return LaunchDescription([
        *launch_args,
        road_detector,
    ])
