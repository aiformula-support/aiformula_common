import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    PACKAGE_NAME = "extremum_seeking_mpc"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)
    FRAME_IDS, TOPIC_NAMES = get_frame_ids_and_topic_names()

    ROS_PARAM_CONFIG = (
        osp.join(PACKAGE_DIR, "config", "extremum_seeking_mpc_params.yaml"),
    )

    extremum_seeking_mpc = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_planning",
        output="screen",
        emulate_tty=True,
        parameters=[*ROS_PARAM_CONFIG,
                    {"base_footprint_frame_id": FRAME_IDS["base_footprint"], },],
        remappings=[
            ('pub_twist_command', TOPIC_NAMES["planning"]["twist_command"]),
            ('sub_road_l', TOPIC_NAMES["perception"]["lane_lines"]["left"]),
            ('sub_road_r', TOPIC_NAMES["perception"]["lane_lines"]["right"]),
            ('sub_actual_speed', TOPIC_NAMES["sensing"]["actual_speed"]),
            ('sub_object_info', TOPIC_NAMES["perception"]["objects"]["info"])
        ],
    )

    return LaunchDescription([
        extremum_seeking_mpc,
    ])
