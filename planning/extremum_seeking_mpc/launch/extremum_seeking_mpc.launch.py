import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from common_python.launch_util import get_frame_ids_and_topic_names

def generate_launch_description():
    PACKAGE_NAME = "extremum_seeking_mpc"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)
    _, TOPIC_NAMES = get_frame_ids_and_topic_names()

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
        parameters=[*ROS_PARAM_CONFIG],
        remappings=[
            ("pub_speed_command", TOPIC_NAMES["control"]["speed_command"]["game_pad"]),
            ('sub_road_l',TOPIC_NAMES["planning"]["road_pointcloud"]["left"]),
            ('sub_road_r',TOPIC_NAMES["planning"]["road_pointcloud"]["right"]),
        ],
    )

    return LaunchDescription([
        extremum_seeking_mpc,
    ])