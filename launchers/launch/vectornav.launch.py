import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    PACKAGE_NAME = "vectornav"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)
    TOPIC_NS = "/aiformula_sensing/vectornav"
    _, TOPIC_NAMES = get_frame_ids_and_topic_names()

    ROS_PARAM_CONFIG = osp.join(PACKAGE_DIR, "config", "vectornav.yaml")
    vectornav = Node(
        package=PACKAGE_NAME,
        executable="vectornav",
        name="vectornav",
        namespace="/aiformula_sensing",
        parameters=[ROS_PARAM_CONFIG],
        remappings=[
            ("vectornav/raw/common", TOPIC_NS + "/raw/common"),
            ("vectornav/raw/time", TOPIC_NS + "/raw/time"),
            ("vectornav/raw/imu", TOPIC_NS + "/raw/imu"),
            ("vectornav/raw/gps", TOPIC_NS + "/raw/gps"),
            ("vectornav/raw/attitude", TOPIC_NS + "/raw/attitude"),
            ("vectornav/raw/ins", TOPIC_NS + "/raw/ins"),
            ("vectornav/raw/gps2", TOPIC_NS + "/raw/gps2"),
        ],
    )
    vn_sensor_msgs = Node(
        package=PACKAGE_NAME,
        executable="vn_sensor_msgs",
        name="vn_sensor_msgs",
        namespace="/aiformula_sensing",
        parameters=[ROS_PARAM_CONFIG],
        remappings=[
            ("vectornav/raw/common", TOPIC_NS + "/raw/common"),
            ("vectornav/raw/time", TOPIC_NS + "/raw/time"),
            ("vectornav/raw/imu", TOPIC_NS + "/raw/imu"),
            ("vectornav/raw/gps", TOPIC_NS + "/raw/gps"),
            ("vectornav/raw/attitude", TOPIC_NS + "/raw/attitude"),
            ("vectornav/raw/ins", TOPIC_NS + "/raw/ins"),
            ("vectornav/raw/gps2", TOPIC_NS + "/raw/gps2"),

            ("vectornav/time_startup", TOPIC_NS + "/time_startup"),
            ("vectornav/time_gps", TOPIC_NS + "/time_gps"),
            ("vectornav/time_syncin", TOPIC_NS + "/time_syncin"),
            ("vectornav/time_pps", TOPIC_NS + "/time_pps"),
            ("vectornav/imu", TOPIC_NAMES["sensing"]["imu"]),
            ("vectornav/gnss", TOPIC_NS + "/gnss"),
            ("vectornav/imu_uncompensated", TOPIC_NS + "/imu_uncompensated"),
            ("vectornav/magnetic", TOPIC_NS + "/magnetic"),
            ("vectornav/temperature", TOPIC_NS + "/temperature"),
            ("vectornav/pressure", TOPIC_NS + "/pressure"),
            ("vectornav/velocity_body", TOPIC_NS + "/velocity_body"),
            ("vectornav/pose", TOPIC_NS + "/pose"),
        ]
    )

    return LaunchDescription([
        vectornav,
        vn_sensor_msgs,
    ])
