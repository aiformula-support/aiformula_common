import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    PACKAGE_NAME = "vectornav"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)
    TOPIC_NS = "/aiformula_sensing/vectornav"
    launch_args = (
        DeclareLaunchArgument(
            "pub_raw_common", default_value=TOPIC_NS + "/raw/common"),
        DeclareLaunchArgument(
            "pub_raw_time", default_value=TOPIC_NS + "/raw/time"),
        DeclareLaunchArgument(
            "pub_raw_imu", default_value=TOPIC_NS + "/raw/imu"),
        DeclareLaunchArgument(
            "pub_raw_gps", default_value=TOPIC_NS + "/raw/gps"),
        DeclareLaunchArgument(
            "pub_raw_attitude", default_value=TOPIC_NS + "/raw/attitude"),
        DeclareLaunchArgument(
            "pub_raw_ins", default_value=TOPIC_NS + "/raw/ins"),
        DeclareLaunchArgument(
            "pub_raw_gps2", default_value=TOPIC_NS + "/raw/gps2"),

        DeclareLaunchArgument(
            "pub_time_startup", default_value=TOPIC_NS + "/time_startup"),
        DeclareLaunchArgument(
            "pub_time_gps", default_value=TOPIC_NS + "/time_gps"),
        DeclareLaunchArgument(
            "pub_time_syncin", default_value=TOPIC_NS + "/time_syncin"),
        DeclareLaunchArgument(
            "pub_time_pps", default_value=TOPIC_NS + "/time_pps"),
        DeclareLaunchArgument(
            "pub_imu", default_value=TOPIC_NS + "/imu"),
        DeclareLaunchArgument(
            "pub_gnss", default_value=TOPIC_NS + "/gnss"),
        DeclareLaunchArgument(
            "pub_imu_uncompensated", default_value=TOPIC_NS + "/imu_uncompensated"),
        DeclareLaunchArgument(
            "pub_magnetic", default_value=TOPIC_NS + "/magnetic"),
        DeclareLaunchArgument(
            "pub_temperature", default_value=TOPIC_NS + "/temperature"),
        DeclareLaunchArgument(
            "pub_pressure", default_value=TOPIC_NS + "/pressure"),
        DeclareLaunchArgument(
            "pub_velocity_body", default_value=TOPIC_NS + "/velocity_body"),
        DeclareLaunchArgument(
            "pub_pose", default_value=TOPIC_NS + "/pose"),

    )

    ROS_PARAM_CONFIG = osp.join(PACKAGE_DIR, "config", "vectornav.yaml")
    vectornav = Node(
        package=PACKAGE_NAME,
        executable="vectornav",
        name="vectornav",
        namespace="/aiformula_sensing",
        parameters=[ROS_PARAM_CONFIG],
        remappings=[
            ("vectornav/raw/common", LaunchConfiguration("pub_raw_common")),
            ("vectornav/raw/time", LaunchConfiguration("pub_raw_time")),
            ("vectornav/raw/imu", LaunchConfiguration("pub_raw_imu")),
            ("vectornav/raw/gps", LaunchConfiguration("pub_raw_gps")),
            ("vectornav/raw/attitude", LaunchConfiguration("pub_raw_attitude")),
            ("vectornav/raw/ins", LaunchConfiguration("pub_raw_ins")),
            ("vectornav/raw/gps2", LaunchConfiguration("pub_raw_gps2")),
        ],
    )
    vn_sensor_msgs = Node(
        package=PACKAGE_NAME,
        executable="vn_sensor_msgs",
        name="vn_sensor_msgs",
        namespace="/aiformula_sensing",
        parameters=[ROS_PARAM_CONFIG],
        remappings=[
            ("vectornav/raw/common", LaunchConfiguration("pub_raw_common")),
            ("vectornav/raw/time", LaunchConfiguration("pub_raw_time")),
            ("vectornav/raw/imu", LaunchConfiguration("pub_raw_imu")),
            ("vectornav/raw/gps", LaunchConfiguration("pub_raw_gps")),
            ("vectornav/raw/attitude", LaunchConfiguration("pub_raw_attitude")),
            ("vectornav/raw/ins", LaunchConfiguration("pub_raw_ins")),
            ("vectornav/raw/gps2", LaunchConfiguration("pub_raw_gps2")),

            ("vectornav/time_startup", LaunchConfiguration("pub_time_startup")),
            ("vectornav/time_gps", LaunchConfiguration("pub_time_gps")),
            ("vectornav/time_syncin", LaunchConfiguration("pub_time_syncin")),
            ("vectornav/time_pps", LaunchConfiguration("pub_time_pps")),
            ("vectornav/imu", LaunchConfiguration("pub_imu")),
            ("vectornav/gnss", LaunchConfiguration("pub_gnss")),
            ("vectornav/imu_uncompensated",
             LaunchConfiguration("pub_imu_uncompensated")),
            ("vectornav/magnetic", LaunchConfiguration("pub_magnetic")),
            ("vectornav/temperature", LaunchConfiguration("pub_temperature")),
            ("vectornav/pressure", LaunchConfiguration("pub_pressure")),
            ("vectornav/velocity_body", LaunchConfiguration("pub_velocity_body")),
            ("vectornav/pose", LaunchConfiguration("pub_pose")),
        ]
    )

    return LaunchDescription([
        *launch_args,
        vectornav,
        vn_sensor_msgs,
    ])
