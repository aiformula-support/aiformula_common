import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    PACKAGE_NAME = "odometry_publisher"
    NODE_NAME = "gyro_odometry"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)
    launch_args = (
        DeclareLaunchArgument(
            "sub_imu",
            default_value="/vectornav/imu",
            # default_value="/aiformula_sensing/vectornav/imu",
            description="Imu topic name",
        ),
        DeclareLaunchArgument(
            "sub_can_frame",
            default_value="/from_can_bus",
            # default_value="/aiformula_sensing/can/frame",
            description="Can topic name",
        ),
        DeclareLaunchArgument(
            "pub_odometry",
            default_value="/aiformula_sensing/gyro_odometry/odom",
            description="Odometry topic name.",
        ),
        DeclareLaunchArgument(
            "use_rosbag",
            default_value="false",
            description="If true, play rosbag",
        ),
        DeclareLaunchArgument(
            "rosbag_path",
            default_value="~/data/aiformula/20240328_shiho_can_imu_mag_gnss/test_01",
            description="Path of rosbag to play",
        ),
        DeclareLaunchArgument(
            "rosbag_play_speed",
            default_value="3.0",
            description="Speed to play rosbag",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="If true, run rviz2",
        ),
        DeclareLaunchArgument(
            "odom_frame_id",
            default_value="odom",
            description="Frame id of odom",
        ),
        DeclareLaunchArgument(
            "robot_frame_id",
            default_value="base_footprint",
            description="Frame id of the robot",
        ),
    )

    ROS_PARAM_CONFIG = os.path.join(PACKAGE_DIR, "config", "wheel.yaml")
    nodes = (
        Node(
            package=PACKAGE_NAME,
            executable=NODE_NAME,
            name=NODE_NAME,
            namespace="/aiformula_sensing",
            output="screen",
            emulate_tty=True,
            parameters=[ROS_PARAM_CONFIG,
                        {
                            "odom_frame_id": LaunchConfiguration("odom_frame_id"),
                            "robot_frame_id": LaunchConfiguration("robot_frame_id"),
                        }],
            remappings=[
                ("sub_imu", LaunchConfiguration("sub_imu")),
                ("sub_can_frame", LaunchConfiguration("sub_can_frame")),
                ("pub_odometry", LaunchConfiguration("pub_odometry")),
            ],
        ),

        # ros2 bag play
        ExecuteProcess(
            cmd=[
                "ros2 bag play",
                " --topics ",
                LaunchConfiguration("sub_imu"),
                LaunchConfiguration("sub_can_frame"),
                " -r ",
                LaunchConfiguration("rosbag_play_speed"),
                " -- ",
                LaunchConfiguration("rosbag_path"),
            ],
            condition=IfCondition(LaunchConfiguration("use_rosbag")),
            shell=True,
        ),

        # rviz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_gyro_odometry",
            arguments=[
                "-d", os.path.join(PACKAGE_DIR, "rviz", NODE_NAME + ".rviz")],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    )
    return LaunchDescription([
        *launch_args,
        *nodes,
    ])
