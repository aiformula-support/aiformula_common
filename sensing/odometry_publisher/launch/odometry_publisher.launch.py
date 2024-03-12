import os
from typing import Tuple
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def get_bag_play_node(
        context: LaunchContext,
        use_rosbag_lc: LaunchConfiguration,
        rosbag_path_lc: LaunchConfiguration,
        can_frame_topic_name_lc: LaunchConfiguration
    ) -> Tuple[Node]:
    can_frame_topic_name = context.perform_substitution(can_frame_topic_name_lc)
    topics = [
        can_frame_topic_name,
    ]
    return (
        ExecuteProcess(
            cmd=[
                "ros2 bag play",
                " --topics ",
                " ".join(topics),
                " -- ",
                rosbag_path_lc,
            ],
            condition=IfCondition(use_rosbag_lc),
            shell=True,
        ),
    )

def generate_launch_description():
    PACKAGE_NAME = "odometry_publisher"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)
    launch_args = (
        DeclareLaunchArgument(
            "sub_can_frame_topic_name",
            default_value="/from_can_bus",
            description="Can topic name",
        ),
        DeclareLaunchArgument(
            "pub_odometry_topic_name",
            default_value="/honda_sensing/odometry_publisher/odom",
            description="Odometry topic name.",
        ),
        DeclareLaunchArgument(
            "use_rosbag",
            default_value="false",
            description="If true, play rosbag",
        ),
        DeclareLaunchArgument(
            "rosbag_path",
            default_value="~/data/aiformula/20240212_odom_data/can_rotations",
            description="Path of rosbag to play",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="If true, run rviz2",
        ),
        DeclareLaunchArgument(
            "odom_frame_id",
            default_value="odom",
            description="Frame id odom",
        ),
        DeclareLaunchArgument(
            "robot_frame_id",
            default_value="base_footprint",
            description="Frame id of the robot",
        ),
    )

    ROS_PARAM_CONFIG = os.path.join(PACKAGE_DIR, "config", "odometry_publisher.yaml")
    nodes = (
        Node(
            package=PACKAGE_NAME,
            executable=PACKAGE_NAME,
            name=PACKAGE_NAME,
            namespace="/honda_sensing",
            output="screen",
            emulate_tty=True,
            parameters=[ROS_PARAM_CONFIG,
                        {
                            "odom_frame_id": LaunchConfiguration("odom_frame_id"),
                            "robot_frame_id": LaunchConfiguration("robot_frame_id"),
                        }],
            remappings=[
                ("sub_can_frame_topic_name", LaunchConfiguration("sub_can_frame_topic_name")),
                ("pub_odometry_topic_name", LaunchConfiguration("pub_odometry_topic_name")),
            ],
        ),

        # ros2 bag play
        OpaqueFunction(
            function=get_bag_play_node,
            args=[
                LaunchConfiguration("use_rosbag"),
                LaunchConfiguration("rosbag_path"),
                LaunchConfiguration("sub_can_frame_topic_name"),
            ],
        ),

        # rviz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_odometry_publisher",
            arguments=["-d", os.path.join(PACKAGE_DIR, "rviz", "odometry_publisher.rviz")],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    )
    return LaunchDescription([
        *launch_args,
        *nodes,
    ])
