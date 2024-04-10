import os.path as osp
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    PACKAGE_NAME = "odometry_publisher"
    PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)
    launch_args = (
        DeclareLaunchArgument(
            "sub_imu",
            default_value="/vectornav/imu",
            # default_value="/aiformula_sensing/vectornav/imu",
            description="Imu topic name",
        ),
        DeclareLaunchArgument(
            "sub_can",
            default_value="/from_can_bus",
            # default_value="/aiformula_sensing/can/vehicle_info",
            description="Can topic name",
        ),
        DeclareLaunchArgument(
            "pub_wheel_odometry",
            default_value="/aiformula_sensing/wheel_odometry_publisher/odom",
            description="Wheel odometry topic name.",
        ),
        DeclareLaunchArgument(
            "pub_gyro_odometry",
            default_value="/aiformula_sensing/gyro_odometry_publisher/odom",
            description="Gyro odometry topic name.",
        ),
        DeclareLaunchArgument(
            "odom_frame_id",
            default_value="odom",
            description="Frame id of odom",
        ),
        DeclareLaunchArgument(
            "vehicle_frame_id_wheel_odometry",
            default_value="base_footprint_wheel_odometry",
            description="Frame id of the vehicle following wheel odometry",
        ),
        DeclareLaunchArgument(
            "vehicle_frame_id_gyro_odometry",
            default_value="base_footprint_gyro_odometry",
            description="Frame id of the vehicle following gyro odometry",
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
    )

    ROS_PARAM_CONFIG_WHEEL_ODOMETRY = (
        osp.join(PACKAGE_DIR, "config", "wheel.yaml"),
    )
    ROS_PARAM_CONFIG_GYRO_ODOMETRY = (
        osp.join(PACKAGE_DIR, "config", "wheel.yaml"),
        osp.join(PACKAGE_DIR, "config", "gyro_odometry_publisher.yaml"),
    )
    nodes = (
        Node(
            package=PACKAGE_NAME,
            executable="wheel_odometry_publisher",
            name="wheel_odometry_publisher",
            namespace="/aiformula_sensing",
            output="screen",
            emulate_tty=True,
            parameters=[*ROS_PARAM_CONFIG_WHEEL_ODOMETRY,
                        {
                            "odom_frame_id": LaunchConfiguration("odom_frame_id"),
                            "vehicle_frame_id": LaunchConfiguration("vehicle_frame_id_wheel_odometry"),
                        }],
            remappings=[
                ("sub_can", LaunchConfiguration("sub_can")),
                ("pub_odometry", LaunchConfiguration("pub_wheel_odometry")),
            ],
        ),

        Node(
            package=PACKAGE_NAME,
            executable="gyro_odometry_publisher",
            name="gyro_odometry_publisher",
            namespace="/aiformula_sensing",
            output="screen",
            emulate_tty=True,
            parameters=[*ROS_PARAM_CONFIG_GYRO_ODOMETRY,
                        {
                            "odom_frame_id": LaunchConfiguration("odom_frame_id"),
                            "vehicle_frame_id": LaunchConfiguration("vehicle_frame_id_gyro_odometry"),
                        }],
            remappings=[
                ("sub_imu", LaunchConfiguration("sub_imu")),
                ("sub_can", LaunchConfiguration("sub_can")),
                ("pub_odometry", LaunchConfiguration("pub_gyro_odometry")),
            ],
        ),

        # ros2 bag play
        ExecuteProcess(
            cmd=[
                "ros2 bag play",
                " --topics ",
                LaunchConfiguration("sub_imu"),
                LaunchConfiguration("sub_can"),
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
            name="rviz2_wheel_and_gyro_odometry_publishers",
            arguments=[
                "-d", osp.join(PACKAGE_DIR, "rviz", "wheel_and_gyro_odometry_publishers.rviz")],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    )
    return LaunchDescription([
        *launch_args,
        *nodes,
    ])
