import os.path as osp
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    PACKAGE_DIR = get_package_share_directory("ros2_socketcan")
    launch_args = (
        DeclareLaunchArgument(
            "interface", default_value="can0"),
        DeclareLaunchArgument(
            "receiver_interval_sec", default_value="0.01"),
        DeclareLaunchArgument(
            "sender_timeout_sec", default_value="0.01"),
        DeclareLaunchArgument(
            "enable_can_fd", default_value="false"),
        DeclareLaunchArgument(
            "interface", default_value="can0"),
        DeclareLaunchArgument(
            "pub_can", default_value="/aiformula_sensing/vehicle_info"),
        DeclareLaunchArgument(
            "sub_can", default_value="/aiformula_control/roboteq_controller/reference_signal"),
    )

    nodes = (
        PushRosNamespace("/aiformula_sensing"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                osp.join(PACKAGE_DIR, "launch/socket_can_receiver.launch.py"),
            ),
            launch_arguments={
                "interface": LaunchConfiguration("interface"),
                "interval_sec": LaunchConfiguration("receiver_interval_sec"),
                "enable_can_fd": LaunchConfiguration("enable_can_fd"),
                "from_can_bus_topic": LaunchConfiguration("pub_can"),
            }.items(),
        ),

        PushRosNamespace("/aiformula_control"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                osp.join(PACKAGE_DIR, "launch/socket_can_sender.launch.py"),
            ),
            launch_arguments={
                "interface": LaunchConfiguration("interface"),
                "interval_sec": LaunchConfiguration("sender_timeout_sec"),
                "enable_can_fd": LaunchConfiguration("enable_can_fd"),
                "to_can_bus_topic": LaunchConfiguration("sub_can"),
            }.items(),
        ),
    )
    return LaunchDescription([
        *launch_args,
        *nodes,
    ])
