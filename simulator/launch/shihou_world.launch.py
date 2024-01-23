import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    launch_args = (
        DeclareLaunchArgument(
            "world_name",
            default_value="ai_car",
            description="World Name",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulation (Gazebo) clock",
        ),
    )

    # use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time = "true"
    world_name = "ai_car"
    world_path = os.path.join(get_package_share_directory("simulator"),
                              "worlds", "shihou_world", world_name + ".model")

    nodes = (
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("vehicle") + "/launch/extrinsic_tfstatic_broadcaster.launch.py"
            ),
            launch_arguments={
                "vehicle_name": "ai_car1",
                "use_sim_time": "true",
                "use_joint_state_publisher": "true",
                "use_gui": "false",
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("gazebo_ros") + "/launch/gzserver.launch.py"
            ),
            launch_arguments={
                "world": world_path,
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("gazebo_ros") + "/launch/gzclient.launch.py"
            ),
        ),
        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo', 'use_sim_time', use_sim_time],
            output='screen'),
    )
    return LaunchDescription([
        *nodes,
    ])
