import os
from typing import Tuple
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def get_gzserver(
        context: LaunchContext,
        world_name_lc: LaunchConfiguration,
        vehicle_name_lc: LaunchConfiguration,
    ) -> Tuple[Node]: 
    world_name = context.perform_substitution(world_name_lc)
    vehicle_name = context.perform_substitution(vehicle_name_lc)
    world_path = os.path.join(get_package_share_directory("simulator"),
                              "worlds", world_name, vehicle_name + ".model")
    return  (
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("gazebo_ros") + "/launch/gzserver.launch.py"
            ),
            launch_arguments={
                "world": world_path,
                "world_name": world_name,
            }.items()
        ),
    )

def generate_launch_description():

    launch_args = (
        DeclareLaunchArgument(
            "world_name",
            default_value="shihou_course",
            description="World Name",
        ),
        DeclareLaunchArgument(
            "vehicle_name",
            default_value="ai_car1",
            description="Vehicle Name",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulation (Gazebo) clock",
        ),
    )

    nodes = (
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("vehicle") + "/launch/extrinsic_tfstatic_broadcaster.launch.py"
            ),
            launch_arguments={
                "vehicle_name": "ai_car1",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "use_joint_state_publisher": "true",
                "use_gui": "false",
            }.items()
        ),
        # ExecuteProcess(
            # cmd=["export", "GAZEBO_MODEL_PATH=$(ros2 pkg prefix vehicle)/share/vehicle/xacro"],
            # shell=True),
        OpaqueFunction(
            function=get_gzserver,
            args=[
                LaunchConfiguration("world_name"),
                LaunchConfiguration("vehicle_name"),
            ],
        ),
       IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("gazebo_ros") + "/launch/gzclient.launch.py"
            ),
        ),
        ExecuteProcess(
            cmd=['ros2', 'param', 'set', '/gazebo',
                 'use_sim_time', LaunchConfiguration("use_sim_time")],
            output='screen'),
    )

    return LaunchDescription([
        *launch_args,
        *nodes,
    ])
