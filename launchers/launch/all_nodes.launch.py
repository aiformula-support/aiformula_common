import os.path as osp
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    VEHICLE_NAME = "ai_car1"
    with open(osp.join(get_package_share_directory("launchers"), "config/topic_list.yaml"), "r") as yml:
        topic_names = yaml.safe_load(yml)
    with open(osp.join(get_package_share_directory("launchers"), "config/frame_id_list.yaml"), "r") as yml:
        frame_ids = yaml.safe_load(yml)

    launch_args = ()

    nodes = (
        # --- tf_static publisher --- #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                osp.join(get_package_share_directory("vehicle"),
                         "launch/extrinsic_tfstatic_broadcaster.launch.py"),
            ),
            launch_arguments={
                "vehicle_name": VEHICLE_NAME,
                "use_sim_time": "false",
            }.items(),
        ),

        # --- Zed Camera --- #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                osp.join(get_package_share_directory("zedx_image_publisher"),
                         "launch/zedx_image_publisher.launch.py"),
            ),
            launch_arguments={
                "pub_left_image": topic_names["sensing"]["zed"]["left_image"]["raw"],
                "pub_left_compressed_image": topic_names["sensing"]["zed"]["left_image"]["raw_compressed"],
                "zed_left_frame_id": frame_ids["zedX"]["left"],
            }.items(),
        ),

        # --- IMU, GNSS --- #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                osp.join(get_package_share_directory("launchers"),
                         "launch/vectornav.launch.py"),
            ),
            launch_arguments={
                "pub_imu": topic_names["sensing"]["imu"],
            }.items(),
        ),

        # --- GamePad --- #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                osp.join(get_package_share_directory("launchers"),
                         "launch/joy.launch.py"),
            ),
            launch_arguments={
                "pub_game_pad_output": topic_names["control"]["game_pad"],
            }.items(),
        ),

        # --- Output velocity and angular velocity --- #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                osp.join(get_package_share_directory("launchers"),
                         "launch/teleop.launch.py"),
            ),
            launch_arguments={
                "sub_game_pad_output": topic_names["control"]["game_pad"],
                "pub_speed_command": topic_names["control"]["speed_command"]["game_pad"],
            }.items(),
        ),

        # --- Motor Controller --- #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                osp.join(get_package_share_directory("roboteq_controller"),
                         "launch/roboteq_controller.launch.py"),
            ),
            launch_arguments={
                "sub_speed_command": topic_names["control"]["speed_command"]["game_pad"],
                "pub_can": topic_names["control"]["output_can_data"],
            }.items(),
        ),

        # --- Can Bridge --- #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                osp.join(get_package_share_directory("launchers"),
                         "launch/socket_can_bridge.launch.py"),
            ),
            launch_arguments={
                "pub_can": topic_names["sensing"]["input_can_data"],
                "sub_can": topic_names["control"]["output_can_data"],
            }.items(),
        ),

        # --- Odometry --- #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                osp.join(get_package_share_directory("odometry_publisher"),
                         "launch/gyro_odometry_publisher.launch.py"),
            ),
            launch_arguments={
                "sub_imu": topic_names["sensing"]["imu"],
                "sub_can": topic_names["sensing"]["input_can_data"],
                "pub_odometry": topic_names["sensing"]["odometry"],
                "odom_frame_id": frame_ids["odom"],
                "robot_frame_id": frame_ids["base_footprint"],
                "use_rviz": "true",
            }.items(),
        ),
    )

    return LaunchDescription([
        *launch_args,
        *nodes,
    ])
