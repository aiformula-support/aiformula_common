import os.path as osp
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    VEHICLE_NAME="ai_car1"
    TOPIC_NAMES = {
        "zed_left_image": "/aiformula_sensing/zedx_image_publisher/left_image_raw",
        "zed_left_compressed_image": "/aiformula_sensing/zedx_image_publisher/left_image_raw/compressed",
        "input_can_data": "/from_can_bus",
        # "input_can_data": "/aiformula_sensing/vehicle_info",
        "output_can_data": "/to_can_bus",
        # "output_can_data": "/aiformula_control/reference_signal",
        "imu": "/vectornav/imu",
        # "imu": "/aiformula_sensing/vectornav/imu",
        "odometry": "/aiformula_sensing/gyro_odometry/odom",
    }

    launch_args = ()

    nodes = (
        # --- tf_static publisher --- #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                osp.join(get_package_share_directory("vehicle"), "launch/extrinsic_tfstatic_broadcaster.launch.py"),
            ),
            launch_arguments={
                "vehicle_name": VEHICLE_NAME,
                "use_sim_time": "false",
                "use_joint_state_publisher": "true",
                "use_gui": "false",
            }.items(),
        ),

        # --- Zed Camera --- #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                osp.join(get_package_share_directory("zedx_image_publisher"), "launch/zedx_image_publisher.launch.py"),
            ),
            launch_arguments={
                "pub_left_image_topic_name": TOPIC_NAMES["zed_left_image"],
                "pub_left_compressed_image_topic_name": TOPIC_NAMES["zed_left_compressed_image"],
            }.items(),
        ),

        # --- Can Bridge --- #
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                osp.join(get_package_share_directory("ros2_socketcan"), "launch/socket_can_bridge.launch.xml"),
            ),
        ),

        # --- IMU, GNSS --- #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                osp.join(get_package_share_directory("vectornav"), "launch/vectornav.launch.py"),
            ),
        ),

        # --- Output velocity and angular velocity from GamePad --- #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                osp.join(get_package_share_directory("teleop_twist_joy"), "launch/teleop-launch.py"),
            ),
        ),

        # --- Odometry --- #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                osp.join(get_package_share_directory("odometry_publisher"), "launch/gyro_odometry.launch.py"),
            ),
            launch_arguments={
                "sub_imu_topic_name": TOPIC_NAMES["imu"],
                "sub_can_frame_topic_name": TOPIC_NAMES["input_can_data"],
                "pub_odometry_topic_name": TOPIC_NAMES["odometry"],
                "use_gui": "true",
            }.items(),
        ),
    )

    return LaunchDescription([
        *launch_args,
        *nodes,
    ])
