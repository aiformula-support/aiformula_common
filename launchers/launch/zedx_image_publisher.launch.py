import os.path as osp
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
from launch_ros.actions import Node
from common_python.launch_util import get_frame_ids_and_topic_names, check_zedx_available_fps


def launch_setup(context):
    TOPIC_NS = "/aiformula_sensing"
    _, TOPIC_NAMES = get_frame_ids_and_topic_names()
    # Launch configuration variables
    config_common_path = LaunchConfiguration("config_path")
    config_camera_path = LaunchConfiguration("config_camera_path")
    grab_resolution_val = LaunchConfiguration("grab_resolution").perform(context)
    grab_frame_rate_val = LaunchConfiguration("grab_frame_rate").perform(context)
    is_valid_fps = check_zedx_available_fps(grab_resolution_val, grab_frame_rate_val)
    return (
        # ZED Wrapper node
        Node(
            package="zed_wrapper",
            namespace=TOPIC_NS,
            executable="zed_wrapper",
            name="zedx_image_publisher",
            output="screen",
            condition=IfCondition(str(is_valid_fps)),
            parameters=[
                # YAML files
                config_common_path,  # Common parameters
                config_camera_path,  # Camera related parameters
                # Overriding
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "simulation.sim_enabled": LaunchConfiguration("sim_mode"),
                    "simulation.sim_address": LaunchConfiguration("sim_address"),
                    "simulation.sim_port": LaunchConfiguration("sim_port"),
                    "general.camera_name": LaunchConfiguration("camera_name"),
                    "general.camera_model": LaunchConfiguration("camera_model"),
                    "general.grab_resolution": LaunchConfiguration("grab_resolution"),
                    "general.grab_frame_rate": int(grab_frame_rate_val),
                    "general.svo_file": LaunchConfiguration("svo_path"),
                    "general.optional_opencv_calibration_file": LaunchConfiguration("calibration_path"),
                    "general.serial_number": LaunchConfiguration("serial_number"),
                    "sensors.publish_imu_tf": LaunchConfiguration("publish_imu_tf"),

                },
            ],
            remappings=[
                ("/aiformula_sensing/zedx_image_publisher/left_raw/image_raw_color",
                 TOPIC_NAMES["sensing"]["zedx"]["left_image"]["raw"]),
                ("/aiformula_sensing/zedx_image_publisher/right_raw/image_raw_color",
                 TOPIC_NAMES["sensing"]["zedx"]["right_image"]["raw"]),
            ],
        ),
    )


def generate_launch_description():
    launch_args = (
        SetEnvironmentVariable(name="RCUTILS_COLORIZED_OUTPUT", value="1"),
        DeclareLaunchArgument(
            "camera_name",
            default_value="zed",
            description="The name of the camera. It can be different from the camera model and it will be used as node `namespace`."),
        DeclareLaunchArgument(
            "camera_model",
            default_value="zedx",
            description="[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.",
            choices=["zed", "zedm", "zed2", "zed2i", "zedx", "zedxm"]),
        DeclareLaunchArgument(
            "grab_resolution",
            default_value=TextSubstitution(text="HD1080"),
            description="The native camera grab resolution. HD1200, HD1080, SVGA, AUTO"),
        DeclareLaunchArgument(
            "grab_frame_rate",
            default_value=TextSubstitution(text="60"),
            description="grabbing rate (HD1200/HD1080: 60, 30, 15 - SVGA: 120, 60, 30, 15)"),
        DeclareLaunchArgument(
            "serial_number",
            default_value="0",
            description="The serial number of the camera to be opened. It is mandatory to use this parameter in multi-camera rigs to distinguish between different cameras."),
        DeclareLaunchArgument(
            "publish_imu_tf",
            default_value="true",
            description="Enable publication of the IMU TF. Note: Ignored if `publish_tf` is False.",
            choices=["true", "false"]),
        DeclareLaunchArgument(
            "svo_path",
            default_value=TextSubstitution(text="live"),
            description="Path to an input SVO file. Note: overrides the parameter `general.svo_file` in `common.yaml`."),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Enable simulation time mode.",
            choices=["true", "false"]),
        DeclareLaunchArgument(
            "sim_mode",
            default_value="false",
            description="Enable simulation mode. Set `sim_address` and `sim_port` to configure the simulator input.",
            choices=["true", "false"]),
        DeclareLaunchArgument(
            "sim_address",
            default_value="127.0.0.1",
            description="The connection address of the simulation server. See the documentation of the supported simulation plugins for more information."),
        DeclareLaunchArgument(
            "sim_port",
            default_value="30000",
            description="The connection port of the simulation server. See the documentation of the supported simulation plugins for more information."),
        DeclareLaunchArgument(
            "config_path",
            default_value=osp.join(get_package_share_directory("vehicle"), "config", "zedx", "common.yaml"),
            description="Path to the YAML configuration file for the camera."),
        DeclareLaunchArgument(
            "config_camera_path",
            default_value=osp.join(get_package_share_directory("vehicle"), "config", "zedx", "zedx.yaml"),
            description="Path to the YAML configuration file for the camera."),
        DeclareLaunchArgument(
            "calibration_path",
            default_value=[
                osp.join(get_package_share_directory("vehicle"), "config", "zedx", "calibration_file", ""),
                LaunchConfiguration("grab_resolution"),  ".yaml"
            ],
            description="Path to the YAML configuration file for the camera."),
    )
    zedx_image_publisher = OpaqueFunction(function=launch_setup)

    return LaunchDescription([
        *launch_args,
        zedx_image_publisher,
    ])
