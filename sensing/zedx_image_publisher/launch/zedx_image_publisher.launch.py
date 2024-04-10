from launch import LaunchDescription
from launch_ros.actions import Node
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    PACKAGE_NAME = "zedx_image_publisher"
    FRAME_IDS, TOPIC_NAMES = get_frame_ids_and_topic_names()

    launch_args = ()

    zedx_image_publisher = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_sensing",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "zedx_left_frame_id": FRAME_IDS["zedx"]["left"],
        }],
        remappings=[
            ("pub_left_image", TOPIC_NAMES["sensing"]["zedx"]["left_image"]["raw"]),
            ("pub_left_compressed_image", TOPIC_NAMES["sensing"]["zedx"]["left_image"]["raw_compressed"]),
        ],
    )

    return LaunchDescription([
        *launch_args,
        zedx_image_publisher,
    ])
