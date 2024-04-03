from launch import LaunchDescription
from launch_ros.actions import Node
from common_python.launch_util import get_frame_ids_and_topic_names


def generate_launch_description():
    PACKAGE_NAME = "motor_controller"
    _, TOPIC_NAMES = get_frame_ids_and_topic_names()

    motor_controller = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_control",
        output="screen",
        emulate_tty=True,
        remappings=[
            ("sub_speed_command", TOPIC_NAMES["control"]["speed_command"]["game_pad"]),
            ("pub_can", TOPIC_NAMES["control"]["output_can_data"]),
        ],
    )

    return LaunchDescription([
        motor_controller,
    ])
