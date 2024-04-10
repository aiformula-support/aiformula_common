from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    PACKAGE_NAME = "roboteq_controller"
    launch_args = (
        DeclareLaunchArgument(
            "sub_speed_command",
            default_value="/aiformula_control/game_pad/cmd_vel",
            description="Topic name of speed command",
        ),
        DeclareLaunchArgument(
            "pub_can",
            default_value="/aiformula_control/roboteq_controller/reference_signal",
            description="Topic name of sending can data",
        ),
    )

    motor_controller = Node(
        package=PACKAGE_NAME,
        executable=PACKAGE_NAME,
        name=PACKAGE_NAME,
        namespace="/aiformula_control",
        output="screen",
        emulate_tty=True,
        remappings=[
            ("sub_speed_command", LaunchConfiguration("sub_speed_command")),
            ("pub_can", LaunchConfiguration("pub_can")),
        ],
    )

    return LaunchDescription([
        *launch_args,
        motor_controller,
    ])
