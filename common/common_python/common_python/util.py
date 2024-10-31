import os

import rclpy
from rclpy.node import Node


def abort(node: Node) -> None:
    """Abnormal termination

    Parameters:
    ----------
    `node`: The instance of th `Node` class

    Examples:
    ----------
    >>> abort(self)
    """
    node.destroy_node()
    rclpy.shutdown()
    exit(1)


def get_ros_distro() -> str:
    """Get ROS2 version

    Examples:
    ----------
    >>> if get_ros_distro() == "humble":
    """
    ros_distro = os.getenv("ROS_DISTRO")
    if ros_distro:
        return ros_distro
    else:
        print("Environmental Variable 'ROS Distro' is not set.")
        rclpy.shutdown()
        exit(1)
