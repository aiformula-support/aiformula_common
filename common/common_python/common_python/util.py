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
