from typing import Any

import rclpy

from common_python.util import abort


def get_ros_parameter(node: rclpy.node.Node, param_name: str) -> Any:
    """Returns the value of the parameter `param_name` passed to the node.

    Parameters:
    ----------
    `node`: the instance of th `Node` class
    `param_name`: the name of the parameter

    Returns:
    ----------
    the value of the parameter

    Examples:
    ----------
    >>> tread = get_ros_parameter(self, "wheel.tread"),
    """

    try:
        node.declare_parameter(param_name, rclpy.Parameter.Type.NOT_SET)
        return node.get_parameter(param_name).value
    except TypeError:
        node.get_logger().error(f"The value for '{param_name}' has not been provided !")
        abort(node)
    except rclpy.exceptions.ParameterAlreadyDeclaredException:
        node.get_logger().error(f"The parameter '{param_name}' has already been declared !")
        abort(node)
    except rclpy.exceptions.InvalidParameterException:
        node.get_logger().error(f"The name of parameter '{param_name}' is invalid !")
        abort(node)
    except rclpy.exceptions.ParameterNotDeclaredException:
        node.get_logger().error(f"The parameter '{param_name}' has not been declared !")
        abort(node)
