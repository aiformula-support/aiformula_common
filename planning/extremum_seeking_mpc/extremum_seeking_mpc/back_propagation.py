import numpy as np
from scipy.interpolate import interp1d

from rclpy.node import Node
from common_python.get_ros_parameter import get_ros_parameter


class BackPropagation:
    def __init__(self, node: Node):
        self.init_parameters(node)
        self.bpgain_function_positive = interp1d(
            self.gain_function_u, self.gain_function_positive_y, self.propagation_gain)
        self.bpgain_function_negative = interp1d(
            self.gain_function_u, self.gain_function_negative_y, self.propagation_gain)

    def init_parameters(self, node: Node):
        self.gain_function_u = get_ros_parameter(
            node, "backpropagation.backpropagation_gain_u")
        self.gain_function_positive_y = get_ros_parameter(
            node, "backpropagation.backpropagation_gain_positive_y")
        self.gain_function_negative_y = get_ros_parameter(
            node, "backpropagation.backpropagation_gain_negative_y")
        self.propagation_gain = get_ros_parameter(
            node, "backpropagation.propagation_gain")

    def apply_backpropagation(self, forward_risk_in, backward_risk_in):
        forward_risk_in = np.clip(forward_risk_in, -1., 1.)
        if (backward_risk_in < 0.0):
            propagation_gain = self.bpgain_function_negative(forward_risk_in)
        else:
            propagation_gain = self.bpgain_function_positive(forward_risk_in)

        return backward_risk_in * propagation_gain
