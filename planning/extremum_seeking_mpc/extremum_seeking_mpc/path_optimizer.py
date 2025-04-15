import numpy as np

from rclpy.node import Node

from .back_propagation import BackPropagation
from common_python.get_ros_parameter import get_ros_parameter
from .extremum_seeking_controller import ExtremumSeekingController
from .util import ControllerParameters, LowPassFilterParameters


class PathOptimizer:
    def __init__(self, node: Node, control_period: float):
        seek_gain = get_ros_parameter(node, "controller_parameters.seek_gain")
        seek_amp = get_ros_parameter(node, "controller_parameters.seek_amp")
        curvature_limit = get_ros_parameter(node, "controller_parameters.curvature_limit")
        feedback_gain = get_ros_parameter(node, "controller_parameters.feedback_gain")
        sin_period = get_ros_parameter(node, "controller_parameters.sin_period")

        mpc_params = ControllerParameters(
            seek_gain=seek_gain,
            seek_amp=seek_amp,
            curvature_max=curvature_limit,
            curvature_min=-curvature_limit,
            feedback_gain=feedback_gain,
            sin_period=sin_period
        )

        lowpass_A = get_ros_parameter(node, "lowpassfilter.A")
        lowpass_B = get_ros_parameter(node, "lowpassfilter.B")
        lowpass_C = get_ros_parameter(node, "lowpassfilter.C")

        lowpass_params = LowPassFilterParameters(
            A=lowpass_A, B=lowpass_B, C=lowpass_C
        )

        self.extremum_seeking_controller_step1 = ExtremumSeekingController(mpc_params, lowpass_params, control_period)
        self.extremum_seeking_controller_step2 = ExtremumSeekingController(mpc_params, lowpass_params, control_period)
        self.extremum_seeking_controller_step3 = ExtremumSeekingController(mpc_params, lowpass_params, control_period)

        self.backpropagation = BackPropagation(node)

    def extremum_seeking_control(self, risk: np.ndarray) -> np.ndarray:
        controllers = [
            self.extremum_seeking_controller_step1,
            self.extremum_seeking_controller_step2,
            self.extremum_seeking_controller_step3
        ]

        # Calculate moving averages
        risk_moving_averages = [ctrl.risk_moving_average(r) for ctrl, r in zip(controllers, risk)]

        # Calculate backpropagation values
        backprop_values = {
            "32": self.backpropagation.backward(risk_moving_averages[1], risk_moving_averages[2]),
            "31": self.backpropagation.backward(risk_moving_averages[0], risk_moving_averages[2]),
            "21": self.backpropagation.backward(risk_moving_averages[0], risk_moving_averages[1])
        }

        # Optimize control inputs
        curvatures = [
            controllers[0].optimize_input(
                risk_moving_averages[0], backprop_values["31"] + backprop_values["21"]),
            controllers[1].optimize_input(
                risk_moving_averages[1], backprop_values["32"]),
            controllers[2].optimize_input(
                risk_moving_averages[2], 0)
        ]

        return np.array(curvatures)
