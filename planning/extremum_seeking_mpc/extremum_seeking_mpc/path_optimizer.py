# --- PathOptimizer using Extremum Seeking Controller ---

from rclpy.node import Node

from .extremum_seeking_controller import ExtremumSeekingController
from .back_propagation import BackPropagation
from common_python.get_ros_parameter import get_ros_parameter
from .util import MpcParameters, LowPassFilterParameters


class PathOptimizer:
    def __init__(self, node: Node, control_time: float):
        self.init_parameters(node)
        params = MpcParameters(seek_gain=self.seek_gain, seek_amp=self.seek_amp,
                               curvature_max=self.curvature_limit, curvature_min=-(self.curvature_limit))
        lowpass_params = LowPassFilterParameters(A=self.lowpass_A, B=self.lowpass_B, C=self.lowpass_C)

        self.extremum_seeking_controller_step1 = ExtremumSeekingController(
            params, control_time, self.feedback_gain, self.sin_period, lowpass_params)
        self.extremum_seeking_controller_step2 = ExtremumSeekingController(
            params, control_time, self.feedback_gain, self.sin_period, lowpass_params)
        self.extremum_seeking_controller_step3 = ExtremumSeekingController(
            params, control_time, self.feedback_gain, self.sin_period, lowpass_params)

        self.backpropagation_21 = BackPropagation(
            self.backpropagation_gain_u, self.backpropagation_gain_positive_y, self.backpropagation_gain_negative_y, self.backpropagation_gain)
        self.backpropagation_31 = BackPropagation(
            self.backpropagation_gain_u, self.backpropagation_gain_positive_y, self.backpropagation_gain_negative_y, self.backpropagation_gain)
        self.backpropagation_32 = BackPropagation(
            self.backpropagation_gain_u, self.backpropagation_gain_positive_y, self.backpropagation_gain_negative_y, self.backpropagation_gain)

    def init_parameters(self, node: Node):
        self.seek_gain = get_ros_parameter(
            node, "mpc_parameters.seek_gain")
        self.seek_amp = get_ros_parameter(
            node, "mpc_parameters.seek_amp")
        self.curvature_limit = get_ros_parameter(
            node, "mpc_parameters.curvature_limit")
        self.feedback_gain = get_ros_parameter(
            node, "mpc_parameters.feedback_gain")
        self.sin_period = get_ros_parameter(
            node, "mpc_parameters.sin_period")
        self.backpropagation_gain_u = get_ros_parameter(
            node, "backpropagation.backpropagation_gain_u")
        self.backpropagation_gain_positive_y = get_ros_parameter(
            node, "backpropagation.backpropagation_gain_positive_y")
        self.backpropagation_gain_negative_y = get_ros_parameter(
            node, "backpropagation.backpropagation_gain_negative_y")
        self.backpropagation_gain = get_ros_parameter(
            node, "backpropagation.backpropagation_gain")
        self.lowpass_A = get_ros_parameter(
            node, "lowpassfilter.A")
        self.lowpass_B = get_ros_parameter(
            node, "lowpassfilter.B")
        self.lowpass_C = get_ros_parameter(
            node, "lowpassfilter.C")

    def extremum_seeking_mpc_3step(self, risk: list[float]) -> list[float]:

        risk1_moving_average = self.extremum_seeking_controller_step1.risk_moving_average(risk[0])
        risk2_moving_average = self.extremum_seeking_controller_step2.risk_moving_average(risk[1])
        risk3_moving_average = self.extremum_seeking_controller_step3.risk_moving_average(risk[2])

        backpropagation_value_32 = self.backpropagation_32.backward(
            risk2_moving_average, risk3_moving_average, self.backpropagation_gain)
        backpropagation_value_31 = self.backpropagation_31.backward(
            risk1_moving_average, risk3_moving_average, self.backpropagation_gain)
        backpropagation_value_21 = self.backpropagation_21.backward(
            risk1_moving_average, risk2_moving_average, self.backpropagation_gain)

        curvature1 = self.extremum_seeking_controller_step1.extremum_seeking_optimize(
            risk1_moving_average, backpropagation_value_31 + backpropagation_value_21)
        curvature2 = self.extremum_seeking_controller_step2.extremum_seeking_optimize(
            risk2_moving_average, backpropagation_value_32)
        curvature3 = self.extremum_seeking_controller_step3.extremum_seeking_optimize(
            risk3_moving_average, 0)

        return [curvature1, curvature2, curvature3]
