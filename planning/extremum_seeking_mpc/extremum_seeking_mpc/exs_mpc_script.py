import numpy as np
from rclpy.node import Node
from dataclasses import dataclass
from scipy.interpolate import interp1d

from common_python.get_ros_parameter import get_ros_parameter
from .util import MpcParameters, LowPassFilterParameters


# --- Extremum Seeking Controller for Optimization ---


class ExtremumSeekingController:
    def __init__(self, params: MpcParameters, control_time, feedback_gain, sin_period, lowpass_params: LowPassFilterParameters):
        self.seek_gain = params.seek_gain
        self.seek_amp = params.seek_amp
        self.curvature_max = params.curvature_max
        self.curvature_min = params.curvature_min
        self.feedback_gain = feedback_gain

        self.risk_mem = 0
        self.optimize_out_memory = 0
        self.moving_average_out = 0

        sim_time = control_time
        sin_period = sin_period
        self.num_moving_average = int(sin_period / sim_time)
        t = np.arange(0, sin_period, sim_time)
        self.sin_array = np.sin(2*np.pi*(t/sin_period))
        # sin divided by 8: fixed value
        # [1., 0.70710678, 0., -0.70710678, -1.]
        self.seek_points = np.hstack(
            [np.flip(self.sin_array[0:3]), self.sin_array[-3: -1]])
        # 5 points -> [0.7, 0.4949747468305832, 0.0, -0.4949747468305832, -0.7]
        self.seek_points = (self.seek_points * self.seek_amp).tolist()

        # LowPass Filter Parameters
        self.lowpass_A = lowpass_params.A
        self.lowpass_B = lowpass_params.B
        self.lowpass_C = lowpass_params.C
        self.x = 0
        self.x_next = 0
        self.y = 0

    def risk_moving_average(self, risk_in):  # list [5x1]
        # Highpass filter
        valueA = risk_in[0] - risk_in[2]
        valueB = risk_in[1] - risk_in[2]
        valueC = risk_in[3] - risk_in[2]
        valueD = risk_in[4] - risk_in[2]

        # risk mutilplied by Reference Signal (Sin Wave) and Moving Average
        self.moving_average_out = (
            2 * self.sin_array[1] * valueB + self.sin_array[2] * valueA
            + 2 * self.sin_array[5] * valueC + self.sin_array[6] * valueD
        ) / self.num_moving_average

        # LowPass Filter
        self.x_next = self.lowpass_A * self.x + self.lowpass_B * self.moving_average_out
        self.moving_average_out = self.lowpass_C * self.x
        self.x = self.x_next

        return self.moving_average_out  # for backpropagation

    def extremum_seeking_optimize(self, moving_average, back_in):
        optimize_out = moving_average + back_in
        optimize_out = self.seek_gain * optimize_out
        optimize_out = optimize_out + self.feedback_gain * self.optimize_out_memory
        optimize_out = np.clip(
            optimize_out, self.curvature_min, self.curvature_max)
        self.optimize_out_memory = optimize_out  # curvature out scaler

        return optimize_out  # curvature(scalar)


# --- backpropagation ---
class backpropagation:
    def __init__(self, backpropagation_gain_u, backpropagation_gain_positive_y, backpropagation_gain_negative_y, backpropagation_gain):
        self.bpgain_function_positive = interp1d(
            backpropagation_gain_u, backpropagation_gain_positive_y, backpropagation_gain)
        self.bpgain_function_negative = interp1d(
            backpropagation_gain_u, backpropagation_gain_negative_y, backpropagation_gain)

    def backward(self, forward_risk_in, backward_risk_in, backpropagation_gain):
        propagation_gain = backpropagation_gain
        forward_risk_in = min(max(forward_risk_in, -1), 1)  # range check
        if (backward_risk_in < 0):
            propagation_gain = self.bpgain_function_negative(forward_risk_in)
        else:
            propagation_gain = self.bpgain_function_positive(forward_risk_in)

        return backward_risk_in * propagation_gain

# --- Extremum Seeking Controller ---


class OptimizePath:
    def __init__(self, node: Node, control_time):
        self.init_parameters(node)
        params = MpcParameters(seek_gain=self.seek_gain, seek_amp=self.seek_amp,
                               curvature_max=self.curvature_limit, curvature_min=-(self.curvature_limit))
        lowpass_params = LowPassFilterParameters(
            A=self.lowpass_A, B=self.lowpass_B, C=self.lowpass_C)

        self.extremum_seeking_controller_step1 = ExtremumSeekingController(
            params, control_time, self.feedback_gain, self.sin_period, lowpass_params)
        self.extremum_seeking_controller_step2 = ExtremumSeekingController(
            params, control_time, self.feedback_gain, self.sin_period, lowpass_params)
        self.extremum_seeking_controller_step3 = ExtremumSeekingController(
            params, control_time, self.feedback_gain, self.sin_period, lowpass_params)

        self.backpropagation_21 = backpropagation(
            self.backpropagation_gain_u, self.backpropagation_gain_positive_y, self.backpropagation_gain_negative_y, self.backpropagation_gain)
        self.backpropagation_31 = backpropagation(
            self.backpropagation_gain_u, self.backpropagation_gain_positive_y, self.backpropagation_gain_negative_y, self.backpropagation_gain)
        self.backpropagation_32 = backpropagation(
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

    def extremum_seeking_mpc_3step(self, risk1, risk2, risk3):
        risk1_moving_average = self.extremum_seeking_controller_step1.risk_moving_average(
            risk1)  # in: list [5x1], out: scalar
        risk2_moving_average = self.extremum_seeking_controller_step2.risk_moving_average(
            risk2)
        risk3_moving_average = self.extremum_seeking_controller_step3.risk_moving_average(
            risk3)

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

        # scalars, vectors (3 points)
        return [curvature1, curvature2, curvature3]
