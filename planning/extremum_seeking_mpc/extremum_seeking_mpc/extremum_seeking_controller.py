# --- Extremum Seeking Controller for Optimization ---

import numpy as np

from .util import MpcParameters, LowPassFilterParameters


class ExtremumSeekingController:
    def __init__(self, params: MpcParameters, control_time: float, feedback_gain: float, sin_period: float, lowpass_params: LowPassFilterParameters):
        # Control Time
        sim_time = control_time

        # MPC Parameters
        self.seek_gain = params.seek_gain
        self.seek_amp = params.seek_amp
        self.curvature_max = params.curvature_max
        self.curvature_min = params.curvature_min
        self.feedback_gain = feedback_gain

        # Setting Sin Wave
        sin_period = sin_period
        sin_time = np.arange(0, sin_period, sim_time)
        self.sin_array = np.sin(2*np.pi*(sin_time/sin_period))

        # Setting Seek Points
        # sin divided by 5: fixed value
        # [1., 0.70710678, 0., -0.70710678, -1.]
        self.seek_points = np.hstack(
            [np.flip(self.sin_array[0:3]), self.sin_array[-3: -1]])
        self.seek_points = (self.seek_points * self.seek_amp).tolist()

        # Moving Average
        self.num_moving_average = int(sin_period / sim_time)

        # LowPass Filter Parameters
        self.lowpass_A = lowpass_params.A
        self.lowpass_B = lowpass_params.B
        self.lowpass_C = lowpass_params.C
        self.state = 0.
        self.state_next = 0.

        # Output Memory
        self.optimize_out_memory = 0.

    def risk_moving_average(self, risk_in: list[float]) -> float:  # list [5x1]
        # Highpass filter
        valueA = risk_in[0] - risk_in[2]
        valueB = risk_in[1] - risk_in[2]
        valueC = risk_in[3] - risk_in[2]
        valueD = risk_in[4] - risk_in[2]

        # Risk mutilplied by Reference Signal (Sin Wave) and Moving Average
        moving_average_out = (
            2 * self.sin_array[1] * valueB + self.sin_array[2] * valueA
            + 2 * self.sin_array[5] * valueC + self.sin_array[6] * valueD
        ) / self.num_moving_average

        # LowPass Filter
        self.state_next = self.lowpass_A * self.state + self.lowpass_B * moving_average_out
        moving_average_out = self.lowpass_C * self.state
        self.state = self.state_next

        return moving_average_out  # for backpropagation

    def extremum_seeking_optimize(self, moving_average: float, back_in: float) -> float:
        optimize_out = moving_average + back_in
        optimize_out = self.seek_gain * optimize_out
        optimize_out = optimize_out + self.feedback_gain * self.optimize_out_memory
        optimize_out = np.clip(optimize_out, self.curvature_min, self.curvature_max)
        self.optimize_out_memory = optimize_out

        return optimize_out
