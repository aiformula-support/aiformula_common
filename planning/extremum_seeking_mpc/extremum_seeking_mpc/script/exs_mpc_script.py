import numpy as np
from scipy.interpolate import interp1d

sim_time = 0.01 # 10ms
sin_period = 0.08 # 0.08 s = 12.5 Hz
num_moving_average = int(sin_period / sim_time)

### --- Extremum Seeking Controller for Optimization ---
class ExtremumSeekingController:
    def __init__(self, seek_gain, seek_amp, curvature_max, curvature_min):
        self.seek_gain = seek_gain
        self.seek_amp  = seek_amp
        self.curvature_max   = curvature_max
        self.curvature_min   = curvature_min
        self.feedback_gain = 0.9

        self.risk_mem = 0
        #self.sin_mem = 0
        self.optimize_out_memory = 0

        self.moving_average_out = 0

        t = np.arange(0, sin_period, sim_time)
        self.sin_array = np.sin(2*np.pi*(t/sin_period))
        # sin dibided by 8: fixed value
        self.seek_points = np.hstack([np.flip(self.sin_array[0:3]), self.sin_array[-3: -1]]) # [1., 0.70710678, 0., -0.70710678, -1.]
        self.seek_points = (self.seek_points * self.seek_amp).tolist() # 5 points -> [0.7, 0.4949747468305832, 0.0, -0.4949747468305832, -0.7]
        
    def risk_moving_average(self, risk_in): # list [5x1]
        # Highpass filter
        valA = risk_in[0] - risk_in[2]
        valB = risk_in[1] - risk_in[2]
        valC = risk_in[3] - risk_in[2]
        valD = risk_in[4] - risk_in[2]

	# risk mutilplied by Reference Signal (Sin Wave) and Moving Average
        self.moving_average_out = (
            2 * self.sin_array[1] * valB + self.sin_array[2] * valA 
            + 2 * self.sin_array[5] * valC + self.sin_array[6] * valD
        ) / num_moving_average
        
        return self.moving_average_out # for backpropagation
        
    def extremum_seeking_optimize(self, moving_average, back_in):
        optimize_out = moving_average + back_in
        optimize_out = self.seek_gain * optimize_out
        optimize_out = optimize_out +  self.feedback_gain * self.optimize_out_memory
        optimize_out = max(optimize_out, self.curvature_min)
        optimize_out = min(optimize_out, self.curvature_max)
        self.optimize_out_memory = optimize_out # curvature out scaler

        return optimize_out # curvature(scalar)
        

### --- backpropagation ---
class backpropagation:
    def __init__(self):
        backpropagation_gain_u = [-1, -0.5, -0.05, 0, 0.05, 0.5, 1]
        backpropagation_gain_positive_y = [0, 0, 0, 0.5, 0.9, 1, 1]
        backpropagation_gain_negative_y = [1, 1, 0.9, 0.5, 0, 0, 0]
        self.bpgain_function_positive = interp1d(backpropagation_gain_u, backpropagation_gain_positive_y)
        self.bpgain_function_negative = interp1d(backpropagation_gain_u, backpropagation_gain_negative_y)

    def backward(self, forward_risk_in, backward_risk_in):
        propagation_gain = 0
        forward_risk_in = min(max(forward_risk_in, -1), 1) # range check
        if (backward_risk_in < 0):
            propagation_gain = self.bpgain_function_negative(forward_risk_in)
        else:
            propagation_gain = self.bpgain_function_positive(forward_risk_in)

        return backward_risk_in * propagation_gain

### --- EXS_MPC ---
class exs_mpc:
    def __init__(self, seek_gain, seek_amp, curvature_max, curvature_min):

        self.extremum_seeking_step1 = ExtremumSeekingController(seek_gain = seek_gain, seek_amp = seek_amp, curvature_max = curvature_max, curvature_min = curvature_min)
        self.extremum_seeking_step2 = ExtremumSeekingController(seek_gain = seek_gain, seek_amp = seek_amp, curvature_max = curvature_max, curvature_min = curvature_min)
        self.extremum_seeking_step3 = ExtremumSeekingController(seek_gain = seek_gain, seek_amp = seek_amp, curvature_max = curvature_max, curvature_min = curvature_min)

        self.backpropagation_21 = backpropagation()
        self.backpropagation_31 = backpropagation()
        self.backpropagation_32 = backpropagation()

    def extremum_seeking_mpc_3step(self, risk1, risk2, risk3):
        risk1_moving_average = self.extremum_seeking_step1.risk_moving_average(risk1) # in: list [5x1], out: scalar
        risk2_moving_average = self.extremum_seeking_step2.risk_moving_average(risk2)
        risk3_moving_average = self.extremum_seeking_step3.risk_moving_average(risk3)

        #print("risk_moving_average: ", risk1_moving_average, risk2_moving_average, risk3_moving_average)
        
        backpropagation_value_32 = self.backpropagation_32.backward(risk2_moving_average, risk3_moving_average)
        backpropagation_value_31 = self.backpropagation_31.backward(risk1_moving_average, risk3_moving_average)
        backpropagation_value_21 = self.backpropagation_21.backward(risk1_moving_average, risk2_moving_average)

        #print("bp: ", bp32, bp31, bp21)
        
        curvature1 = self.extremum_seeking_step1.extremum_seeking_optimize(risk1_moving_average, backpropagation_value_21 + backpropagation_value_31)
        curvature2 = self.extremum_seeking_step2.extremum_seeking_optimize(risk2_moving_average, backpropagation_value_32)
        curvature3 = self.extremum_seeking_step3.extremum_seeking_optimize(risk3_moving_average, 0)

        return [curvature1, curvature2, curvature3] # scalars, vectors (3 points)
