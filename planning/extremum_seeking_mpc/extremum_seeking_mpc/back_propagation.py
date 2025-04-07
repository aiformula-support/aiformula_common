# --- BackPropagation ---
from scipy.interpolate import interp1d


class BackPropagation:
    def __init__(self, backpropagation_gain_u, backpropagation_gain_positive_y, backpropagation_gain_negative_y, backpropagation_gain):
        self.bpgain_function_positive = interp1d(
            backpropagation_gain_u, backpropagation_gain_positive_y, backpropagation_gain)
        self.bpgain_function_negative = interp1d(
            backpropagation_gain_u, backpropagation_gain_negative_y, backpropagation_gain)

    def backward(self, forward_risk_in, backward_risk_in, backpropagation_gain):
        propagation_gain = backpropagation_gain
        forward_risk_in = min(max(forward_risk_in, -1), 1)
        if (backward_risk_in < 0):
            propagation_gain = self.bpgain_function_negative(forward_risk_in)
        else:
            propagation_gain = self.bpgain_function_positive(forward_risk_in)

        return backward_risk_in * propagation_gain
