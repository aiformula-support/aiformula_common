import numpy as np
from rclpy.node import Node
from scipy.interpolate import interp1d
from scipy.stats import multivariate_normal

from common_python.get_ros_parameter import get_ros_parameter
from .util import Side


class RoadRiskCalculator:
    def __init__(self, node: Node):
        self.init_parameters(node)
        Weight_u = self.road_weight_u
        Weight_y1 = self.road_weight_y1
        Weight_y2 = self.road_weight_y2
        Weight_y3 = self.road_weight_y3
        Weight_y4 = self.road_weight_y4
        Weight_y5 = self.road_weight_y5
        self.Weight1_function = interp1d(Weight_u, Weight_y1)
        self.Weight2_function = interp1d(Weight_u, Weight_y2)
        self.Weight3_function = interp1d(Weight_u, Weight_y3)
        self.Weight4_function = interp1d(Weight_u, Weight_y4)
        self.Weight5_function = interp1d(Weight_u, Weight_y5)
        self.identification_gain_matrix = np.zeros((3, 3))
        np.fill_diagonal(self.identification_gain_matrix, self.identification_gain)
        self.forget_vector = np.array(self.forget_vector)
        self.theta_base = np.zeros(3)

    def init_parameters(self, node: Node):
        self.road_risk_left_gradient = get_ros_parameter(
            node, "road_risk_potential.road_risk_left_gradient")
        self.road_risk_right_gradient = get_ros_parameter(
            node, "road_risk_potential.road_risk_right_gradient")
        self.road_risk_margin = get_ros_parameter(
            node, "road_risk_potential.road_risk_margin")
        self.road_risk_gain = get_ros_parameter(
            node, "road_risk_potential.road_risk_gain")
        self.road_weight_u = get_ros_parameter(
            node, "road_weight.weight_u")
        self.road_weight_y1 = get_ros_parameter(
            node, "road_weight.weight_y1")
        self.road_weight_y2 = get_ros_parameter(
            node, "road_weight.weight_y2")
        self.road_weight_y3 = get_ros_parameter(
            node, "road_weight.weight_y3")
        self.road_weight_y4 = get_ros_parameter(
            node, "road_weight.weight_y4")
        self.road_weight_y5 = get_ros_parameter(
            node, "road_weight.weight_y5")
        self.max_point_length = get_ros_parameter(
            node, "road_identification.max_point_length")
        self.identification_gain = get_ros_parameter(
            node, "road_identification.identification_gain")
        self.forget_vector = get_ros_parameter(
            node, "road_identification.forget_vector")
        self.road_parameter_limit = get_ros_parameter(
            node, "road_identification.road_parameter_limit")
        self.benefit_scale = get_ros_parameter(
            node, "road_benefit_function.scale")
        self.benefit_covariance = get_ros_parameter(
            node, "road_benefit_function.covariance")

    def line_identification(self, xys: np.ndarray) -> tuple[float, float, np.ndarray]:
        xs = np.ravel(xys[:, :1].T)
        normalize_offset = min(xs)
        normalize_denominator = max(xs) - normalize_offset
        index = np.random.permutation(len(xs))
        xs = xs[index]
        xs = (xs - normalize_offset) / normalize_denominator

        ys = np.ravel(xys[:, 1:2].T)
        ys = ys[index]

        xxs = xs * xs
        ones = np.array([1] * len(xs))
        zetas = np.vstack([xxs, xs, ones]).T
        w1s = self.Weight1_function(xs)
        w2s = self.Weight2_function(xs)
        w3s = self.Weight3_function(xs)
        w4s = self.Weight4_function(xs)
        w5s = self.Weight5_function(xs)

        thetas = np.zeros((5, 3))
        dthetas = np.zeros((5, 3))

        for zeta, w1, w2, w3, w4, w5, y in zip(zetas, w1s, w2s, w3s, w4s, w5s, ys):
            # preparation any parameters
            # p: 3x3, zeta: 3x1 = 3x1
            adaptive_gain_numerator = np.dot(self.identification_gain_matrix, zeta.reshape(3, 1))
            # zeta: 1x3, P: 3x3 = 1x3
            zp = np.dot(zeta.reshape(1, 3), self.identification_gain_matrix)
            # zp: 1x3, zeta: 3x1 = zpz: 1x1
            adaptive_gain_denominator = 1 + np.dot(zp, zeta.reshape(3, 1))
            adaptive_gain = adaptive_gain_numerator / adaptive_gain_denominator

            # Estimate seek_y_position & Error
            y_hat1 = np.dot(thetas[0], zeta.T) * w1
            y_hat2 = np.dot(thetas[1], zeta.T) * w2
            y_hat3 = np.dot(thetas[2], zeta.T) * w3
            y_hat4 = np.dot(thetas[3], zeta.T) * w4
            y_hat5 = np.dot(thetas[4], zeta.T) * w5
            y_hat = y_hat1 + y_hat2 + y_hat3 + y_hat4 + y_hat5
            y_error = y - (y_hat)
            y_error = np.clip(y_error, -1, 1)
            weight_y_error1 = (y_error * w1) * adaptive_gain
            weight_y_error2 = (y_error * w2) * adaptive_gain
            weight_y_error3 = (y_error * w3) * adaptive_gain
            weight_y_error4 = (y_error * w4) * adaptive_gain
            weight_y_error5 = (y_error * w5) * adaptive_gain

            # parameter identifier
            thetas[0], dthetas[0] = self.f_identifier(
                np.ravel(weight_y_error1), dthetas[0])
            thetas[1], dthetas[1] = self.f_identifier(
                np.ravel(weight_y_error2), dthetas[1])
            thetas[2], dthetas[2] = self.f_identifier(
                np.ravel(weight_y_error3), dthetas[2])
            thetas[3], dthetas[3] = self.f_identifier(
                np.ravel(weight_y_error4), dthetas[3])
            thetas[4], dthetas[4] = self.f_identifier(
                np.ravel(weight_y_error5), dthetas[4])

        return normalize_denominator, normalize_offset, thetas

    def f_identifier(self, ddtheta: list[float], dtheta_memory: list[float]) -> tuple[np.ndarray, np.ndarray]:
        dtheta = ddtheta + (dtheta_memory * self.forget_vector)
        dtheta[0] = np.clip(dtheta[0], -self.road_parameter_limit, self.road_parameter_limit)
        theta = dtheta + np.array(self.theta_base)

        return theta, dtheta

    def estimate_yhat(self, x_u: float, normalize_denominator: float, normalize_offset: float, thetas: np.ndarray) -> float:
        x = (x_u - normalize_offset) / normalize_denominator
        xx = x * x
        zeta = np.array([xx, x, 1.]).T
        w1 = self.Weight1_function(x)
        w2 = self.Weight2_function(x)
        w3 = self.Weight3_function(x)
        w4 = self.Weight4_function(x)
        w5 = self.Weight5_function(x)

        y_hat1 = np.dot(thetas[0], zeta.T) * w1
        y_hat2 = np.dot(thetas[1], zeta.T) * w2
        y_hat3 = np.dot(thetas[2], zeta.T) * w3
        y_hat4 = np.dot(thetas[3], zeta.T) * w4
        y_hat5 = np.dot(thetas[4], zeta.T) * w5
        y_hat = y_hat1 + y_hat2 + y_hat3 + y_hat4 + y_hat5

        return y_hat

    def compute_road_risk(self, seek_positions: list[float], normalize_denominator: float, normalize_offset: float, thetas: np.ndarray, side: Side) -> tuple[list[float], float]:
        risks = []
        for seek_p in seek_positions:
            risk = np.zeros(5)
            y_hat = 0.
            seek_x = seek_p[0][2]  # center
            seek_ys = [p for p in seek_p[1]]
            if (seek_x > normalize_offset) and (seek_x < (normalize_offset + normalize_denominator)):
                y_hat = self.estimate_yhat(
                    seek_x, normalize_denominator, normalize_offset, thetas)

                for i in range(len(seek_ys)):
                    # distance between seek_point and road_bound
                    sigma = seek_ys[i] - y_hat

                    if side == Side.RIGHT:
                        risk[i] = self.road_risk_gain * \
                            (-np.arctan(self.road_risk_left_gradient *
                             (sigma+self.road_risk_margin))+1.7)  # right road risk potential
                    elif side == Side.LEFT:
                        risk[i] = self.road_risk_gain * \
                            (np.arctan(self.road_risk_right_gradient *
                             (sigma+self.road_risk_margin))+1.7)  # left road risk potential
            risks.append(risk)
        return risks, y_hat  # list [5x1] x 3

    def benefit_path(self, seek_positions: list[float], y_hat_l: float, y_hat_r: float) -> list[float]:
        benefits = []
        for seek_p in seek_positions:
            benefit = np.zeros(5)
            y_hat_c = (y_hat_l + y_hat_r) / 2
            scale = self.benefit_scale
            covariance = [self.benefit_covariance]
            seek_ys = [p for p in seek_p[1]]

            for i in range(len(seek_ys)):
                benefit[i] = scale * multivariate_normal.pdf(seek_ys[i], mean=y_hat_c, cov=covariance)
            benefits.append(benefit)

        return benefits
