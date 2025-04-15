from dataclasses import dataclass
import numpy as np
from scipy.interpolate import interp1d
from scipy.stats import multivariate_normal

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from common_python.get_ros_parameter import get_ros_parameter
from .util import Side


class RoadRiskCalculator:
    @dataclass
    class Params:
        identification_gain: int
        forget_vector: list[float]

    def __init__(self, node: Node, buffer_size: int):
        params = self.init_parameters(node)
        self.init_connections(node, buffer_size)

        self.point_length = np.zeros((Side.NUM_SIDES))
        self.road_offset = np.zeros(Side.NUM_SIDES)
        self.road_thetas = np.zeros((Side.NUM_SIDES, 5, 3))

        weight_u = self.road_weight_u
        weight_y = np.zeros((5, len(weight_u)))

        weight_indices = [
            slice(None, 2),
            2,
            3,
            4,
            slice(5, None)
        ]

        self.Weight_functions = []

        for idx, weight_idx in enumerate(weight_indices):
            weight_y[idx, weight_idx] = 1.0
            interpolated_function = interp1d(weight_u, weight_y[idx, :])
            self.Weight_functions.append(interpolated_function)

        self.identification_gain_matrix = np.zeros((3, 3))
        np.fill_diagonal(self.identification_gain_matrix, params.identification_gain)
        self.forget_vector = np.array(params.forget_vector)
        self.theta_base = np.zeros(3)
        self.thetas = np.zeros((5, 3))
        self.dthetas = np.zeros((5, 3))

    def init_parameters(self, node: Node) -> Params:
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
        self.max_point_length = get_ros_parameter(
            node, "road_identification.max_point_length")
        identification_gain = get_ros_parameter(
            node, "road_identification.identification_gain")
        forget_vector = get_ros_parameter(
            node, "road_identification.forget_vector")
        self.road_parameter_limit = get_ros_parameter(
            node, "road_identification.road_parameter_limit")
        self.benefit_scale = get_ros_parameter(
            node, "road_benefit_function.scale")
        self.benefit_covariance = get_ros_parameter(
            node, "road_benefit_function.covariance")

        return self.Params(
            identification_gain=identification_gain,
            forget_vector=forget_vector
        )

    def init_connections(self, node: Node, buffer_size):
        self.left_lane_line_sub = node.create_subscription(
            PointCloud2, 'sub_road_l', lambda msg: self.lane_line_callback(msg, Side.LEFT), buffer_size)
        self.right_lane_line_sub = node.create_subscription(
            PointCloud2, 'sub_road_r', lambda msg: self.lane_line_callback(msg, Side.RIGHT), buffer_size)

    def lane_line_callback(self, msg_pointcloud2: PointCloud2, side: Side) -> None:
        pc_iter = pc2.read_points(msg_pointcloud2, field_names=['x', 'y'], skip_nans=True)
        lane_points = np.array(pc_iter.tolist())
        if lane_points.shape[0] < 2:
            return
        self.point_length[side], self.road_offset[side], self.road_thetas[side] = self.line_identification(lane_points)

    def line_identification(self, lane_points: np.ndarray) -> tuple[float, float, np.ndarray]:
        x_coords, y_coords = lane_points[:, 0], lane_points[:, 1]
        x_min = min(x_coords)
        x_range = max(x_coords) - x_min
        normalized_x = (x_coords - x_min) / x_range

        x_vectors = np.column_stack([
            normalized_x ** 2,
            normalized_x,
            np.ones_like(normalized_x)
        ])

        weights_list = np.array([f(normalized_x) for f in self.Weight_functions]).T

        for x_vector, weights, y_coord in zip(x_vectors, weights_list, y_coords):
            adaptive_gain_numerator = self.identification_gain_matrix @ x_vector
            zp = x_vector @ self.identification_gain_matrix
            adaptive_gain_denominator = 1 + np.dot(zp, x_vector)
            adaptive_gain = adaptive_gain_numerator / adaptive_gain_denominator
            weighted_y_hats = self.thetas @ x_vector * weights
            y_error = y_coord - weighted_y_hats.sum()
            weighted_y_errors = np.outer((y_error * weights), adaptive_gain)
            self.thetas, self.dthetas = self.f_identifier(weighted_y_errors, self.dthetas)

        return x_range, x_min, self.thetas

    def f_identifier(self, ddtheta: np.ndarray, dtheta_memory: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        dtheta = ddtheta + (dtheta_memory * self.forget_vector)
        dtheta[:, 0] = np.clip(dtheta[:, 0], -self.road_parameter_limit, self.road_parameter_limit)
        theta = dtheta + np.array(self.theta_base)

        return theta, dtheta

    def estimate_yhat(self, x_u: float, x_range: float, x_min: float, thetas: np.ndarray) -> float:
        normalized_x = (x_u - x_min) / x_range
        x_vectors = np.column_stack([
            normalized_x ** 2,
            normalized_x,
            np.ones_like(normalized_x)
        ])

        weights = [weight_function(normalized_x) for weight_function in self.Weight_functions]
        y_hats = [np.dot(theta, x_vectors.T) * weight for theta, weight in zip(thetas, weights)]

        return sum(y_hats)

    def compute_road_risk(self, seek_positions: np.ndarray, side: Side) -> tuple[np.ndarray, float]:
        risks = []
        y_hat_last = 0.

        for seek_p in seek_positions:
            seek_x = seek_p[0][2]  # center of seek_points
            seek_ys = seek_p[1]
            risk = np.zeros(5)

            if not (self.road_offset[side] < seek_x < self.road_offset[side] + self.point_length[side]):
                risks.append(risk)
                continue

            y_hat = self.estimate_yhat(seek_x, self.point_length[side], self.road_offset[side], self.road_thetas[side])
            y_hat_last = y_hat

            risk = self.compute_risk_for_position(seek_ys, y_hat, side)
            risks.append(risk)

        return np.array(risks), y_hat_last

    def compute_risk_for_position(self, seek_ys: np.ndarray, y_hat: float, side: Side) -> np.ndarray:
        risk = np.zeros(len(seek_ys))
        for idx, y in enumerate(seek_ys):
            sigma = y - y_hat
            if side == Side.RIGHT:
                risk[idx] = self.road_risk_gain * \
                    (-np.arctan(self.road_risk_left_gradient * (sigma + self.road_risk_margin)) + 1.7)
            elif side == Side.LEFT:
                risk[idx] = self.road_risk_gain * \
                    (np.arctan(self.road_risk_right_gradient * (sigma + self.road_risk_margin)) + 1.7)
        return risk

    def benefit_path(self, seek_positions: np.ndarray, y_hat_l: float, y_hat_r: float) -> np.ndarray:
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

        return np.array(benefits)
