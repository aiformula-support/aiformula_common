import numpy as np
from scipy.stats import multivariate_normal

from rclpy.node import Node

from aiformula_interfaces.msg import ObjectInfoMultiArray
from common_python.get_ros_parameter import get_ros_parameter


class ObjectRiskCalculator:
    def __init__(self, node: Node, buffer_size: int):
        self.init_parameters(node)
        self.init_connections(node, buffer_size)
        self.object_infos = []

    def init_parameters(self, node: Node):
        self.object_risk_variance_x = get_ros_parameter(
            node, "object_risk_potential.object_risk_variance_x")
        self.object_risk_variance_y = get_ros_parameter(
            node, "object_risk_potential.object_risk_variance_y")
        self.object_risk_correlation_coefficient = get_ros_parameter(
            node, "object_risk_potential.object_risk_correlation_coefficient")
        self.object_risk_gain = get_ros_parameter(
            node, "object_risk_potential.object_risk_gain")

    def init_connections(self, node: Node, buffer_size):
        self.object_position_sub = node.create_subscription(
            ObjectInfoMultiArray, 'sub_object_info', self.object_info_callback, buffer_size)

    def object_info_callback(self, msg: ObjectInfoMultiArray) -> None:
        self.object_infos = msg.objects

    def compute_object_risk(self, seek_positions: np.ndarray) -> np.ndarray:
        seek_points_risk = []
        if not self.object_infos:
            seek_points_risk = [np.zeros(seek_positions[0].shape[1]) for _ in range(len(seek_positions))]
            return np.array(seek_points_risk)
        else:
            for seek_position in seek_positions:
                seek_point_risk = self.object_risk(self.object_infos, seek_position)
                seek_points_risk.append(seek_point_risk)

            return np.array(seek_points_risk)

    def object_risk(self, objects: list[float], seek_positions: np.ndarray) -> np.ndarray:
        risks = []
        gaussian_cov = [
            [self.object_risk_variance_x**2,
                self.object_risk_correlation_coefficient*self.object_risk_variance_x*self.object_risk_variance_y],
            [self.object_risk_correlation_coefficient*self.object_risk_variance_x *
                self.object_risk_variance_y, self.object_risk_variance_y**2]
        ]
        for seek_points_idx in range(seek_positions.shape[1]):
            sum_risk = 0.
            for object in objects:
                gaussian_mean = [seek_positions[0, seek_points_idx], seek_positions[1, seek_points_idx]]
                # risk_potential by 2-dimention gauss function
                risk_value = multivariate_normal.pdf([object.x, object.y], mean=gaussian_mean, cov=gaussian_cov)
                sum_risk += risk_value * self.object_risk_gain * object.width * (object.confidence ** 2)

            risks.append(sum_risk)
        return np.array(risks)
