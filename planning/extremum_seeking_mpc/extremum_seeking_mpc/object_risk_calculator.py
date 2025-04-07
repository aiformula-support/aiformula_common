import numpy as np
from rclpy.node import Node

from common_python.get_ros_parameter import get_ros_parameter


class ObjectRiskCalculator:
    def __init__(self, node: Node):
        self.init_parameters(node)

    def init_parameters(self, node: Node):
        self.object_risk_variance_x = get_ros_parameter(
            node, "object_risk_potential.object_risk_variance_x")
        self.object_risk_variance_y = get_ros_parameter(
            node, "object_risk_potential.object_risk_variance_y")
        self.object_risk_correlation_coefficient = get_ros_parameter(
            node, "object_risk_potential.object_risk_correlation_coefficient")
        self.object_risk_gain = get_ros_parameter(
            node, "object_risk_potential.object_risk_gain")

    def compute_object_risk(self, objects: list[float], seek_position: list[float]) -> list[np.ndarray]:
        seek_points_risk = []
        if len(objects) == 0:
            seek_points_risk = [np.zeros(5)] * 3
            return seek_points_risk
        else:
            for idx in range(3):
                seek_point_risk = self.object_risk(objects, seek_position[idx])
                seek_points_risk.append(seek_point_risk)

            return seek_points_risk

    def object_risk(self, objects: list[float], seek_position: list[float]) -> list[float]:
        risks = []
        for turb in range(seek_position.shape[1]):
            sum_risk = 0.
            for object in objects:
                # risk_potential by 2-dimention gauss function
                risk_value = self.bivariate_gaussian(object.x, object.y, seek_position[:, turb][0], seek_position[:, turb]
                                                     [1], self.object_risk_variance_x, self.object_risk_variance_y, self.object_risk_correlation_coefficient)
                sum_risk += risk_value * self.object_risk_gain * object.width * (object.confidence ** 2)

            risks.append(sum_risk)
        return risks

    def bivariate_gaussian(self, object_x: float, object_y: float, seekpoint_x: float, seekpoint_y: float, sigma_x: float, sigma_y: float, coefficient: float) -> float:
        denominator = 2 * np.pi * sigma_x * sigma_y * np.sqrt(1 - coefficient**2)
        exponent = -1 / (2 * (1 - coefficient**2)) * (
            (object_x - seekpoint_x)**2 / sigma_x**2 - 2 * coefficient * (object_x - seekpoint_x) *
            (object_y - seekpoint_y) / (sigma_x * sigma_y) +
            (object_y - seekpoint_y)**2 / sigma_y**2
        )
        return np.exp(exponent) / denominator
