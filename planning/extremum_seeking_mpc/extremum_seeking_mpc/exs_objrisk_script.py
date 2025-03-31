import math
import numpy as np
from rclpy.node import Node
from scipy.interpolate import interp1d

from common_python.get_ros_parameter import get_ros_parameter


class ObjectEstimation:
    def __init__(self, node: Node):
        self.init_parameters(node)

        ObjectRisk_X_u = self.object_risk_table_x_u
        ObjectRisk_X_y = self.object_risk_table_x_y
        ObjectRisk_Y_u = self.object_risk_table_y_u
        ObjectRisk_Y_y = self.object_risk_table_y_y

        self.risk_function_x = interp1d(ObjectRisk_X_u, ObjectRisk_X_y)
        self.risk_function_y = interp1d(ObjectRisk_Y_u, ObjectRisk_Y_y)

        self.risk_max = [max(ObjectRisk_X_u), max(ObjectRisk_Y_u)]
        self.risk_min = [min(ObjectRisk_X_u), min(ObjectRisk_Y_u)]

    def init_parameters(self, node: Node):
        self.object_risk_table_x_u = get_ros_parameter(
            node, "object_risk_potential.object_risk_table_x_u")
        self.object_risk_table_x_y = get_ros_parameter(
            node, "object_risk_potential.object_risk_table_x_y")
        self.object_risk_table_y_u = get_ros_parameter(
            node, "object_risk_potential.object_risk_table_y_u")
        self.object_risk_table_y_y = get_ros_parameter(
            node, "object_risk_potential.object_risk_table_y_y")
        self.object_risk_variance_x = get_ros_parameter(
            node, "object_risk_potential.object_risk_variance_x")
        self.object_risk_variance_y = get_ros_parameter(
            node, "object_risk_potential.object_risk_variance_y")
        self.object_risk_correlation_coefficient = get_ros_parameter(
            node, "object_risk_potential.object_risk_correlation_coefficient")
        self.object_risk_gain = get_ros_parameter(
            node, "object_risk_potential.object_risk_gain")

    def calculation_object_risk(self, objs, seek_position_absolute):
        if len(objs) == 0:
            seekpoint_risk = np.zeros(5)
            return [seekpoint_risk, seekpoint_risk, seekpoint_risk]
        else:
            # list num x (5x1), np.array 2x5
            seek_point1_risk = self.object_risk(
                objs, seek_position_absolute[0])
            seek_point2_risk = self.object_risk(
                objs, seek_position_absolute[1])
            seek_point3_risk = self.object_risk(
                objs, seek_position_absolute[2])

            # list [5x1] x 3
            return [seek_point1_risk, seek_point2_risk, seek_point3_risk]

    def object_risk(self, objs, seek_position_absolute):
        # obj: (x, y) * 2 list, ego_pos: (x, y) * 5 np.array
        risks = []
        for turb in range(seek_position_absolute.shape[1]):
            sum_risk = 0
            for obj in objs:
                # risk_potential by 2-dimention gauss function
                risk_value = self.bivariate_gaussian(obj[0], obj[1], seek_position_absolute[:, turb][0], seek_position_absolute[:, turb]
                                                     [1], self.object_risk_variance_x, self.object_risk_variance_y, self.object_risk_correlation_coefficient)
                sum_risk += risk_value * \
                    self.object_risk_gain * obj[2] * (obj[3] ** 2)

            risks.append(sum_risk)
        return risks  # list 5x1

    def bivariate_gaussian(self, object_x, object_y, seekpoint_x, seekpoint_y, sigma_x, sigma_y, coefficient):
        denominator = 2 * np.pi * sigma_x * \
            sigma_y * np.sqrt(1 - coefficient**2)
        exponent = -1 / (2 * (1 - coefficient**2)) * (
            (object_x - seekpoint_x)**2 / sigma_x**2 - 2 * coefficient * (object_x - seekpoint_x) *
            (object_y - seekpoint_y) / (sigma_x * sigma_y) +
            (object_y - seekpoint_y)**2 / sigma_y**2
        )
        return np.exp(exponent) / denominator
