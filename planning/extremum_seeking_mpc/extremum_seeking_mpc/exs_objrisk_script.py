import math
import numpy as np
from rclpy.node import Node
from scipy.interpolate import interp1d

from common_python.get_ros_parameter import get_ros_parameter


class ObjectEstimation:
    def __init__(self, node: Node):
        self.init_parameters(node)
        # it will be ros paramter
        PedestrianRisk_X_u = [-3, -2, -0.6, -0.5, -
                              0.35, -0.2, 0, 0.2, 0.35, 0.5, 0.6, 2, 3]
        PedestrianRisk_X_y = [0, 0.3, 0.4, 0.6,
                              0.8, 1., 1., 1., 0.8, 0.6, 0.4, 0.3, 0]
        PedestrianRisk_Y_u = self.object_risk_table_u
        PedestrianRisk_Y_y = self.object_risk_table_y

        self.risk_function_x = interp1d(PedestrianRisk_X_u, PedestrianRisk_X_y)
        self.risk_function_y = interp1d(PedestrianRisk_Y_u, PedestrianRisk_Y_y)

        self.risk_max = [max(PedestrianRisk_X_u), max(PedestrianRisk_Y_u)]
        self.risk_min = [min(PedestrianRisk_X_u), min(PedestrianRisk_Y_u)]

        self.object_risk_gain = 100

    def init_parameters(self, node: Node):
        self.object_risk_table_u = get_ros_parameter(
            node, "object_risk_potential.pedestrian_risk_table_u")
        self.object_risk_table_y = get_ros_parameter(
            node, "object_risk_potential.pedestrian_risk_table_y")

    def calculation_object_risk(self, objs, seekpos_abs):
        if len(objs) == 0:
            curvature_risk = np.zeros(5)
            return [curvature_risk, curvature_risk, curvature_risk]
        else:
            # list num x (5x1), np.array 2x5
            seek_point1_risk = self.object_risk(objs, seekpos_abs[0])
            # list num x (5x1), np.array 2x5
            seek_point2_risk = self.object_risk(objs, seekpos_abs[1])
            # list num x (5x1), np.array 2x5
            seek_point3_risk = self.object_risk(objs, seekpos_abs[2])

            # list [5x1] x 3
            return [seek_point1_risk, seek_point2_risk, seek_point3_risk]

    def object_risk(self, objs, seekpos_abs):
        # obj: (x, y) * 2 list, ego_pos: (x, y) * 5 np.array
        risks = []
        for turb in range(seekpos_abs.shape[1]):
            sum_risk = 0
            for obj in objs:
                a = self.gaussian_func(obj[0],obj[1],seekpos_abs[:, turb][0],seekpos_abs[:, turb][1],0.8)
                #print(f"a:{a}")
                #print(f"obj[2]:{obj[2]}")
                u = obj[0:2] - seekpos_abs[:, turb]
                u_x = np.clip(u[0], self.risk_min[0], self.risk_max[0])
                u_y = np.clip(u[1], self.risk_min[1], self.risk_max[1])
                #sum_risk += self.risk_function_x(u_x).tolist() * \
                #    self.risk_function_y(u_y).tolist() * \
                #    self.object_risk_gain * obj[2] * obj[3]
                sum_risk += a * self.object_risk_gain * obj[2] * obj[3]
                #if sum_risk > 0:
                #    print(f"obj_risk:{sum_risk}")

            risks.append(sum_risk)
        return risks  # list 5x1

    def gaussian_func(self, obj_x, obj_y, seek_x, seek_y, sigma):
        exponent = -((obj_x - seek_x) ** 2 + (obj_y - seek_y) ** 2) / (2 * pow(sigma, 2))
        denominator = 2 * math.pi * pow(sigma, 2)
        return pow(math.e, exponent) / denominator
