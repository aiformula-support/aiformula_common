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

    def init_parameters(self, node: Node):
        self.object_risk_table_u = get_ros_parameter(
            node, "object_risk_potential.pedestrian_risk_table_u")
        self.object_risk_table_y = get_ros_parameter(
            node, "object_risk_potential.pedestrian_risk_table_y")

    def create_rotation_matrix(self, angle):
        rotation_matrix = np.array(
            [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        return rotation_matrix

    def risk_3step(self, objs, ego_position, ego_angle, seekpos):
        # objs: [x, y] * num (list), seekpos: 3x([x,y]x5) (np.array)
        # Assume omega is 1/curvature instead ang.z
        seek_point1_rotation_matrix = self.create_rotation_matrix(ego_angle[0])
        seek_point2_rotation_matrix = self.create_rotation_matrix(
            ego_angle[0] + ego_angle[1])

        # step1
        seek_point1_risk = self.risk_obj_five(
            objs, seekpos[0])  # list num x (5x1), np.array 2x5
        # step2
        new_objs = np.dot(seek_point1_rotation_matrix,
                          (objs - ego_position[0]).T).T  # [x,y] * num_obj
        seek_point2_risk = self.risk_obj_five(new_objs, seekpos[1])
        # step3
        new_objs = np.dot(seek_point2_rotation_matrix,
                          (objs - ego_position[0] - ego_position[1]).T).T
        seek_point3_risk = self.risk_obj_five(new_objs, seekpos[2])

        # list [5x1] x 3
        return [seek_point1_risk, seek_point2_risk, seek_point3_risk]

    def risk_obj_five(self, objs, ego_positions):
        # obj: (x, y) * 2 list, ego_pos: (x, y) * 5 np.array
        risks = []
        for ego_position in ego_positions.T:
            sum_risk = 0
            distance_to_object = objs - ego_position
            for u in distance_to_object:
                u_x = max(u[0], self.risk_min[0])
                u_x = min(u_x, self.risk_max[0])
                u_y = max(u[1], self.risk_min[1])
                u_y = min(u_y, self.risk_max[1])
                sum_risk += self.risk_function_x(u_x).tolist() * \
                    self.risk_function_y(u_y).tolist()
            risks.append(sum_risk)

        return risks  # list 5x1

    # --- New! --- #

    def risk3step_simple(self, objs, seekpos_abs):
        # print(f"objs: {objs}")
        # print(f"seekpos_abs: {seekpos_abs[0]}")
        if len(objs) == 0:
            curvature_risk = np.zeros(5)
            return [curvature_risk, curvature_risk, curvature_risk]
        else:
            # list num x (5x1), np.array 2x5
            curvature1_risk = self.risk_obj(objs, seekpos_abs[0])
            # list num x (5x1), np.array 2x5
            curvature2_risk = self.risk_obj(objs, seekpos_abs[1])
            # list num x (5x1), np.array 2x5
            curvature3_risk = self.risk_obj(objs, seekpos_abs[2])

            # list [5x1] x 3
            return [curvature1_risk, curvature2_risk, curvature3_risk]

    def risk_obj(self, objs, seekpos_abs):
        # obj: (x, y) * 2 list, ego_pos: (x, y) * 5 np.array
        risks = []
        for turb in range(seekpos_abs.shape[1]):
            sum_risk = 0
            # print(f"objs:{objs}")
            for obj in objs:
                # print(f"obj:{obj[2]}")
                u = obj[0:2] - seekpos_abs[:, turb]
                u_x = np.clip(u[0], self.risk_min[0], self.risk_max[0])
                u_y = np.clip(u[1], self.risk_min[1], self.risk_max[1])
                sum_risk += self.risk_function_x(u_x).tolist() * \
                    self.risk_function_y(u_y).tolist() * \
                    4000.0 * obj[2] * obj[3]
                if sum_risk > 0:
                    print(f"obj_risk:{sum_risk}")

            risks.append(sum_risk)
        return risks  # list 5x1

    def gaussian_func(obj_x, obj_y, seek_x, seek_y, sigma):
        exponent = - ((obj_x - seek_x) ** 2 + (obj_y - seek_y)
                      ** 2) / (2 * pow(sigma, 2))
        denominator = 2 * math.pi * pow(sigma, 2)
        return pow(math.e, exponent) / denominator
