import numpy as np
from scipy.interpolate import interp1d


class ObjectEstimation:
    def __init__(self, risk_table_u, risk_table_y):
        # it will be ros paramter
        PedestrianRisk_X_u = [-2, -1, -0.6, -0.5, -
                              0.35, -0.2, 0, 0.2, 0.35, 0.5, 0.6, 1, 2]
        PedestrianRisk_X_y = [0, 0, 0, 0.1, 1., 1., 1, 1., 1., 0.1, 0, 0, 0]
        PedestrianRisk_Y_u = risk_table_u
        PedestrianRisk_Y_y = risk_table_y

        self.risk_function_x = interp1d(PedestrianRisk_X_u, PedestrianRisk_X_y)
        self.risk_function_y = interp1d(PedestrianRisk_Y_u, PedestrianRisk_Y_y)

        self.risk_max = [max(PedestrianRisk_X_u), max(PedestrianRisk_Y_u)]
        self.risk_min = [min(PedestrianRisk_X_u), min(PedestrianRisk_Y_u)]

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
