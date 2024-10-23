import numpy as np
from scipy.interpolate import interp1d

class ObjectEstimation:
    def __init__(self, risk_table_u, risk_table_y):
        ### it will be ros paramter
        PedestrianRisk_X_u = [-2, -1, -0.6, -0.5, -0.35, -0.2, 0, 0.2, 0.35, 0.5, 0.6, 1, 2]
        PedestrianRisk_X_y = [0, 0, 0, 0.1, 1., 1., 1, 1., 1., 0.1, 0, 0, 0]
        PedestrianRisk_Y_u = risk_table_u
        PedestrianRisk_Y_y = risk_table_y

        self.risk_function_x = interp1d(PedestrianRisk_X_u, PedestrianRisk_X_y)
        self.risk_function_y = interp1d(PedestrianRisk_Y_u, PedestrianRisk_Y_y)

        self.risk_max = [max(PedestrianRisk_X_u), max(PedestrianRisk_Y_u)]
        self.risk_min = [min(PedestrianRisk_X_u), min(PedestrianRisk_Y_u)]

    def risk3step(self, objs, ego_position, ego_angle, seekpos):
        # objs: [x, y] * num (list), seekpos: 3x([x,y]x5) (np.array)
        # Assume omega is 1/curvature instead ang.z

        curvature1_rotation_matrix = np.array([[np.cos(ego_angle[0]), -np.sin(ego_angle[0])], [np.sin(ego_angle[0]), np.cos(ego_angle[0])]]) # 2x2
        curvature2_rotation_matrix = np.array([[np.cos(ego_angle[0] + ego_angle[1]), -np.sin(ego_angle[0] + ego_angle[1])], [np.sin(ego_angle[0] + ego_angle[1]), np.cos(ego_angle[0] + ego_angle[1])]])

        # step1
        curvature1_risk = self.risk_obj_five(objs, seekpos[0]) # list num x (5x1), np.array 2x5
        # step2
        new_objs = np.dot(curvature1_rotation_matrix, (objs - ego_position[0]).T).T # [x,y] * num_obj
        curvature2_risk = self.risk_obj_five(new_objs, seekpos[1])
        # step3
        new_objs = np.dot(curvature2_rotation_matrix, (objs - ego_position[0] - ego_position[1]).T).T
        curvature3_risk = self.risk_obj_five(new_objs, seekpos[2])
        
        return [curvature1_risk, curvature2_risk, curvature3_risk] # list [5x1] x 3

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
                sum_risk += self.risk_function_x(u_x).tolist() * self.risk_function_y(u_y).tolist()
            risks.append(sum_risk)

        return risks # list 5x1
