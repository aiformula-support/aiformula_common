import numpy as np

# estimated position from curvatures
class geometric_pos_curvatures:

    def __init__(self, t_step1 = 0.1, t_step2 = 0.15, t_step3 = 0.2):

        self.horizon_step = np.array([t_step1, (t_step2-t_step1), (t_step3-t_step2)])
        self.ego_position = []
        self.ego_angle = []
    
    #---- estimated egocar positions ----
    def estimate_ego_pos(self, ego_velocity, curvature):
        # ego_v: lin_x, ang.z, curvature(curvature): 3 horizon
        # Assume omega is 1/curvature instead ang.z
        ##### 1st static object estimation only. not yet dynamic object estimation
        seek_point1_ego_position, seek_point1_angle = self.estimate_pos_curvature(ego_velocity[0], curvature[0], self.horizon_step[0]) # (x, y) position
        seek_point2_ego_position, seek_point2_angle = self.estimate_pos_curvature(ego_velocity[0], curvature[1], self.horizon_step[1])
        seek_point3_ego_position, seek_point3_angle = self.estimate_pos_curvature(ego_velocity[0], curvature[2], self.horizon_step[2])
        
        seek_point1_rotation_matrix = np.array([[np.cos(seek_point1_angle), -np.sin(seek_point1_angle)], [np.sin(seek_point1_angle), np.cos(seek_point1_angle)]])
        seek_point2_rotation_matrix = np.array([[np.cos(seek_point1_angle + seek_point2_angle), -np.sin(seek_point1_angle + seek_point2_angle)], [np.sin(seek_point1_angle + seek_point2_angle), np.cos(seek_point1_angle + seek_point2_angle)]])

        seek_point2_ego_position = np.dot(seek_point1_rotation_matrix, seek_point2_ego_position.T).T
        seek_point3_ego_position = np.dot(seek_point2_rotation_matrix, seek_point3_ego_position.T).T

        seek_point2_angle = seek_point2_angle + seek_point1_angle
        seek_point3_angle = seek_point3_angle + seek_point2_angle

        self.ego_position = np.array([seek_point1_ego_position, seek_point2_ego_position, seek_point3_ego_position]).reshape(3,2) # (1,2) * 3 => (3,2)
        self.ego_angle = np.array([seek_point1_angle, seek_point2_angle, seek_point3_angle])    # (1,3)

        return self.ego_position, self.ego_angle

    def estimate_pos(self, speed, r, t_est):
        l = speed * t_est
        angle = float(l / r)

        if (r < 1000):
            x = r * np.sin(angle)
            y = r * (1 - np.cos(angle))
        else: # straight
            x = l
            y = 0.
            angle = 0.
        return np.array([x, y]), angle

    def estimate_pos_curvature(self, speed, curvature, t_est):
        radius = 1000
        if (abs(curvature) > 0.001):
            radius = 1 / curvature
        return self.estimate_pos(speed, radius, t_est)
    
    def estimate_direct_seek_y_poss(self, ego_position, curvature_seek):
        curvature1_seeks = (ego_position[0] + np.array([np.zeros(5), np.array(curvature_seek[0])]).T).T # 2x5
        curvature2_seeks = (ego_position[1] + np.array([np.zeros(5), np.array(curvature_seek[1])]).T).T
        curvature3_seeks = (ego_position[2] + np.array([np.zeros(5), np.array(curvature_seek[2])]).T).T
        return [curvature1_seeks, curvature2_seeks, curvature3_seeks] # 3x(2x5)
    
    def estimate_base_abs_poss(self, egopos, curvature_seek):
        curvature2_abs_seek = egopos[0].reshape(2,1) + curvature_seek[1]
        curvature3_abs_seek = (egopos[0] + egopos[1]).reshape(2, 1) + curvature_seek[2]

        return [curvature_seek[0], curvature2_abs_seek, curvature3_abs_seek]
