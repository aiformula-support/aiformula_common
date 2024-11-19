import numpy as np

# estimated position from curvatures


class GeometricPoseCurvatures:

    def __init__(self, time_step1, time_step2, time_step3):
        time_steps = [time_step1, time_step2, time_step3]
        self.horizon_steps = np.diff(time_steps, prepend=0)
        self.ego_positions = []
        self.ego_angles = []

    # ---- estimated egocar positions ----
    def estimate_ego_pos(self, ego_velocity, curvature):
        # ego_v: lin_x, ang.z, curvature(curvature): 3 horizon
        # Assume omega is 1/curvature instead ang.z
        # 1st static object estimation only. not yet dynamic object estimation
        seek_point1_ego_position, seek_point1_angle = self.estimate_pos_curvature(
            ego_velocity[0], curvature[0], self.horizon_steps[0])  # (x, y) position
        seek_point2_ego_position, seek_point2_angle = self.estimate_pos_curvature(
            ego_velocity[0], curvature[1], self.horizon_steps[1])
        seek_point3_ego_position, seek_point3_angle = self.estimate_pos_curvature(
            ego_velocity[0], curvature[2], self.horizon_steps[2])

        seek_point2_ego_position_transformed = self.rotate_point(
            seek_point2_ego_position, seek_point1_angle)
        seek_point3_ego_position_transformed = self.rotate_point(
            seek_point3_ego_position, seek_point1_angle + seek_point2_angle)

        seek_point2_angle += seek_point1_angle
        seek_point3_angle += seek_point2_angle

        self.ego_positions = np.vstack(
            [seek_point1_ego_position, seek_point2_ego_position_transformed, seek_point3_ego_position_transformed])
        self.ego_angles = np.array(
            [seek_point1_angle, seek_point2_angle, seek_point3_angle])    # (1,3)

        return self.ego_positions, self.ego_angles

    def create_rotation_matrix(self, angle):
        rotation_matrix = np.array(
            [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        return rotation_matrix

    def rotate_point(self, previous_point, angle):
        point_transformed = (self.create_rotation_matrix(
            angle) @ previous_point.T).T
        return point_transformed

    def estimate_pos(self, ego_velocity, radius, horizon_time):
        distance = ego_velocity * horizon_time
        angle = float(distance / radius)

        if (radius < 1000):  # curve
            x = radius * np.sin(angle)
            y = radius * (1 - np.cos(angle))
        else:  # straight
            x = distance
            y = 0.
            angle = 0.
        return np.array([x, y]), angle

    def estimate_pos_curvature(self, ego_velocity, curvature, horizon_time):
        radius = 1000
        if (abs(curvature) > 0.001):
            radius = 1 / curvature
        return self.estimate_pos(ego_velocity, radius, horizon_time)

    def estimate_direct_seek_y_poss(self, ego_positions, curvature_seek):
        curvature1_seeks = (
            ego_positions[0] + np.array([np.zeros(5), np.array(curvature_seek[0])]).T).T  # 2x5
        curvature2_seeks = (
            ego_positions[1] + np.array([np.zeros(5), np.array(curvature_seek[1])]).T).T
        curvature3_seeks = (
            ego_positions[2] + np.array([np.zeros(5), np.array(curvature_seek[2])]).T).T
        # 3x(2x5)
        return [curvature1_seeks, curvature2_seeks, curvature3_seeks]

    def estimate_base_abs_poss(self, egopos, curvature_seek):
        curvature2_abs_seek = egopos[0].reshape(2, 1) + curvature_seek[1]
        curvature3_abs_seek = (
            egopos[0] + egopos[1]).reshape(2, 1) + curvature_seek[2]

        return [curvature_seek[0], curvature2_abs_seek, curvature3_abs_seek]
