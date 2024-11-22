import numpy as np

# estimated position from curvatures


class GeometricPoseCurvatures:

    def __init__(self, horizon_times):
        self.horizon_steps = np.diff(horizon_times, prepend=0)
        self.horizon_length = len(horizon_times)
        self.ego_positions = []
        self.ego_angles = []
        self.seek_points = list(np.zeros(self.horizon_length))
        self.seek_angles = list(np.zeros(self.horizon_length))
        self.seek_points_transformed = list(np.zeros(self.horizon_length-1))

    # ---- estimated egocar positions ----
    def estimate_ego_pos(self, ego_velocity, curvature):
        # ego_v: lin_x, ang.z, curvature(curvature): 3 horizon
        # Assume omega is 1/curvature instead ang.z
        # 1st static object estimation only. not yet dynamic object estimation
        for step in range(self.horizon_length):
            self.seek_points[step], self.seek_angles[step] = self.estimate_pos_curvature(
                ego_velocity[0], curvature[step], self.horizon_steps[step])

        # Rotate Point
        for step in range(self.horizon_length - 1):
            if step == 0:
                self.seek_points_transformed[step] = self.rotate_point(
                    self.seek_points[step+1], self.seek_angles[step])
            else:
                self.seek_points_transformed[step] = self.rotate_point(
                    self.seek_points[step+1], self.seek_angles[step] + self.seek_angles[step+1])

        for step in range(self.horizon_length - 1):
            self.seek_angles[step+1] = self.seek_angles[step]

        self.ego_positions = np.vstack(
            [self.seek_points[0], self.seek_points_transformed])
        self.ego_angles = np.array(
            self.seek_angles)    # (1,3)

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
        curvature_seeks = list(np.zeros(self.horizon_length))
        for num in range(self.horizon_length):
            curvature_seeks[num] = (
                ego_positions[num] + np.array([np.zeros(5), np.array(curvature_seek[num])]).T).T

        return curvature_seeks

    def estimate_base_abs_poss(self, egopos, curvature_seek):
        curvature_abs_seeks = list(
            np.zeros((len(curvature_seek[0]), len(curvature_seek[0][0]))))

        for num in range(self.horizon_length-1):
            if num == 0:
                curvature_abs_seeks[num] = egopos[num].reshape(
                    2, 1) + curvature_seek[num+1]
            else:
                curvature_abs_seeks[num] = (
                    egopos[num-1] + egopos[num]).reshape(2, 1) + curvature_seek[num+1]

        curvature_abs_seekss = [curvature_seek[0]]
        curvature_abs_seekss.extend(curvature_abs_seeks)

        return list(curvature_abs_seekss)
