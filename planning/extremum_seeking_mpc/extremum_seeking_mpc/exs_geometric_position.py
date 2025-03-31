import numpy as np
from rclpy.node import Node

from common_python.get_ros_parameter import get_ros_parameter
from .util import Position, Pose

# --- predict egpcar position from curvatures ---


class GeometricPoseCurvatures:

    def __init__(self, node: Node, horizon_times):
        self.init_parameters(node)
        self.horizon_durations = np.diff(horizon_times, prepend=0)
        self.horizon_length = len(horizon_times)
        self.ego_positions = []
        self.ego_angles = []

    def init_parameters(self, node: Node):
        self.curvature_radius_threshold = get_ros_parameter(
            node, "mpc_parameters.curvature_radius_threshold")

    # ---- predict egocar positions ----
    def predict_relative_ego_positions(self, ego_velocity, curvatures):
        # ego_v: lin_x, ang.z, curvature(curvature): 3 horizon
        # Assume omega is 1/curvature instead ang.z

        # Initialize
        predicted_positions = [Pose(pos=Position(x=0.0, y=0.0), yaw=0.0)
                               for _ in range(self.horizon_length)]
        predicted_positions_rotate_transformed = list(
            np.zeros(self.horizon_length-1))

        # Update predict position
        for idx, (curvature, horizon_duration) in enumerate(zip(curvatures, self.horizon_durations)):
            predicted_positions[idx].pos, predicted_positions[idx].yaw = self.predict_position(
                ego_velocity, curvature, horizon_duration)

        # Rotate Point
        for horizon_idx in range(self.horizon_length - 1):
            # calculate yaw
            yaw_to_apply = predicted_positions[horizon_idx].yaw + (
                predicted_positions[horizon_idx+1].yaw if horizon_idx > 0 else 0)

            # apply yaw angle to position
            predicted_positions_rotate_transformed[horizon_idx] = self.rotate_position(
                predicted_positions[horizon_idx+1].pos, yaw_to_apply)

        # Update yaw angle
        for i in range(1, self.horizon_length):
            predicted_positions[i].yaw = predicted_positions[i - 1].yaw

        self.ego_positions = np.vstack(
            [predicted_positions[0].pos, predicted_positions_rotate_transformed])
        self.ego_angles = np.array(
            [predicted_position.yaw for predicted_position in predicted_positions])

        return self.ego_positions, self.ego_angles

    def create_rotation_matrix(self, angle):
        rotation_matrix = np.array(
            [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        return rotation_matrix

    def rotate_position(self, position, angle):
        position_transformed = (self.create_rotation_matrix(
            angle) @ np.array(position).T).T
        return position_transformed

    def predict_position(self, ego_velocity, curvature, horizon_time):
        radius = self.curvature_radius_threshold
        if (abs(curvature) > (1/self.curvature_radius_threshold)):
            radius = 1. / curvature
        travel_distance = ego_velocity * horizon_time
        arc_angle = float(travel_distance / radius)
        if (radius < self.curvature_radius_threshold):  # curve
            x = radius * np.sin(arc_angle)
            y = radius * (1 - np.cos(arc_angle))
        else:  # straight
            x = travel_distance
            y = 0.
            arc_angle = 0.
        return np.array([x, y]), arc_angle

    def predict_relative_seek_positions(self, ego_positions, seek_y_positions):
        relative_seek_positions = list(np.zeros(self.horizon_length))
        for idx in range(self.horizon_length):
            relative_seek_positions[idx] = (
                ego_positions[idx] + np.array([np.zeros(5), np.array(seek_y_positions[idx])]).T).T

        return relative_seek_positions

    def predict_absolute_seek_positions(self, ego_positions, relative_seek_positions):
        absolute_seek_positions = list(
            np.zeros((len(relative_seek_positions[0]), len(relative_seek_positions[0][0]))))

        for idx in range(self.horizon_length-1):
            if idx == 0:
                absolute_seek_positions[idx] = ego_positions[idx].reshape(
                    2, 1) + relative_seek_positions[idx+1]
            else:
                absolute_seek_positions[idx] = (
                    ego_positions[idx-1] + ego_positions[idx]).reshape(2, 1) + relative_seek_positions[idx+1]

        absolute_seek_positions = [
            relative_seek_positions[0]] + absolute_seek_positions

        return list(absolute_seek_positions)
