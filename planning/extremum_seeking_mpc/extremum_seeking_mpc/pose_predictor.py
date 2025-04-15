import numpy as np

from nav_msgs.msg import Odometry
from rclpy.node import Node

from common_python.get_ros_parameter import get_ros_parameter
from .util import Pose, Velocity


class PosePredictor:

    def __init__(self, node: Node, horizon_times: list[float], seek_y_positions: np.ndarray, buffer_size: int):
        self.init_parameters(node)
        self.init_connections(node, buffer_size)
        self.ego_actual_velocity = None
        self.horizon_durations = np.diff(horizon_times, prepend=0)
        self.horizon_length = len(horizon_times)
        self.seek_y_positions = seek_y_positions

    def init_parameters(self, node: Node):
        self.curvature_radius_threshold = get_ros_parameter(
            node, "curvature_radius_threshold")

    def init_connections(self, node: Node, buffer_size: int):
        self.actucal_speed_sub = node.create_subscription(
            Odometry, 'sub_odom', self.odometry_callback, buffer_size)

    def odometry_callback(self, odom_msg: Odometry) -> None:
        linear_velocity = np.sqrt(odom_msg.twist.twist.linear.x ** 2 + odom_msg.twist.twist.linear.y ** 2)
        self.ego_actual_velocity = Velocity(linear=linear_velocity,
                                            angular=odom_msg.twist.twist.angular.z)

    def predict_relative_ego_positions(self, curvatures: np.ndarray) -> np.ndarray:
        # ego_v: lin_x, ang.z, curvature(curvature): 3 horizon
        # Assume omega is 1/curvature instead ang.z

        # Initialize
        predicted_positions = [Pose()for _ in range(self.horizon_length)]
        predicted_positions_rotate_transformed = np.zeros((self.horizon_length - 1, 2))

        # Update predict position
        for curvature, horizon_duration, predicted_position in zip(curvatures, self.horizon_durations, predicted_positions):
            predicted_position.pos, predicted_position.yaw = self.predict_position(curvature, horizon_duration)

        # Rotate Point
        for horizon_idx in range(self.horizon_length - 1):
            # calculate yaw
            rotation_angle = predicted_positions[horizon_idx].yaw
            if horizon_idx:
                rotation_angle += predicted_positions[horizon_idx+1].yaw

            # apply yaw angle to position
            predicted_positions_rotate_transformed[horizon_idx] = self.rotate_position(
                predicted_positions[horizon_idx+1].pos, rotation_angle)

        # Update yaw angle
        for i in range(1, self.horizon_length):
            predicted_positions[i].yaw = predicted_positions[i - 1].yaw

        ego_positions = np.vstack(
            [predicted_positions[0].pos, predicted_positions_rotate_transformed])

        return ego_positions

    def create_rotation_matrix(self, angle: float) -> np.ndarray:
        rotation_matrix = np.array(
            [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        return rotation_matrix

    def rotate_position(self, position: np.ndarray, angle: float) -> np.ndarray:
        rotation_matrix = self.create_rotation_matrix(angle)
        position_transformed = (rotation_matrix @ np.array(position).T).T
        return position_transformed

    def predict_position(self, curvature: float, horizon_time: float) -> tuple[np.ndarray, float]:
        radius = self.curvature_radius_threshold
        if (abs(curvature) > (1./self.curvature_radius_threshold)):
            radius = 1. / curvature
        travel_distance = self.ego_actual_velocity.linear * horizon_time
        arc_angle = float(travel_distance / radius)
        if (radius < self.curvature_radius_threshold):  # curve
            x = radius * np.sin(arc_angle)
            y = radius * (1. - np.cos(arc_angle))
        else:  # straight
            x = travel_distance
            y = 0.
            arc_angle = 0.
        return np.array([x, y]), arc_angle

    def predict_relative_seek_positions(self, ego_positions: np.ndarray) -> np.ndarray:
        relative_seek_positions = list(np.zeros(self.horizon_length))
        for idx in range(self.horizon_length):
            relative_seek_positions[idx] = (
                ego_positions[idx] + np.array([np.zeros(5), np.array(self.seek_y_positions[idx])]).T).T

        return np.array(relative_seek_positions)

    def predict_absolute_seek_positions(self, ego_positions: np.ndarray, relative_seek_positions: np.ndarray) -> np.ndarray:
        absolute_seek_positions = list(
            np.zeros((len(relative_seek_positions[0]), len(relative_seek_positions[0][0]))))

        for idx in range(self.horizon_length - 1):
            prev_position = ego_positions[idx-1] if idx > 0 else ego_positions[idx]
            absolute_seek_positions[idx] = (prev_position + ego_positions[idx]
                                            ).reshape(2, 1) + relative_seek_positions[idx+1]

        absolute_seek_positions = [
            relative_seek_positions[0]] + absolute_seek_positions

        return np.array(absolute_seek_positions)
