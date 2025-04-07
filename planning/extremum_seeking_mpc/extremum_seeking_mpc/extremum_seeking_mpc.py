import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .path_optimizer import PathOptimizer
from .road_risk_calculator import RoadRiskCalculator
from .pose_predictor import PosePredictor
from .object_risk_calculator import ObjectRiskCalculator
from common_python.get_ros_parameter import get_ros_parameter
from .util import Velocity, Side, Vector2
from aiformula_interfaces.msg import ObjectInfoMultiArray


class ExtremumSeekingMpc(Node):

    def __init__(self):
        super().__init__('extremum_seeking_mpc')
        # Initialize
        self.init_parameters()
        self.init_members()
        self.init_connections()

        self.ego_actual_velocity = None
        self.get_logger().info(f'ego_target_velocity : {self.ego_target_velocity}')

        # initialize tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Extremum_Seeking_MPC parameter
        self.curvatures = [0.] * len(self.predict_horizon)

        # for risk calculation
        self.point_length = np.zeros((Side.NUM_SIDES))
        self.road_offset = np.zeros(Side.NUM_SIDES)
        self.thetas = np.zeros((Side.NUM_SIDES, 5, 3))
        self.object_infos = []

        self.timer = self.create_timer(self.control_time, self.publish_cmd_vel_timer_callback)

    def init_parameters(self):
        self.ego_target_velocity = get_ros_parameter(
            self, "mpc_parameters.planned_speed")
        self.predict_horizon = get_ros_parameter(
            self, "mpc_parameters.horizon_times")
        self.curvature_gain = get_ros_parameter(
            self, "mpc_parameters.curvature_gain")
        self.benefit_gain = get_ros_parameter(
            self, "mpc_parameters.benefit_gain")
        self.deceleration_curvature_threshold = get_ros_parameter(
            self, "velocity_control.deceleration_curvature_threshold")
        self.deceleration_gain = get_ros_parameter(
            self, "velocity_control.deceleration_gain")
        self.base_footprint_frame_id = get_ros_parameter(
            self, "base_footprint_frame_id")
        self.control_time = get_ros_parameter(
            self, "control_time")
        self.buffer_size = get_ros_parameter(
            self, "buffer_size")

    def init_members(self):
        self.object_estimation = ObjectRiskCalculator(self)
        self.road_estimation = RoadRiskCalculator(self)
        self.extremum_seeking_mpc = PathOptimizer(self, self.control_time)
        self.prediction_position = PosePredictor(self, self.predict_horizon, [
                                                 self.extremum_seeking_mpc.extremum_seeking_controller_step1.seek_points] * len(self.predict_horizon))

    def init_connections(self):
        # initialize publishers
        self.twist_pub = self.create_publisher(
            Twist, 'pub_twist_command', self.buffer_size)
        # initialize subscribers
        self.left_lane_line_sub = self.create_subscription(
            PointCloud2, 'sub_road_l', lambda msg: self.lane_line_callback(msg, Side.LEFT), self.buffer_size)
        self.right_lane_line_sub = self.create_subscription(
            PointCloud2, 'sub_road_r', lambda msg: self.lane_line_callback(msg, Side.RIGHT), self.buffer_size)
        self.actucal_speed_sub = self.create_subscription(
            Odometry, 'sub_odom', self.odometry_callback, self.buffer_size)
        self.object_position_sub = self.create_subscription(
            ObjectInfoMultiArray, 'sub_object_info', self.object_info_callback, self.buffer_size)

    def lane_line_callback(self, msg_pointcloud2: PointCloud2, side: Side) -> None:
        pc_data = pc2.read_points(msg_pointcloud2)
        pc_data = np.array(list(pc_data))
        if len(list(pc_data)) > 1:
            xys = np.array(pc_data.tolist())[:, :2]
            self.point_length[side], self.road_offset[side], self.thetas[side] = self.road_estimation.line_identification(
                xys)
        else:
            return

    def odometry_callback(self, odom_msg: Odometry) -> None:
        linear_velocity = -(odom_msg.twist.twist.linear.x)
        self.ego_actual_velocity = Velocity(linear=linear_velocity,
                                            angular=odom_msg.twist.twist.angular.z)

    def object_info_callback(self, msg: ObjectInfoMultiArray) -> None:
        self.object_infos = msg.objects

    def predict_ego_position(self, curvatures: list[float]) -> tuple[list[float], list[float]]:
        ego_positions = self.prediction_position.predict_relative_ego_positions(
            self.ego_actual_velocity.linear, curvatures)
        seek_position_relative = self.prediction_position.predict_relative_seek_positions(ego_positions)
        seek_positions = self.prediction_position.predict_absolute_seek_positions(ego_positions, seek_position_relative)

        return ego_positions, seek_positions

    def calculate_object_risk(self, seek_positions: list[float]) -> list[float]:
        object_risk = self.object_estimation.compute_object_risk(
            self.object_infos, seek_positions)
        return object_risk

    def calculate_road_risk(self, seek_positions: list[float]) -> tuple[list[float], list[float], list[float]]:
        left_road_risk, left_y_hat = self.road_estimation.compute_road_risk(
            seek_positions, self.point_length[Side.LEFT], self.road_offset[Side.LEFT], self.thetas[Side.LEFT], Side.LEFT)  # list [5x1] x 3
        right_road_risk, right_y_hat = self.road_estimation.compute_road_risk(
            seek_positions, self.point_length[Side.RIGHT], self.road_offset[Side.RIGHT], self.thetas[Side.RIGHT], Side.RIGHT)
        benefit = self.road_estimation.benefit_path(
            seek_positions, left_y_hat, right_y_hat)
        return left_road_risk, right_road_risk, benefit

    def calculate_total_risk(self, object_risk: list[float], left_road_risk: list[float], right_road_risk: list[float], benefit: list[float]) -> list[float]:
        total_risk = []
        for idx in range(len(self.predict_horizon)):
            risk_tmp = (np.array(object_risk[idx]) + np.array(left_road_risk[idx]) +
                        np.array(right_road_risk[idx]) - self.benefit_gain * np.array(benefit[idx])).tolist()
            total_risk.append(risk_tmp)

        return total_risk

    def calculate_yaw_rate(self, total_risk: list[float]) -> tuple[float, float, list[float]]:
        curvatures = self.extremum_seeking_mpc.extremum_seeking_mpc_3step(total_risk)
        _, yaw_angle = self.prediction_position.predict_position(
            self.ego_actual_velocity.linear, self.curvatures[0], self.predict_horizon[0])
        yaw_rate = yaw_angle / self.predict_horizon[0]

        return yaw_angle, yaw_rate, curvatures

    def calculate_linear_velocity(self, yaw_angle: float) -> float:
        ang_circle = max(
            (abs(yaw_angle) - self.deceleration_curvature_threshold), 0.)
        vehicle_linear_velocity = max(self.ego_target_velocity[0] -
                                      (ang_circle * self.deceleration_gain), 0.)

        return vehicle_linear_velocity

    def publish_cmd_vel(self, vehicle_linear_velocity: float, yaw_rate: float) -> None:
        twist_msg = Twist()
        twist_msg.linear.x = vehicle_linear_velocity
        twist_msg.angular.z = yaw_rate * self.curvature_gain
        self.twist_pub.publish(twist_msg)

    def tf_viewer(self, ego_positions: list[float]) -> None:
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        parent_frame_ids = ("base_footprint", "seek_point_0", "seek_point_1")
        child_frame_ids = ("seek_point_0", "seek_point_1", "seek_point_2")
        for position, parent_frame_id, child_frame_id in zip(ego_positions, parent_frame_ids, child_frame_ids):
            ts.header.frame_id = parent_frame_id
            ts.child_frame_id = child_frame_id
            ts.transform.translation.x = position[Vector2.X]
            ts.transform.translation.y = position[Vector2.Y]
            ts.transform.translation.z = 0.
            ts.transform.rotation.x = 0.
            ts.transform.rotation.y = 0.
            ts.transform.rotation.z = 0.
            ts.transform.rotation.w = 1.
            self.tf_broadcaster.sendTransform(ts)

    def publish_cmd_vel_timer_callback(self):
        # ---- start extremum_seeking_mpc sequence ----
        ego_positions, seek_positions = self.predict_ego_position(self.curvatures)

        object_risk = self.calculate_object_risk(seek_positions)

        left_road_risk, right_road_risk, benefit = self.calculate_road_risk(seek_positions)

        total_risk = self.calculate_total_risk(object_risk, left_road_risk, right_road_risk, benefit)

        yaw_angle, yaw_rate, self.curvatures = self.calculate_yaw_rate(total_risk)

        vehicle_linear_velocity = self.calculate_linear_velocity(yaw_angle)

        self.publish_cmd_vel(vehicle_linear_velocity, yaw_rate)

        self.tf_viewer(ego_positions)

    def stop_vehicle(self):
        twist_stop_msg = Twist()
        twist_stop_msg.linear.x = 0.0
        twist_stop_msg.angular.z = 0.0
        self.twist_pub.publish(twist_stop_msg)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    extremum_seeking_mpc = ExtremumSeekingMpc()
    try:
        rclpy.spin(extremum_seeking_mpc)
    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt (Ctrl+C), shutting down...")
        extremum_seeking_mpc.stop_vehicle()
    finally:
        extremum_seeking_mpc.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
