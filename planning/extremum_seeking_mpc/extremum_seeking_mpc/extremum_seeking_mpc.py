import numpy as np
import signal
import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, TransformStamped, PoseArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from tf2_ros import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .exs_mpc_script import OptimizePath
from .exs_road_script import RoadEstimation
from .exs_geometric_position import GeometricPoseCurvatures
from .exs_objrisk_script import ObjectEstimation
from common_python.get_ros_parameter import get_ros_parameter
from .util import Velocity, Side
from aiformula_interfaces.msg import ObjectInfo, ObjectInfoMultiArray


class ExtremumSeekingMpc(Node):

    def __init__(self):
        super().__init__('extremum_seeking_mpc')
        # Initialize
        self.init_parameters()
        self.init_members()
        self.init_connections()

        # for debug
        # self.actual_ego_v = None
        # self.actual_ego_v = Velocity(linear=self.ego_v_target, angular=0.0)
        self.actual_ego_v = Velocity(linear=2.0, angular=0.0)
        self.get_logger().info(f'ego_v_target : {self.ego_v_target}')

        # initialize tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # cmd_vel Message
        self.twist_msg = Twist()

        # EXS_MPC parameter
        self.curvatures = [0] * len(self.predict_horizon)
        self.seek_y_positions = [
            self.exs_mpc.extremum_seeking_controller.seek_points] * len(self.predict_horizon)  # sin is divided by fixed value of 5

        # for risk calculation
        self.object_risk = []
        self.left_road_risk = []
        self.right_road_risk = []
        self.point_length = np.zeros((Side.NUM_SIDES))
        self.road_offset = np.zeros(Side.NUM_SIDES)
        self.thetas = np.zeros((Side.NUM_SIDES, 5, 3))
        self.benefit = []
        self.object_position = []

        # Ctrl + C
        signal.signal(signal.SIGINT, self.reset_command)

        self.timer = self.create_timer(
            0.01, self.publish_cmd_vel_timer_callback)

    def init_parameters(self):
        self.ego_v_target = get_ros_parameter(
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

    def init_members(self):
        self.object_estimation = ObjectEstimation(self)
        self.road_estimation = RoadEstimation(self)
        self.prediction_position = GeometricPoseCurvatures(
            self, self.predict_horizon)
        self.exs_mpc = OptimizePath(self)

    def init_connections(self):
        buffer_size = 10
        # initialize publishers
        self.twist_pub = self.create_publisher(
            Twist, 'pub_twist_command', buffer_size)
        # initialize subscribers
        self.left_lane_line_sub = self.create_subscription(
            PointCloud2, 'sub_road_l', lambda msg: self.lane_line_callback(msg, Side.LEFT), buffer_size)
        self.right_lane_line_sub = self.create_subscription(
            PointCloud2, 'sub_road_r', lambda msg: self.lane_line_callback(msg, Side.RIGHT), buffer_size)
        self.actucal_speed_sub = self.create_subscription(
            Twist, 'sub_actual_speed', self.actual_speed_callback, buffer_size)

        # New!
        self.object_position_sub = self.create_subscription(
            ObjectInfoMultiArray, '/aiformula_perception/object_publisher/object_info', self.object_position_callback, buffer_size)

    def lane_line_callback(self, msg_pointcloud2: PointCloud2, side: Side) -> None:
        pc_data = pc2.read_points(msg_pointcloud2)
        pc_data = np.array(list(pc_data))
        if len(list(pc_data)) > 1:
            xys = np.array(pc_data.tolist())[:, :2]  # Nx3
            self.point_length[side], self.road_offset[side], self.thetas[side] = self.road_estimation.line_identification(
                xys)  # thetas: 5x(1x3)
        else:
            return

    def actual_speed_callback(self, twist_msg):
        self.actual_ego_v = Velocity(linear=twist_msg.linear.x,
                                     angular=twist_msg.angular.z)

    # New!

    def object_position_callback(self, msg):
        # print("callback")
        # print(msg.objects)
        # print(msg.objects.x)
        x = 0.0
        y = 0.0
        obj_width = 0.0
        confidence = 0.0
        points2 = []
        for obj in msg.objects:
            x = obj.x
            y = obj.y
            obj_width = obj.width
            confidence = obj.confidence
            points2.append([x, y, obj_width, confidence])
        points = list([x, y])
        # print(points2)
        # self.get_logger().info(f"received pose: x = {x}, y = {y}")
        # if not points:
        #    self.object_position = []
        #    return
        # print(points)
        self.object_position = points2

    def predict_ego_position(self):
        self.ego_position, self.ego_angle = self.prediction_position.predict_relative_ego_positions(
            self.actual_ego_v.linear, self.curvatures)  # 3x2, 1x3
        self.seek_position_relative = self.prediction_position.predict_relative_seek_positions(
            self.ego_position, self.seek_y_positions)  # 3x(2x5)
        self.seek_position_absolute = self.prediction_position.predict_absolute_seek_positions(
            self.ego_position, self.seek_position_relative)

    def calculate_object_risk(self):
        # self.object_risk = self.object_estimation.risk_3step(
        #    self.object_position, self.ego_position, self.ego_angle, self.seek_position_relative)  # list [5x1] x 3
        self.object_risk = self.object_estimation.risk3step_simple(
            self.object_position, self.seek_position_absolute)  # list [5x1] x 3
        # print(f"obj_risk:{self.object_risk}")

    def calculate_road_risk(self):
        self.left_road_risk, left_y_hat = self.road_estimation.calculation_road_risk(
            self.seek_position_absolute, self.point_length[Side.LEFT], self.road_offset[Side.LEFT], self.thetas[Side.LEFT], Side.LEFT)  # list [5x1] x 3
        self.right_road_risk, right_y_hat = self.road_estimation.calculation_road_risk(
            self.seek_position_absolute, self.point_length[Side.RIGHT], self.road_offset[Side.RIGHT], self.thetas[Side.RIGHT], Side.RIGHT)
        self.benefit = self.road_estimation.benefit_path(
            self.seek_position_absolute, left_y_hat, right_y_hat)

    def calculate_total_risk(self):
        self.total_risk = []
        risk_tmp = (np.array(self.object_risk[0]) + np.array(self.left_road_risk[0]) + np.array(
            self.right_road_risk[0]) - self.benefit_gain * np.array(self.benefit[0])).tolist()
        self.total_risk.append(risk_tmp)
        risk_tmp = (np.array(self.object_risk[1]) + np.array(self.left_road_risk[1]) + np.array(
            self.right_road_risk[1]) - self.benefit_gain * np.array(self.benefit[1])).tolist()
        self.total_risk.append(risk_tmp)
        risk_tmp = (np.array(self.object_risk[2]) + np.array(self.left_road_risk[2]) + np.array(
            self.right_road_risk[2]) - self.benefit_gain * np.array(self.benefit[2])).tolist()
        self.total_risk.append(risk_tmp)  # list [5x1] x 3

    def calculate_yaw_rate(self):
        self.curvatures = self.exs_mpc.extremum_seeking_mpc_3step(
            self.total_risk[0], self.total_risk[1], self.total_risk[2])  # 3 points
        _, self.yaw_angle = self.prediction_position.predict_position(
            self.actual_ego_v.linear, self.curvatures[0], self.predict_horizon[0])
        self.yaw_rate = self.yaw_angle / self.predict_horizon[0]

    def calculate_linear_velocity(self):
        ang_circle = max(
            (abs(self.yaw_angle) - self.deceleration_curvature_threshold), 0)
        self.vehicle_linear_velocity = max(self.ego_v_target[0] -
                                           (ang_circle * self.deceleration_gain), 0.)

    def publish_cmd_vel(self):
        self.twist_msg.linear.x = self.vehicle_linear_velocity
        self.twist_msg.angular.z = self.yaw_rate * self.curvature_gain
        self.twist_pub.publish(self.twist_msg)

    def tf_viewer(self, ego_position):
        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = self.base_footprint_frame_id
        ts.child_frame_id = "seek_point1"
        ts.transform.translation.x = ego_position[0][0]
        ts.transform.translation.y = ego_position[0][1]
        ts.transform.translation.z = 0.
        ts.transform.rotation.x = 0.
        ts.transform.rotation.y = 0.
        ts.transform.rotation.z = 0.
        ts.transform.rotation.w = 1.
        self.tf_broadcaster.sendTransform(ts)
        ts.header.frame_id = "seek_point1"
        ts.child_frame_id = "seek_point2"
        ts.transform.translation.x = ego_position[1][0]
        ts.transform.translation.y = ego_position[1][1]
        self.tf_broadcaster.sendTransform(ts)
        ts.header.frame_id = "seek_point2"
        ts.child_frame_id = "seek_point3"
        ts.transform.translation.x = ego_position[2][0]
        ts.transform.translation.y = ego_position[2][1]
        self.tf_broadcaster.sendTransform(ts)

    def publish_cmd_vel_timer_callback(self):
        # ---- get object position ----
        # self.object_position = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        # ---- start exs_mpc sequence ----
        # predict egocar position
        self.predict_ego_position()

        # object risk (fixed position only, not yet dynamic position (ex. kalmanfilter))
        self.calculate_object_risk()

        # road risk
        self.calculate_road_risk()

        # risk seeking
        self.calculate_total_risk()

        # calculate yaw rate
        self.calculate_yaw_rate()

        # for test, spped circle
        self.calculate_linear_velocity()

        # publish cmd_vel
        self.publish_cmd_vel()

        # visualize tf
        self.tf_viewer(self.ego_position)

    def reset_command(self, sig, frame):  # Ctrl + C
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.twist_pub.publish(self.twist_msg)
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    extremum_seeking_mpc = ExtremumSeekingMpc()
    try:
        rclpy.spin(extremum_seeking_mpc)
    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt (Ctrl+C), shutting down...")
    finally:
        extremum_seeking_mpc.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
