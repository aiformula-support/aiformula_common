import numpy as np
import signal
import sys
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
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


class ExtremumSeekingMpc(Node):

    def __init__(self):
        super().__init__('extremum_seeking_mpc')
        # Initialize
        self.init_parameters()
        self.init_members()
        self.init_connections()

        # for debug
        self.ego_v = self.ego_v_target
        self.get_logger().info(f'ego_v_target : {self.ego_v_target}')

        # initialize tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # cmd_vel Message
        self.vel_msg = Twist()

        # EXS_MPC parameter
        self.curvatures = [0] * len(self.predict_horizon)
        self.curvature_seeks = [self.exs_mpc.extremum_seeking_controller.seek_points,
                                self.exs_mpc.extremum_seeking_controller.seek_points,
                                self.exs_mpc.extremum_seeking_controller.seek_points]  # csa type: sin is dibided by fixed value of 8

        # for risk calculation
        self.object_risk = []
        self.left_road_risk = []
        self.right_road_risk = []
        self.left_point_length = 0.
        self.left_road_offset = 0.
        self.left_thetas = np.zeros((5, 3))  # 5x3
        self.right_point_length = 0.
        self.right_road_offset = 0.
        self.right_thetas = np.zeros((5, 3))  # 5x3
        self.benefit = []

        # Ctrl + C
        signal.signal(signal.SIGINT, self.reset_command)

        self.timer = self.create_timer(0.01, self.on_timer)

    def init_parameters(self):
        self.object_risk_table_u = get_ros_parameter(
            self, "object_risk_potential.pedestrian_risk_table_u")
        self.object_risk_table_y = get_ros_parameter(
            self, "object_risk_potential.pedestrian_risk_table_y")
        self.road_sigma = get_ros_parameter(
            self, "road_risk_potential.road_risk_table_u")
        self.road_risk_table = get_ros_parameter(
            self, "road_risk_potential.road_risk_table_y")
        self.ego_v_target = get_ros_parameter(
            self, "mpc_parameters.planned_speed")
        self.predict_horizon = get_ros_parameter(
            self, "mpc_parameters.horizon_times")
        self.seek_gain = get_ros_parameter(
            self, "mpc_parameters.seek_gain")
        self.seek_amp = get_ros_parameter(
            self, "mpc_parameters.seek_amp")
        self.curvature_limit = get_ros_parameter(
            self, "mpc_parameters.curvature_limit")
        self.curvature_gain = get_ros_parameter(
            self, "mpc_parameters.curvature_gain")
        self.benefit_gain = get_ros_parameter(
            self, "mpc_parameters.benefit_gain")
        self.speed_circle = get_ros_parameter(
            self, "velocity_control.speed_circle")
        self.deceleration_gain = get_ros_parameter(
            self, "velocity_control.deceleration_gain")
        self.frame_id = get_ros_parameter(
            self, "tf_frame_id")

    def init_members(self):
        self.object_estimation = ObjectEstimation(
            self.object_risk_table_u, self.object_risk_table_y)
        self.road_estimation = RoadEstimation(
            self.road_sigma, self.road_risk_table)
        self.pos_estimation = GeometricPoseCurvatures(
            self.predict_horizon)
        self.exs_mpc = OptimizePath(
            self.seek_gain, self.seek_amp, self.curvature_limit, -(self.curvature_limit))

    def init_connections(self):
        # initialize publishers
        buffer_size = 10
        self.pub_cmd = self.create_publisher(
            Twist, 'pub_speed_command', buffer_size)
        self.sub_left_road = self.create_subscription(
            PointCloud2, 'sub_road_l', self.listener_left_road, buffer_size)
        self.sub_right_road = self.create_subscription(
            PointCloud2, 'sub_road_r', self.listener_right_road, buffer_size)
        self.sub_ego_velocity = self.create_subscription(
            Twist, 'sub_ego_velocity', self.listener_ego_velocity, buffer_size)

    def listener_left_road(self, msg_pointcloud2: PointCloud2) -> [int, int, list]:
        # def listener_left_road(self, msg_pointcloud2):
        pc_data = pc2.read_points(msg_pointcloud2)
        pc_data = np.array(list(pc_data))
        if (len(list(pc_data)) > 1):
            xys = np.array(pc_data.tolist())[:, :2]  # Nx3
            self.left_point_length, self.left_road_offset, self.left_thetas = self.road_estimation.line_identification(
                xys)  # thetas: 5x(1x3)
        else:
            return

    def listener_right_road(self, msg_pointcloud2: PointCloud2) -> [int, int, list]:
        # def listener_right_road(self, msg_pointcloud2):
        pc_data = pc2.read_points(msg_pointcloud2)
        pc_data = np.array(list(pc_data))
        if (len(list(pc_data)) > 1):
            xys = np.array(pc_data.tolist())[:, :2]  # Nx3
            self.right_point_length, self.right_road_offset, self.right_thetas = self.road_estimation.line_identification(
                xys)  # thetas: 5x(1x3)
        else:
            return

    def listener_ego_velocity(self, twist_msg):
        self.ego_v = [twist_msg.linear.x, twist_msg.angular.z]

    def estimate_ego_position(self):
        self.ego_position, self.egohead = self.pos_estimation.estimate_ego_pos(
            self.ego_v, self.curvatures)  # 3x2, 1x3
        self.seekpos = self.pos_estimation.estimate_direct_seek_y_poss(
            self.ego_position, self.curvature_seeks)  # 3x(2x5)
        self.seekpos_abs = self.pos_estimation.estimate_base_abs_poss(
            self.ego_position, self.seekpos)

    def calculate_object_risk(self):
        self.object_risk = self.object_estimation.risk_3step(
            self.object_position, self.ego_position, self.egohead, self.seekpos)  # list [5x1] x 3

    def calculate_road_risk(self):
        self.left_road_risk, left_y_hat = self.road_estimation.calculation_road_risk(
            self.seekpos_abs, self.left_point_length, self.left_road_offset, self.left_thetas)  # list [5x1] x 3
        self.right_road_risk, right_y_hat = self.road_estimation.calculation_road_risk(
            self.seekpos_abs, self.right_point_length, self.right_road_offset, self.right_thetas)
        self.benefit = self.road_estimation.benefit_path(
            self.seekpos_abs, left_y_hat, right_y_hat)

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

    def calculate_yaw_angle_velocity(self):
        self.curvatures = self.exs_mpc.extremum_seeking_mpc_3step(
            self.total_risk[0], self.total_risk[1], self.total_risk[2])  # 3 points
        _, self.yaw_angle = self.pos_estimation.estimate_pos_curvature(
            self.ego_v[0], self.curvatures[0], self.predict_horizon[0])
        self.yaw_angle_velocity = self.yaw_angle / self.predict_horizon[0]

    def speed_control(self):
        ang_circle = max((abs(self.yaw_angle) - self.speed_circle), 0)
        self.x_vel = max(self.ego_v_target[0] -
                         (ang_circle * self.deceleration_gain), 0.)

    def publish_cmd_vel(self):
        self.vel_msg.linear.x = self.x_vel
        self.vel_msg.angular.z = self.yaw_angle_velocity * self.curvature_gain
        self.pub_cmd.publish(self.vel_msg)

    def tf_viewer(self, ego_position):
        ts = TransformStamped()
        msg_time = self.get_clock().now().to_msg()
        ts.header.stamp = msg_time
        ts.header.frame_id = self.frame_id
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

    def on_timer(self):
        # ---- get object position from tf ----
        self.object_position = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        # ---- start exs_mpc sequence ----
        # estimated egocar position
        self.estimate_ego_position()

        # object risk (fixed position only, not yet dynamic position (ex. kalmanfilter))
        self.calculate_object_risk()

        # road risk
        self.calculate_road_risk()

        # risk seeking
        self.calculate_total_risk()

        # calculate_angula_r_z
        self.calculate_yaw_angle_velocity()

        # for test, spped circle
        self.speed_control()

        # publish cmd_vel
        self.publish_cmd_vel()

        # visualize tf
        self.tf_viewer(self.ego_position)

    def reset_command(self, sig, frame):  # Ctrl + C
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.pub_cmd.publish(self.vel_msg)
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    extremum_seeking_mpc = ExtremumSeekingMpc()
    rclpy.spin(extremum_seeking_mpc)
    extremum_seeking_mpc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
