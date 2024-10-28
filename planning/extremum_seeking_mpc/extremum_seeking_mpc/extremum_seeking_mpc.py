import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .script.exs_mpc_script import *
from .script.exs_road_script import *
from .script.exs_geometric_position import *
from .script.exs_objrisk_script import *

import signal
import sys

from std_msgs.msg import Float64,Float64MultiArray

class ExsMpc(Node):

    def __init__(self):
        super().__init__('extremum_seeking_mpc')
        # Initialize ROS parameters
        self.init_parameters()
        
        # initialize publishers
        buffer_size = 10
        self.pub_cmd = self.create_publisher(Twist, 'pub_speed_command', buffer_size)
        self.sub_road_l = self.create_subscription(PointCloud2, 'sub_road_l', self.listener_road_l, buffer_size)
        self.sub_road_r = self.create_subscription(PointCloud2, 'sub_road_r', self.listener_road_r, buffer_size)
        
        # initialize tf
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

	    # cmd_vel Message
        self.vel_msg = Twist()

        # instance of scripts
        object_risk_table_u = self.get_parameter('object_risk_potential.pedestrian_risk_table_u').get_parameter_value().double_array_value # u is distance from object
        object_risk_table_y = self.get_parameter('object_risk_potential.pedestrian_risk_table_y').get_parameter_value().double_array_value # y is object risk
        self.object_estimation = ObjectEstimation(object_risk_table_u, object_risk_table_y)
        road_sigma = self.get_parameter('road_risk_potential.road_risk_table_u').get_parameter_value().double_array_value
        road_risk_table_l = self.get_parameter('road_risk_potential.road_l_risk_table_y').get_parameter_value().double_array_value
        road_risk_table_r = self.get_parameter('road_risk_potential.road_r_risk_table_y').get_parameter_value().double_array_value
        self.road_estimation = RoadEstimation(road_sigma,road_risk_table_l,road_risk_table_r)

        # EXS_MPC parameter
        self.ego_v = self.get_parameter('mpc_parameters.planned_speed').get_parameter_value().double_array_value
        self.predict_horizon = self.get_parameter('mpc_parameters.holizon_times').get_parameter_value().double_array_value
        self.get_logger().info(f'ego_v : {self.ego_v}')
        seek_gain = self.get_parameter('mpc_parameters.seek_gain').get_parameter_value().double_value
        seek_amp = self.get_parameter('mpc_parameters.seek_amp').get_parameter_value().double_value
        curvature_max = self.get_parameter('mpc_parameters.esc_max').get_parameter_value().double_value
        curvature_min = self.get_parameter('mpc_parameters.esc_min').get_parameter_value().double_value
        self.curvature_gain = self.get_parameter('mpc_parameters.curvature_gain').get_parameter_value().integer_value
        self.curvatures = [0] * len(self.predict_horizon)
        self.pos_estimation = geometric_pos_curvatures(self.predict_horizon[0], self.predict_horizon[1], self.predict_horizon[2])
        self.exs_mpc = exs_mpc(seek_gain,seek_amp,curvature_max,curvature_min)
        self.curvature_seeks = [self.exs_mpc.extremum_seeking_step1.seek_points,
                         self.exs_mpc.extremum_seeking_step2.seek_points,
                         self.exs_mpc.extremum_seeking_step3.seek_points] # csa type: sin is dibided by fixed value of 8

        # for risk calculation
        self.object_risk = []
        self.road_l_risk = []
        self.road_r_risk = []
        self.norm_den_l = 0.
        self.norm_ofs_l = 0.
        self.thetas_l = np.full((5,3), 0.)
        self.norm_den_r = 0.
        self.norm_ofs_r = 0.
        self.thetas_r = np.full((5,3), 0.)
        self.benefit = []
        self.benefit_gain = self.get_parameter('mpc_parameters.benefit_gain').get_parameter_value().integer_value

        self.timer = self.create_timer(0.01, self.on_timer)
        
        signal.signal(signal.SIGINT,self.signal_handler)
        
    def init_parameters(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ("mpc_parameters.holizon_times", [1.0, 2.0, 3.0]),
                ("mpc_parameters.planned_speed", [0.1, 0.0]),
                ("mpc_parameters.seek_gain", -0.2),
                ("mpc_parameters.seek_amp", 0.07),
                ("mpc_parameters.esc_max", 0.2),
                ("mpc_parameters.esc_min", -0.2),
                ("mpc_parameters.curvature_gain", 15),
                ("mpc_parameters.benefit_gain", 2),
                ("object_risk_potential.pedestrian_risk_table_u", [2., 1., 0.6, 0.5, 0.35, 0.2, 0., -0.2, -0.35, -0.5, -0.6, -1., -2.]),
                ("object_risk_potential.pedestrian_risk_table_y", [0., 0., 0., 0.1, 0.5, 0.9, 1., 0.9, 0.5, 0.1, 0., 0., 0.]),
                ("road_risk_potential.road_risk_table_u", [-2.4, -2.0, -1.6, -1.2, -0.8, -0.4, 0., 0.4, 0.8, 1.2, 1.6, 2.0, 2.4]),
                ("road_risk_potential.road_l_risk_table_y", [0.,0.,0.,0.,0.,0.,0., 0., 0.2, 0.4, 0.6, 0.8, 1.0]),
                ("road_risk_potential.road_r_risk_table_y", [1.0, 0.8, 0.6, 0.4, 0.2, 0., 0.,0.,0.,0.,0.,0.,0.])
            ]
        )
        
    def listener_road_l(self, msg_pc2):
        pc_data = pc2.read_points(msg_pc2)
        if (len(pc_data) > 1):
            xys = np.array(pc_data.tolist())[:, :2] #Nx3
            self.norm_den_l, self.norm_ofs_l, self.thetas_l = self.road_estimation.line_id_quintiple(xys) # thetas: 5x(1x3)

    def listener_road_r(self, msg_pc2):
        pc_data = pc2.read_points(msg_pc2)
        if (len(pc_data) > 1):
            xys = np.array(pc_data.tolist())[:, :2] #Nx3
            self.norm_den_r, self.norm_ofs_r, self.thetas_r = self.road_estimation.line_id_quintiple(xys) # thetas: 5x(1x3)

    def signal_handler(self,sig,frame): # Ctrl + C
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.pub_cmd.publish(self.vel_msg)
        sys.exit(0)

    def on_timer(self):
        #print("********** 1 cycle **********")
        #---- get object position from tf ----
        object_position = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
        #try:
        #    t = self.tf_buffer.lookup_transform(
        #        'camera_depth_optical_frame',
        #        'object1',
        #        rclpy.time.Time(),
        #    )
        #except TransformException as ex:
        #    self.get_logger().info(
        #        f'Could not transform base_link to object1: {ex}')
        #    return
        #object_position.append([t.transform.translation.x, t.transform.translation.y])
        #try:
        #    t = self.tf_buffer.lookup_transform(
        #        'camera_depth_optical_frame',
        #        'object2',
        #        rclpy.time.Time(),
        #    )
        #except TransformException as ex:
        #    self.get_logger().info(
        #        f'Could not transform base_link to object2: {ex}')
        #    return
        #object_position.append([t.transform.translation.x, t.transform.translation.y])

        #---- start exs_mpc sequence ----
        # extimated egocar position
        egopos, egohead = self.pos_estimation.estimate_ego_pos(self.ego_v, self.curvatures) # 3x2, 1x3
        seekpos = self.pos_estimation.estimate_direct_seek_y_poss(egopos, self.curvature_seeks) # 3x(2x5)
        seekpos_abs = self.pos_estimation.estimate_base_abs_poss(egopos, seekpos)
        
        # object risk (fixed position only, not yet dynamic position (ex. kalmanfilter))
        self.object_risk = self.object_estimation.risk3step(object_position, egopos, egohead, seekpos) # list [5x1] x 3

        # road risk
        self.road_l_risk,y_hat_l = self.road_estimation.CalculationRoadRisk(seekpos_abs, self.norm_den_l, self.norm_ofs_l, self.thetas_l) # list [5x1] x 3
        self.road_r_risk,y_hat_r = self.road_estimation.CalculationRoadRisk(seekpos_abs, self.norm_den_r, self.norm_ofs_r, self.thetas_r)
        self.benefit = self.road_estimation.Benefit_path(seekpos_abs, y_hat_l, y_hat_r)

        ### risk seeking
        self.total_risk  = []
        risk_tmp = (np.array(self.object_risk[0]) + np.array(self.road_l_risk[0]) + np.array(self.road_r_risk[0]) - self.benefit_gain * np.array(self.benefit[0])).tolist()
        self.total_risk.append(risk_tmp)       
        risk_tmp = (np.array(self.object_risk[1]) + np.array(self.road_l_risk[1]) + np.array(self.road_r_risk[1]) - self.benefit_gain * np.array(self.benefit[1])).tolist()      
        self.total_risk.append(risk_tmp)
        risk_tmp = (np.array(self.object_risk[2]) + np.array(self.road_l_risk[2]) + np.array(self.road_r_risk[2]) - self.benefit_gain * np.array(self.benefit[2])).tolist()
        self.total_risk.append(risk_tmp) # list [5x1] x 3

        self.curvatures = self.exs_mpc.extremum_seeking_mpc_3step(self.total_risk[0], self.total_risk[1], self.total_risk[2]) # 3 points
        _, yaw_angle = self.pos_estimation.estimate_pos_curvature(self.ego_v[0], self.curvatures[0], self.predict_horizon[0])
        yaw_angle_velocity = yaw_angle / self.predict_horizon[0]
        
        # for test, spped circle
        ang_circle = max((abs(yaw_angle) - 0.16), 0)
        x_vel = max(self.ego_v[0] - (ang_circle * 5), 0.)

        msg_time = self.get_clock().now().to_msg()
                
        self.vel_msg.linear.x = x_vel
        self.vel_msg.angular.z = yaw_angle_velocity * self.curvature_gain
        self.pub_cmd.publish(self.vel_msg)

        ts = TransformStamped()
        ts.header.stamp = msg_time
        ts.header.frame_id = "camera_depth_optical_frame"
        ts.child_frame_id = "seek_point1"
        ts.transform.translation.x = egopos[0][0]
        ts.transform.translation.y = egopos[0][1]
        ts.transform.translation.z = 0.
        ts.transform.rotation.x = 0.
        ts.transform.rotation.y = 0.
        ts.transform.rotation.z = 0.
        ts.transform.rotation.w = 1.
        self.tf_broadcaster.sendTransform(ts)
        ts.header.frame_id = "seek_point1"
        ts.child_frame_id = "seek_point2"
        ts.transform.translation.x = egopos[1][0]
        ts.transform.translation.y = egopos[1][1]
        self.tf_broadcaster.sendTransform(ts)
        ts.header.frame_id = "seek_point2"
        ts.child_frame_id = "seek_point3"
        ts.transform.translation.x = egopos[2][0]
        ts.transform.translation.y = egopos[2][1]
        self.tf_broadcaster.sendTransform(ts)

def main():
    rclpy.init()
    node = ExsMpc()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
