#!/usr/bin/env python
import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from can_msgs.msg import Frame
from typing import List


class MotorController(Node):

    def __init__(self):
        super().__init__('motor_controller')
        buffer_size = 10
        self.twist_sub = self.create_subscription(Twist, 'sub_speed_command', self.twist_callback, buffer_size)
        self.can_pub = self.create_publisher(Frame, 'pub_can', buffer_size)
        self.cmd = Frame()
        self.declare_parameter('motor.tread')
        self.declare_parameter('motor.diameter')
        self.declare_parameter('motor.gear_ratio')
        self.declare_parameter('motor.publish_timer_loop_duration')
        self.tread = self.get_parameter('motor.tread').get_parameter_value().double_value
        self.diameter = self.get_parameter('motor.diameter').get_parameter_value().double_value
        self.gear_ratio = self.get_parameter('motor.gear_ratio').get_parameter_value().double_value
        self.publish_timer_loop_duration = self.get_parameter(
            'motor.publish_timer_loop_duration').get_parameter_value().double_value
        self.publish_timer = self.create_timer(self.publish_timer_loop_duration, self.canframe_callback)

    def twist_callback(self, msg):
        self.vel = msg
        ref = self.toRefRPM(self.vel.linear.x, self.vel.angular.z)
        l_cmd = self.toCanCmd(ref[0])
        r_cmd = self.toCanCmd(ref[1])
        can_data = l_cmd + r_cmd
        self.cmd.header.frame_id = "can0"        # Default can0
        self.cmd.id = 0x210                      # MotorControler CAN ID : 0x210
        self.cmd.dlc = 8                         # Data length
        self.cmd.data = can_data

    def canframe_callback(self):
        self.can_pub.publish(self.cmd)

#  Velocity -> RPM Calc
#  V_right = (V + tread/2 * w), V_left = (V - tread / 2 * w ) [m/s]
#  w_right = V/r + (d/2r) * w   [rad/s]
#  w_left = V/r - (d/2r) * w    [rad/s]
#  rpm = w * 60 / 2* pi [rpm]
#  rpm = rpm * gear_ratio  [rpm]

    def toRefRPM(self, linear_velocity, angular_velocity):  # Calc Motor ref rad/s
        wheel_angular_velocities = np.array([(linear_velocity / (self.diameter * 0.5)) + (self.tread / self.diameter) * angular_velocity,  # right[rad/s]
                                             (linear_velocity / (self.diameter * 0.5)) - (self.tread / self.diameter) * angular_velocity])  # left[rad/s]
        min = 60
        rpm = wheel_angular_velocities * (min / (2 * math.pi))
        print(rpm)
        return (rpm * self.gear_ratio).tolist()

    @staticmethod
    def toCanCmd(rpm: float) -> List[int]:
        rounded = round(rpm)
        bytes = rounded.to_bytes(4, "little", signed=True)
        return list(bytes)


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
