#!/usr/bin/env python
# coding: utf-8
import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from can_msgs.msg import Frame
from typing import List


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.subscriber = self.create_subscription(
            Twist, 'sub_speed_command', self.callback, 10)
        self.publisher_ = self.create_publisher(Frame, 'pub_can', 10)
        self.vel = Twist()
        self.cmd = Frame()
        self.diameter = 0.248
        self.tread = 0.6
        self.gear_ratio = 1
        timer = 1/100
        self.timer = self.create_timer(timer, self.callback)
        self.declare_parameter('tread')
        self.declare_parameter('diameter')
        self.declare_parameter('gear_ratio')
        self.tread = self.get_parameter('tread').get_parameter_value().double_value
        self.diameter = self.get_parameter('diameter').get_parameter_value().double_value
        self.gear_ratio = self.get_parameter('gear_ratio').get_parameter_value().double_value

    def cmd_callback(self, msg):
        self.vel = msg
        ref = toRefRPM(self.vel.linear.x, self.vel.angular.z)
        l_cmd = toCanCmd(ref[0])
        r_cmd = toCanCmd(ref[1])
        can_data = l_cmd + r_cmd

        self.cmd.header.frame_id = "can0"        # Default can0
        self.cmd.id = 0x210                      # MotorControler CAN ID : 0x210
        self.cmd.dlc = 8                         # Data length
        self.cmd.data = can_data

    def callback(self):
        self.publisher_.publish(self.cmd)


def toRefRPM(self, linear, angular):  # Calc Motor ref rpm
    ref = np.array([(1 / (self.diameter * 0.5)) * linear + (self.tread / self.diameter) * angular,  # right[rad/s]
                   (1 / (self.diameter * 0.5)) * linear - (self.tread / self.diameter) * angular])  # left[raad/s]
    ref *= 30 / math.pi               # [rpm]  タイヤ1回転　30
    ref *= self.gear_ratio            # [rpm]
    return ref.tolist()


def toCanCmd(rpm: float) -> List[int]:
    rounded = round(rpm)
    bytes = rounded.to_bytes(4, "little", signed=True)
    return list(bytes)


def main(args=None):
    rclpy.init(args=args)
    roboteq_controller = RoboteqController()
    rclpy.spin(roboteq_controller)
    roboteq_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
