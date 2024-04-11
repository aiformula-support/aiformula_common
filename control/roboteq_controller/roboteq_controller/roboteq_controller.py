#!/usr/bin/env python
# coding: utf-8
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64, UInt8, Int32
from can_msgs.msg import Frame
from decimal import Decimal, ROUND_HALF_DOWN


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

    def cmd_callback(self, msg):
        self.vel = msg
        self.cmd_linear_x = self.vel.linear.x
        self.cmd_angular_z = self.vel.angular.z

        r_ref = (1 / (self.diameter / 2)) * self.cmd_linear_x + (
            self.tread / self.diameter
        ) * self.cmd_angular_z  # [rad/s]
        r_ref = r_ref * 30 / math.pi  # [rpm]  タイヤ1回転　30rpm
        r_ref = r_ref * self.gear_ratio  # [rpm]

        l_ref = (1 / (self.diameter / 2)) * self.cmd_linear_x - (
            self.tread / self.diameter
        ) * self.cmd_angular_z  # [rad/s]
        l_ref = l_ref * 30 / math.pi  # [rpm]
        l_ref = l_ref * self.gear_ratio  # [rpm]
        # convert cg command
        r_ref = Decimal(str(r_ref)).quantize(
            Decimal("0"), rounding=ROUND_HALF_DOWN)
        l_ref = Decimal(str(l_ref)).quantize(
            Decimal("0"), rounding=ROUND_HALF_DOWN)
        r_byte = (int(r_ref)).to_bytes(
            4, "big", signed=True)  # high byteから順番に並んでいる
        l_byte = (int(l_ref)).to_bytes(4, "big", signed=True)
        r_cmd = [int(r_byte[0]), int(r_byte[1]),
                 int(r_byte[2]), int(r_byte[3])]
        l_cmd = [int(l_byte[0]), int(l_byte[1]),
                 int(l_byte[2]), int(l_byte[3])]
#        self.cmd.header(
#            header=0x210, can_data=[l_cmd[3], l_cmd[2], l_cmd[1], l_cmd[0], r_cmd[3], r_cmd[2], r_cmd[1], r_cmd[0]]
#        )
        self.cmd.header.frame_id = "can0"
        self.cmd.id = 0x210
        self.cmd.dlc = 8
        self.cmd.data = [l_cmd[3], l_cmd[2], l_cmd[1],
                         l_cmd[0], r_cmd[3], r_cmd[2], r_cmd[1], r_cmd[0]]
        self.publisher_.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
