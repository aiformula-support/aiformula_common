import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from common_python.get_ros_parameter import get_ros_parameter


class JoyTransFormer(Node):
    def __init__(self):
        super().__init__('teleop_twist_handle_controller')
        buffer_size = 10
        self.joy_sub = self.create_subscription(Joy, 'sub_joy_frame', self.joy_callback, buffer_size)
        self.twist_pub = self.create_publisher(Twist, 'pub_teleop_twist_handle_controller', buffer_size)
        self.vel = Twist()
        self.max_vel = get_ros_parameter(self, "max_vel")
        self.max_angular = get_ros_parameter(self, "max_angular")
        self.determine_pressed = get_ros_parameter(self, "determine_pressed")

    def joy_callback(self, Joy):
        brake_ratio = (Joy.axes[3] + 1.0) * 0.5   # raw:-1.0 ~ 1.0 -> ratio: 0 ~ 1.0
        accel_ratio = (Joy.axes[2] + 1.0) * 0.5
        stearing_ratio = Joy.axes[0]              # raw:-1.0 ~ 1.0

        if brake_ratio < self.determine_pressed and accel_ratio > self.determine_pressed:
            self.vel.linear.x = self.max_vel * accel_ratio
            self.vel.angular.z = self.max_angular * stearing_ratio

        else:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0

        self.twist_pub.publish(self.vel)
        self.get_logger().info("Velocity: Linear=%f" % (self.vel.linear.x))
        self.get_logger().info("Velocity: Angular=%f" % (self.vel.angular.z))


def main(args=None):
    rclpy.init(args=args)
    joy_transformer = JoyTransFormer()
    rclpy.spin(joy_transformer)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
