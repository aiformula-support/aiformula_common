import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoyTransFormer(Node):
    def __init__(self):
        super().__init__('teleop_twist_handle_controller')
        buffer_size = 10
        self.joy_sub = self.create_subscription(Joy, 'sub_joy_frame', self.joy_callback, buffer_size)
        self.twist_pub = self.create_publisher(Twist, 'pub_teleop_twist_handle_controller', buffer_size)
        self.vel = Twist()
        self.declare_parameter('max_vel')
        self.declare_parameter('max_angular')
        self.declare_parameter('determine_pressed')
        self.max_vel = self.get_parameter('max_vel').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular').get_parameter_value().double_value
        self.determine_pressed = self.get_parameter('determine_pressed').get_parameter_value().double_value

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
