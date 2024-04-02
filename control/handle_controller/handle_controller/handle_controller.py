import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class JoyTransFormer(Node):
    def __init__(self):
        super().__init__('handle_controller')
        self.subscription = self.create_subscription(Joy, 'sub_joy_frame', self.listener_callback, 10)
        self.publisher = self.create_publisher(Twist, 'pub_handle_controller', 10)
        self.vel = Twist()
        self.declare_parameter('max_vel')
        self.declare_parameter('max_angular')
        self.max_vel = self.get_parameter('max_vel').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular').get_parameter_value().double_value

    def listener_callback(self, Joy):
        accel_raw = Joy.axes[2]
        accel = accel_raw + 1.0         # joy setting begin -1.0 range -1.0 ~ 1.0
        stearing = Joy.axes[0]
        brake_raw = Joy.axes[3]
        brake = brake_raw + 1.0

        if accel > 0.1:
            self.vel.linear.x = self.max_vel*accel
            self.vel.angular.z = self.max_angular*stearing

        elif brake > 0.1:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0

        else:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0

        self.publisher.publish(self.vel)
        print(accel)
        self.get_logger().info("Velocity: Linear=%f" % (self.vel.linear.x))
        self.get_logger().info("Velocity: Angular=%f" % (self.vel.angular.z))


def main(args=None):
    rclpy.init(args=args)
    joy_transformer = JoyTransFormer()
    rclpy.spin(joy_transformer)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
