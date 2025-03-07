from enum import IntEnum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from common_python.get_ros_parameter import get_ros_parameter


class Button(IntEnum):
    CROSS = 0
    SQUARE = 1
    CIRCLE = 2
    TRIANGLE = 3
    PADDLE_SHIFT_R = 4
    PADDLE_SHIFT_L = 5
    R2 = 6
    L2 = 7
    SHARE = 8
    OPTION = 9
    R3 = 10
    L3 = 11
    PLUS = 19
    MINUS = 20
    RED_DIAL_CLOCKWISE = 21
    RED_DIAL_COUNTERCLOCKWISE = 22
    ENTER_ICON = 23
    PLAY_STATION = 24


class Axis(IntEnum):
    STEERING = 0          # Full Left Turn: 1.0, Full Right Turn: -1.0
    CLUTCH = 1            # Natural: 1.0, Full Throttle: -1.0
    ACCEL = 2             # Natural: 1.0, Full Throttle: -1.0
    BRAKE = 3             # Natural: 1.0, Full Throttle: -1.0
    CROSS_HORIZONTAL = 4  # Left: 1.0, Right: -1.0
    CROSS_VERTICAL = 5    # Up  : 1.0, Down : -1.0


class TeleopTwistHandleController(Node):
    def __init__(self):
        super().__init__('teleop_twist_handle_controller_node')
        self.get_ros_params()
        self.was_accel_pressed = False

        # Publisher & Subscriber
        buffer_size = 10
        self.joy_sub = self.create_subscription(Joy, 'sub_joy', self.joy_callback, buffer_size)
        self.twist_pub = self.create_publisher(Twist, 'pub_cmd_vel', buffer_size)
        self.twist_mux_lock_pub = self.create_publisher(Bool, 'pub_twist_mux_lock', buffer_size)

    def get_ros_params(self):
        self.max_linear_vel = get_ros_parameter(self, "max_linear_vel")
        self.max_angular_vel = get_ros_parameter(self, "max_angular_vel")

    def joy_callback(self, joy_msg):
        brake_ratio = (joy_msg.axes[Axis.BRAKE] + 1.0) * 0.5  # raw:-1.0 ~ 1.0 -> ratio: 0 ~ 1.0
        accel_ratio = (joy_msg.axes[Axis.ACCEL] + 1.0) * 0.5
        steering_ratio = joy_msg.axes[Axis.STEERING]

        # Do not publish when neither the accelerator nor the brake is pressed.
        if brake_ratio:
            self.lock_low_priority_speed_commands()
            self.publish_velocity(0.0, 0.0)
        elif accel_ratio:
            self.was_accel_pressed = True
            self.publish_velocity(self.max_linear_vel * accel_ratio, self.max_angular_vel * steering_ratio)
        elif self.was_accel_pressed:
            self.publish_velocity(0.0, 0.0)
            self.was_accel_pressed = False

        if joy_msg.buttons[Button.ENTER_ICON]:
            self.unlock_low_priority_speed_commands()

    def publish_velocity(self, linear_velocity, angular_velocity):
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.twist_pub.publish(twist_msg)
        self.get_logger().debug(f"(v, w): ({twist_msg.linear.x:.2f}, {twist_msg.angular.z:.2f})")

    def lock_low_priority_speed_commands(self):
        lock_msg = Bool()
        lock_msg.data = True
        self.twist_mux_lock_pub.publish(lock_msg)
        self.get_logger().debug("Lock the low-priority speed commands.")

    def unlock_low_priority_speed_commands(self):
        lock_msg = Bool()
        lock_msg.data = False
        self.twist_mux_lock_pub.publish(lock_msg)
        self.get_logger().debug("Unlock the low-priority speed commands.")


def main(args=None):
    rclpy.init(args=args)
    teleop_twist__handle_controller = TeleopTwistHandleController()
    rclpy.spin(teleop_twist__handle_controller)
    teleop_twist__handle_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
