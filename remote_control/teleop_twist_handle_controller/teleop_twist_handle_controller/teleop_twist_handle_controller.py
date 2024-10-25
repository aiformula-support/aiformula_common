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


class AXIS(IntEnum):
    STEALING = 0          # Full Left Turn: 1.0, Full Right Turn: -1.0
    CLUTCH = 1            # Natural: 1.0, Full Throttle: -1.0
    ACCEL = 2             # Natural: 1.0, Full Throttle: -1.0
    BRAKE = 3             # Natural: 1.0, Full Throttle: -1.0
    CROSS_HORIZONTAL = 4  # Left: 1.0, Right: -1.0
    CROSS_VERTICAL = 5    # Up  : 1.0, Down : -1.0


class TeleopTwistHandleController(Node):
    def __init__(self):
        super().__init__('teleop_twist_handle_controller_node')
        self.get_parameter()
        self.vel = Twist()
        self.is_pedal_pressed = False

        # Publisher & Subscriber
        buffer_size = 10
        self.joy_sub = self.create_subscription(Joy, 'sub_joy', self.joy_callback, buffer_size)
        self.twist_pub = self.create_publisher(Twist, 'pub_cmd_vel', buffer_size)
        self.twist_mux_lock_pub = self.create_publisher(Bool, 'pub_twist_mux_lock', buffer_size)

    def get_parameter(self):
        self.max_vel = get_ros_parameter(self, "max_vel")
        self.max_angular = get_ros_parameter(self, "max_angular")
        self.determine_pressed = get_ros_parameter(self, "determine_pressed")

    def joy_callback(self, joy_msg):
        brake_ratio = (joy_msg.axes[AXIS.BRAKE] + 1.0) * 0.5  # raw:-1.0 ~ 1.0 -> ratio: 0 ~ 1.0
        accel_ratio = (joy_msg.axes[AXIS.ACCEL] + 1.0) * 0.5
        stearing_ratio = joy_msg.axes[AXIS.STEALING]

        if brake_ratio >= self.determine_pressed or accel_ratio >= self.determine_pressed:
            self.is_pedal_pressed = True
            if brake_ratio >= self.determine_pressed:
                self.publish_lock()
                self.set_velocity(0.0, 0.0)
            else:
                self.set_velocity(self.max_vel * accel_ratio, self.max_angular * stearing_ratio)
        elif self.is_pedal_pressed:
            self.set_velocity(0.0, 0.0)
            self.is_pedal_pressed = False

        if self.is_pedal_pressed:  # Do not publish when neither the accelerator nor the brake is pressed.
            self.twist_pub.publish(self.vel)
            self.get_logger().debug("(v, w): (%.2f, %.2f)" % (self.vel.linear.x, self.vel.angular.z))

        if joy_msg.buttons[Button.ENTER_ICON]:
            self.publish_release()

    def set_velocity(self, linear_velocity, angular_velocity):
        self.vel.linear.x = linear_velocity
        self.vel.angular.z = angular_velocity

    def publish_lock(self):
        lock_msg = Bool()
        lock_msg.data = True
        self.twist_mux_lock_pub.publish(lock_msg)

    def publish_release(self):
        release_lock_msg = Bool()
        release_lock_msg.data = False
        self.twist_mux_lock_pub.publish(release_lock_msg)
        self.get_logger().debug("Release Lock")


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TeleopTwistHandleController())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
