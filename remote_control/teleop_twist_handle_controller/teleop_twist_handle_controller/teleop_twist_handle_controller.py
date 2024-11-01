from enum import IntEnum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from common_python.get_ros_parameter import get_ros_parameter
from common_python.util import to_timestamp_double


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
        # See “Acceleration Model” in `apply_acceleration()` below.
        self.drag_coefficient = self.max_linear_acceleration / self.max_linear_vel ** self.drag_exponent
        self.was_accel_pressed = False
        self.prev_time = 0.0
        self.twist_msg = Twist()
        self.twist_msg.linear.x = self.twist_msg.linear.z = 0.0
        self.print_params()

        # Publisher & Subscriber
        buffer_size = 10
        self.joy_sub = self.create_subscription(Joy, 'sub_joy', self.joy_callback, buffer_size)
        self.twist_pub = self.create_publisher(Twist, 'pub_cmd_vel', buffer_size)
        self.coasting_twist_pub = self.create_publisher(Twist, 'pub_cmd_vel_coasting', buffer_size)
        self.twist_mux_lock_pub = self.create_publisher(Bool, 'pub_twist_mux_lock', buffer_size)

    def get_ros_params(self):
        self.max_linear_vel = get_ros_parameter(self, "max_linear_vel")
        self.max_angular_vel = get_ros_parameter(self, "max_angular_vel")
        self.max_linear_acceleration = get_ros_parameter(self, "max_linear_acceleration")
        self.drag_exponent = get_ros_parameter(self, "drag_exponent")
        self.stopping_vel = get_ros_parameter(self, "stopping_vel")

    def print_params(self):
        self.get_logger().debug("============================")
        self.get_logger().debug("(teleop_twist_handle_controller.yaml)")
        self.get_logger().debug(f"  max_linear_vel          : {self.max_linear_vel:.2f} [m/s]")
        self.get_logger().debug(f"  max_angular_vel         : {self.max_angular_vel:.2f} [rad/s]")
        self.get_logger().debug(f"  max_linear_acceleration : {self.max_linear_acceleration:.2f} [m/s^2]")
        self.get_logger().debug(f"  drag_exponent           : {self.drag_exponent:.2f}")
        self.get_logger().debug(f"  stopping_vel            : {self.stopping_vel:.2f} [m/s]")

        self.get_logger().debug("\n(initialize)")
        self.get_logger().debug(f"  drag_coefficient        : {self.drag_coefficient:.2f} [m/s]")
        self.get_logger().debug("============================\n")

    def joy_callback(self, joy_msg):
        brake_ratio = (joy_msg.axes[Axis.BRAKE] + 1.0) * 0.5  # raw:-1.0 ~ 1.0 -> ratio: 0 ~ 1.0
        accel_ratio = (joy_msg.axes[Axis.ACCEL] + 1.0) * 0.5
        steering_ratio = joy_msg.axes[Axis.STEERING]
        current_time = to_timestamp_double(joy_msg.header.stamp)

        if joy_msg.buttons[Button.ENTER_ICON]:
            self.unlock_low_priority_speed_commands()

        if self.prev_time:
            dt = current_time - self.prev_time
            if brake_ratio:
                self.lock_low_priority_speed_commands()
                self.twist_msg.linear.x = self.twist_msg.linear.z = 0.0
            else:
                self.apply_acceleration(accel_ratio, dt)
            self.twist_msg.angular.z = self.max_angular_vel * steering_ratio
            if accel_ratio or brake_ratio:
                self.twist_pub.publish(self.twist_msg)
            else:
                self.coasting_twist_pub.publish(self.twist_msg)

        self.prev_time = current_time

    def apply_acceleration(self, accel_ratio, dt):
        """ Acceleration Model
        V2 = V1 + a * dt - b * V1^n * dt
        t -> infinite
        Vt = Vt + a * dt - b * Vt^n * dt
        b = a / Vt^n
        ------------------------------
        V1 : velocity at time `t`
        V2 : velocity at time `t + dt`
        Vt : terminal velocity
        a  : max acceleration
        b  : drag coefficient
        n  : drag exponent
        """
        linear_acceleration = self.max_linear_acceleration * accel_ratio - \
            self.drag_coefficient * self.twist_msg.linear.x ** self.drag_exponent
        accelerated_linear_velocity = self.twist_msg.linear.x + linear_acceleration * dt
        self.twist_msg.linear.x = max(0.0, min(accelerated_linear_velocity, self.max_linear_vel))
        if self.twist_msg.linear.x < self.stopping_vel:
            self.twist_msg.linear.x = 0.0

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
