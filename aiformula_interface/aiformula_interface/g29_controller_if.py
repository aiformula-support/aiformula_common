import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyTranslate(Node): 
    def __init__(self):
        super().__init__('g29_controller_node') 
        self.publisher = self.create_publisher(Twist,'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy,'joy', self.listener_callback, 10)
        self.vel = Twist()

    def listener_callback(self, Joy): 
        accel_raw = Joy.axes[2]
        accel = accel_raw + 1.0
        if accel > 0.1:
            self.vel.linear.x  = 2*accel

        back_raw = Joy.axes[3]
        back = back_raw + 1.0
        if back > 0.1:
            self.vel.linear.x = -1.5*back

        stearing = Joy.axes[0]
        self.vel.angular.z  = -4*stearing
        self.publisher.publish(self.vel)
        self.get_logger().info("Velocity: Linear=%f" % (Joy.axes[2])) 
        self.get_logger().info("Velocity: Angular=%f" % (Joy.axes[0])) 

def main(args=None):
    rclpy.init(args=args)
    joy_translate = JoyTranslate()
    rclpy.spin(joy_translate)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
