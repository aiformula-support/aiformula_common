import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoyTransFormer(Node): 
    def __init__(self):
        super().__init__('handle_controller') 
        self.publisher = self.create_publisher(Twist,'cmd_vel', 10)
        self.subscription = self.create_subscription(Joy,'joy', self.listener_callback, 10)
        self.vel = Twist()

    def listener_callback(self, Joy): 
        accel_raw = Joy.axes[2]
        accel = accel_raw + 1.0
        if accel > 0.1:
            self.vel.linear.x  = 2*accel
            
        brake_raw = Joy.axes[3]
        brake = brake_raw + 1.0

        if brake > 0.1:
            self.vel.linear.x  = 0.0

        back_raw = Joy.axes[1]
        back = back_raw + 1.0
        if back > 0.1:
            self.vel.linear.x = -1.5*back

        stearing = Joy.axes[0]
        self.vel.angular.z  = -4*stearing

        self.publisher.publish(self.vel)
        self.get_logger().info("Velocity: Linear=%f" % (self.vel.linear.x)) 
        self.get_logger().info("Velocity: Angular=%f" % (self.vel.angular.z)) 

def main(args=None):
    rclpy.init(args=args)
    joy_transformer = JoyTransFormer()
    rclpy.spin(joy_transformer)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
