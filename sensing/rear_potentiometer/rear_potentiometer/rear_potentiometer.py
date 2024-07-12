import math

import rclpy
from rclpy.node import Node

from can_msgs.msg import Frame
from std_msgs.msg import Float32

class RearPotentiometer(Node):
  def __init__(self):
    super().__init__('rear_potentiometer')
    buffer_size = 10
    self.can_sub = self.create_subscription(Frame,'aiformula_sensing/vehicle_info',self.callback,buffer_size)
    self.angle_pub = self.create_publisher(Float32,'aiformula_sensing/rear_potentiometer/yaw',buffer_size)
  
  def callback(self,msg):
    angle_p = 0.0
    # Voltage Extraction
    if msg.id == 17:
      if msg.data[1] == 0: # CCW 0 ~ 22.17 deg
        angle_p = -(msg.data[0] * 0.087)
      elif msg.data[1] == 1: # CCW 22.17 ~ 30 deg
        angle_p = -(msg.data[0] * 0.087 + 22.17)
      elif msg.data[1] == 255: # CW 0 ~ 24.68 deg
        angle_p = (255 - msg.data[0]) * 0.0968
      elif msg.data[1] == 254: # CW 24.68 ~ 30 deg
        angle_p = ((255 - msg.data[0]) * 0.0968) + 24.68

      angle_p = math.radians(angle_p) # deg to rad
      print("rear_angle: " + str(angle_p) + " rad")
      angle_msg = Float32()
      angle_msg.data = angle_p
      self.angle_pub.publish(angle_msg)

def main(args=None):
  rclpy.init(args=args)
  rear_potentiometer = RearPotentiometer()
  rclpy.spin(rear_potentiometer)
  rear_potentiometer.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
