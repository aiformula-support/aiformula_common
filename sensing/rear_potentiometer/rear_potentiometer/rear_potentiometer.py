import math

import rclpy
from rclpy.node import Node

from can_msgs.msg import Frame
from std_msgs.msg import Float32

def callback(msg):
 angle_p = 0.0
 # Voltage Extraction
 if(msg.id == 17):
  print("data[0]: ")
  print(msg.data[0])
  print("data[1]: ")
  print(msg.data[1])
  if msg.data[1] == 0: # CCW 0 ~ 22.17 deg
    angle_p = msg.data[0] * 0.087
  elif msg.data[1] == 1: # CCW 22.17 ~ 30 deg
    angle_p = msg.data[0] * 0.087 + 22.17
  elif msg.data[1] == 255: # CW 0 ~ 24.68 deg
    angle_p = -((255 - msg.data[0]) * 0.0968)
  elif msg.data[1] == 254: # CW 24.68 ~ 30 deg
    angle_p = -(((255 - msg.data[0]) * 0.0968) + 24.68)

  print("angle: ")
  print(angle_p)
  ang = Float32()
  ang.data = angle_p
  publisher.publish(ang)

def main(args=None):
 # Initialize
 rclpy.init(args=args)
 
 # Node Define
 node = rclpy.create_node('angle_send_node')
 
 # Subscriber Define
 subscriber = node.create_subscription(Frame,'aiformula_sensing/vehicle_info',callback,10)
 
 global publisher
 # Publisher Define
 publisher = node.create_publisher(Float32,'aiformula_sensing/rear_potentiometer/yaw',10)
 
 # ROSSpin
 rclpy.spin(node)
 
 # Node Destory
 node.destroy_node()
 rclpy.shutdown

if __name__ == '__main__':
 main()
