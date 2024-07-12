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
    potentio_val = 0.0
    #---Voltage to Angle---#
    # msg.data[1]:0  ,msg.data[0]:0   ~ msg.data[1]:1,  msg.data[0]:90  -> CCW 0 ~ 30 deg
    # msg.data[1]:255,msg.data[0]:255 ~ msg.data[1]:254,msg.data[0]:200 -> CW 0 ~ 30 deg

    val_to_deg_ccw = 0.087 # CCW 30 / 345(電圧幅) = 0.087
    val_to_deg_cw = 0.097 # CW 30 / 310(電圧幅) = 0.097

    ccw_pre = 22.17 # msg.data[1]:0,msg.data[1]:0 ~ 255 -> CCW 0 ~ 22.17 deg
    cw_pre = 24.68 # msg.data[1]:255,msg.data[1]:255 ~ 0 -> CW 0 ~ 24.68 deg

    if msg.id == 0x11: # CANID 0x11 potentiometer
      if msg.data[1] == 0: # CCW 0 ~ 22.17 deg
        potentio_val = -(msg.data[0] * val_to_deg_ccw)
      elif msg.data[1] == 1: # CCW 22.17 ~ 30 deg
        potentio_val = -(msg.data[0] * val_to_deg_ccw + ccw_pre)
      elif msg.data[1] == 255: # CW 0 ~ 24.68 deg
        potentio_val = (255 - msg.data[0]) * val_to_deg_cw
      elif msg.data[1] == 254: # CW 24.68 ~ 30 deg
        potentio_val = ((255 - msg.data[0]) * val_to_deg_cw) + cw_pre

      potentio_val = math.radians(potentio_val) # deg to rad

      print("rear_angle: " + str(potentio_val) + " rad")
      angle_msg = Float32()
      angle_msg.data = potentio_val
      self.angle_pub.publish(angle_msg)

def main(args=None):
  rclpy.init(args=args)
  rear_potentiometer = RearPotentiometer()
  rclpy.spin(rear_potentiometer)
  rear_potentiometer.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()