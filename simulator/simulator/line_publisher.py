import rclpy
from rclpy.node import Node

import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

import numpy as np
import csv

import sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix

package = str(Path(__file__).resolve().parent.name)
workspace = Path(get_package_prefix(package)).parents[1]
ROOT = workspace / 'src/EC7D_AIformula_Control/simulator'
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))  # add ROOT to PATH

class LinePublisher(Node):
    def __init__(self):
        super().__init__('bev_camera')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.callback_odom,10)
        self.right_points_pub = self.create_publisher(PointCloud2, 'line_points/right', 10)
        self.center_points_pub = self.create_publisher(PointCloud2, 'line_points/center', 10)
        self.left_points_pub = self.create_publisher(PointCloud2, 'line_points/left', 10)

        timer = 1/10
        self.timer_image = self.create_timer(timer, self.callback_points)

        self.odom = Odometry()
        self.center_points = []
        self.left_points = []
        self.right_points = []

        with open(ROOT/'worlds/shihou_course/shihou_lane_points.csv') as f:
            reader = csv.reader(f)
            points = [row for row in reader]
            del points[0]
            points = np.array(points, dtype='float32')

        ## CENTER LINE
        for i in range(len(points)):
            point_x = points[i][0]
            point_y = points[i][1]
            point_z = 0.001
            self.center_points.append([point_x, point_y, point_z])
        
        ## LEFT LINE
        for i in range(len(points)):
            point_x = points[i][2]
            point_y = points[i][3]
            point_z = 0.001
            self.left_points.append([point_x, point_y, point_z])

        ## RIGHT LINE
        for i in range(len(points)):
            point_x = points[i][4]
            point_y = points[i][5]
            point_z = 0.001
            self.right_points.append([point_x, point_y, point_z])

    def callback_odom(self,data):
        self.odom = data

    def callback_points(self):
        #Publish points
        head = Header()
        head.frame_id = "odom"
        head.stamp = self.get_clock().now().to_msg()

        center_points = pc2.create_cloud_xyz32(head, self.center_points)
        left_points = pc2.create_cloud_xyz32(head, self.left_points)
        right_points = pc2.create_cloud_xyz32(head, self.right_points)

        self.center_points_pub.publish(center_points)
        self.left_points_pub.publish(left_points)
        self.right_points_pub.publish(right_points)
            
def main(args=None):
   rclpy.init(args=args)
   line_publisher =LinePublisher()
   rclpy.spin(line_publisher)
   line_publisher.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()
