import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
#import sensor_msgs.msg as pc2 
from std_msgs.msg import Header

class BEVPointCloud(Node):
	def __init__(self):
		super().init_node('bev_pointcloud')
		self.image_sub = self.create_subscription("ll_result", Image, self.callback_transformer,10)
		self.ll_points_pub = self.create_publisher(PointCloud2, 'll_points', 10)
		self.image = Image() 
		self.bridge = CvBridge()

#-------------------------------------------#
# Image Bird eye visualization transformer #
#-------------------------------------------#

	def callback_transformer(self, data):
		self.ll_seg_mask = self.bridge.imgmsg_to_cv2(data, "mono8")
		s = np.array(self.ll_seg_mask)
		x = s.nonzero()
		x = np.float32(np.column_stack(x))[:,[1,0]]

		src = np.float32([[540,300],[720,300],[0,492],[1280,492]])
		dst = np.float32([[-1.2, 25], [1.2, 25], [-1.2, 1], [1.2, 1]])

		homography, mask = cv2.findHomography(src, dst)
		warped_pt = cv2.perspectiveTransform(np.array([x]), homography) # The transformation matrix
#		print(warped_pt.shape)
		header = Header()
		header.stamp = rclpy.Time.now()
		header.frame_id = "base_link"
		pcd=[]

		for i in range(len(warped_pt[0])):
			point_x = warped_pt[0,i,1]
			point_y = warped_pt[0,i,0]
			# point_x = warped_pt[0,i,0]
			# point_y = warped_pt[0,i,1]
			point_z = 0.1
			pcd.append([point_x, point_y, point_z])

		point_cloud = pc2.create_cloud_xyz32(header, pcd)
		self.ll_points_pub.publish(point_cloud)

def main(args=None):
	rclpy.init(args=args)
	bev_pointcloud = BEVPointCloud()
	rclpy.spin(bev_pointcloud)
	bev_pointcloud.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()