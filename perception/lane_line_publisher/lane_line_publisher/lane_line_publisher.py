import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
import numpy as np
import sensor_msgs.msg as sensor_msgs
import std_msgs.msg as std_msgs
from sensor_msgs.msg import Image,PointCloud2
import open3d as o3d
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker, MarkerArray

class LaneLinePublisher(Node):
	def __init__(self):
		super().__init__('lane_line_publisher')
		self.image_sub = self.create_subscription(Image, "aiformula_perception/road_detector/mask_image", self.callback_transformer,10)
		self.ll_points_pub = self.create_publisher(PointCloud2, 'aiformula_perception/lane_line_publisher/bev_points', 10)
		self.line_points_pub = self.create_publisher(PointCloud2, 'line_points', 10)
		self.image = Image() 
		self.bridge = CvBridge()
		self.o3d_points = o3d.geometry.PointCloud()
		self.counter = 0
		self.line = []

#-------------------------------------------#
# Image Bird eye visualization transformer #
#-------------------------------------------#

	def callback_transformer(self, data):
		self.ll_seg_mask = self.bridge.imgmsg_to_cv2(data, "mono8")
		s = np.array(self.ll_seg_mask)
		x = s.nonzero()
		x = np.float32(np.column_stack(x))[:,[1,0]]

##Parameter
		# src = np.float32([[785, 500], [1080, 500], [285, 1050], [1500, 1050]]) #simlator param
		# dst = np.float32([[-3.2, 12], [3.2, 12], [-3.2, 3.5], [3.2, 3.5]])

		src = np.float32([[818, 947], [1045, 950], [766, 1046], [1051, 1052]]) #calib_board result real_world
		dst = np.float32([[-0.324, 0.936], [0.324, 0.936], [-0.324, 0.72], [0.324, 0.72]])

		homography, mask = cv2.findHomography(src, dst)
		warped_pt = cv2.perspectiveTransform(np.array([x]), homography) # The transformation matrix
		
		pcd=[]

		for i in range(len(warped_pt[0])):
			point_x = warped_pt[0,i,0]
			if point_x > 10.:
				continue
			point_y = warped_pt[0,i,1]
			if point_y > 20.:
				continue

			point_z = 0.1
			pcd.append([point_x, point_y, point_z])

		self.points= np.asarray(pcd)
		self.o3d_points = o3d.geometry.PointCloud()
		self.o3d_points.points = o3d. utility.Vector3dVector(self.points)
		self.pcd = point_cloud(self.points,'map')
		self.ll_points_pub.publish(self.pcd)

## approximation simulation
		self.downpcd = self.o3d_points.voxel_down_sample(voxel_size=0.05)
		labels =np.array(self.downpcd.cluster_dbscan(eps=0.5, min_points=15, print_progress=True))
		downpcd = np.array(self.downpcd.points)		
		max_label = labels.max()
		clusters = []
		line_clusters =[]

		for label in np.unique(labels):
			indices = np.flatnonzero(labels==label)
			cluster = downpcd[indices]
			clusters.append(cluster)

		for i in range(max_label):
			xs = np.linspace(min(clusters[i][:,0]),max(clusters[i][:,0]),len(clusters[i][:,0]))
			res = np.polyfit(clusters[i][:,0],clusters[i][:,1],3)
			line = np.poly1d(res)(xs)
			line = np.column_stack((xs,line))
			pointz = np.zeros((1,len(line))).T
			line = np.append(line, pointz, axis=1)
			line_clusters.extend(line)
		
		self.clusters = point_cloud(np.array((line_clusters[::20])),'map')
		self.line_points_pub.publish(self.clusters)

def point_cloud(points, parent_frame):
    ros_dtype = sensor_msgs.PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize # A 32-bit float takes 4 bytes.

    data = points.astype(dtype).tobytes() 

    fields = [sensor_msgs.PointField(
        name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
        for i, n in enumerate('xyz')]

    header = std_msgs.Header(frame_id=parent_frame)

    return sensor_msgs.PointCloud2(
        header=header,
        height=1, 
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 3), # Every point consists of three float32s.
        row_step=(itemsize * 3 * points.shape[0]),
        data=data
    )

def main(args=None):
	rclpy.init(args=args)
	lane_line_publisher = LaneLinePublisher()
	rclpy.spin(lane_line_publisher)
	lane_line_publisher.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()
