import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt

class BevCamera(Node):
	def __init__(self):
		super().__init__('bev_camera')
		self.image_sub = self.create_subscription(Image, "/zed_image", self.callback_image,10)
		self.image = np.zeros((480,640,3), np.uint8)
		self.cv_image = np.zeros((480,640,3), np.uint8)
		self.bridge = CvBridge()

	def callback_image(self, data):
		try:
			self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		IMAGE_H = 480
		IMAGE_W = 640
#		src = np.float32([[0, IMAGE_H], [1280, IMAGE_H], [0, 0], [IMAGE_W, 0]])
		src = np.float32([[0, 350], [1250, 350], [200, 240], [1000, 240]])
#		src = np.float32([[0, 310], [1250, 310], [210, 170], [1070, 170]])  #600
#		src = np.float32([[0, 400], [1250, 400], [210, 310], [1070, 310]])  #550
#		src = np.float32([[0, 310], [1250, 310], [210, 200], [1070, 200]])  #500
#		src = np.float32([[0, 375], [1250, 375], [210, 250], [1070, 250]])  #450
		dst = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [0, 0], [IMAGE_W, 0]])
		M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
		Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

#		img = self.cv_image[450:(450+IMAGE_H), 0:IMAGE_W] # Apply np slicing for ROI crop

		warped_img = cv2.warpPerspective(self.cv_image, M, (IMAGE_W, IMAGE_H)) # Image warping
		self.cv_image = cv2.drawMarker(self.cv_image, (1250, 310), (255, 0, 0))
		self.cv_image = cv2.drawMarker(self.cv_image, (0, 310), (255, 0, 0))
		self.cv_image = cv2.drawMarker(self.cv_image, (210, 200), (255, 0, 0))
		self.cv_image = cv2.drawMarker(self.cv_image, (1070, 200), (255, 0, 0))

#		self.image_pub.publish(self.bridge.cv2_to_imgmsg(warped_img, "bgr8"))

		cv2.imshow("Image", self.cv_image)
		cv2.imshow("Line edges", warped_img)
		cv2.waitKey(1)

def main(args=None):
   rclpy.init(args=args)
   bev_camera = BevCamera()
   rclpy.spin(bev_camera)
   
   bev_camera.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()
