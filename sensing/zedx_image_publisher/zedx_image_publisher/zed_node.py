import rclpy
from rclpy.node import Node
import time

from sensor_msgs.msg import CompressedImage,Image
from cv_bridge import CvBridge
import sys
import cv2
import numpy as np
import pyzed.sl as sl

r_resize = 1

class ZedXImagePublisher(Node):
    def __init__(self):
        super().__init__('zedx_image_publisher')
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 60
        self.zed = sl.Camera()
        status = self.zed.open(init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            exit(-1)
        
        self.runtime = sl.RuntimeParameters()
        # self.runtime.sensing_mode = sl.SENSING_MODE.STANDARD

        self.image_size = self.zed.get_camera_information().camera_configuration.resolution
        # self.image_size.width = self.image_size.width /2
        # self.image_size.height = self.image_size.height /2
        self.image_zed = sl.Mat(self.image_size.width, self.image_size.height, sl.MAT_TYPE.U8_C4)

        self.commpressed_img_pub_ = self.create_publisher(CompressedImage, '/aiformula_sensing/zedx_image_publisher/image_raw/compressed', 10)
        self.raw_img_pub_ = self.create_publisher(Image, '/aiformula_sensing/zedx_image_publisher/image_raw', 1)
        timer_cv2 = 1/30   # 15 FPS
        timer_period_raw = 1/30 # publish timer
        timer_period_com = 1/60

        self.timer_cv2 = self.create_timer(timer_cv2, self.cv2_callback)
        #self.timer_img_pub = self.create_timer(timer_period_raw, self.timer_img_callback)
        self.timer_pub = self.create_timer(timer_period_com, self.timer_callback)
       
        self.br = CvBridge()
        self.time_old = time.time()
        self.time_now = time.time()

    
    def cv2_callback(self):
        self.err = self.zed.grab(self.runtime)
        if self.err == sl.ERROR_CODE.SUCCESS :
            image_sl_left = sl.Mat()
            self.zed.retrieve_image(image_sl_left, sl.VIEW.RIGHT, sl.MEM.CPU, self.image_size)
            image_cv_left = image_sl_left.get_data()

            image_sl_right = sl.Mat()
            self.zed.retrieve_image(image_sl_right, sl.VIEW.LEFT, sl.MEM.CPU, self.image_size)
            image_cv_right = image_sl_right.get_data()
            
            sbs_image = np.concatenate((image_cv_left, image_cv_right), axis=1)

            pub_img = self.br.cv2_to_imgmsg(sbs_image, 'passthrough') 
            pub_img.header.stamp = self.get_clock().now().to_msg()
            self.raw_img_pub_.publish(pub_img)
 
            # self.time_old = self.time_now
            # self.time_now = time.time()
            # print(self.time_now - self.time_old, pub_img.header.stamp)


 #           cv2.imshow("Image", sbs_image)
 #           cv2.waitKey(1)

    # def timer_img_callback(self):
    #     # self.time_old = self.time_now
    #     # self.time_now = time.time()
    #     # print(self.time_now - self.time_old)

    #     if self.ret == True:
    #         pub_raw = self.photo
    #         pub_raw = cv2.resize(pub_raw, (self.w, self.h))

    #         pub_img = self.br.cv2_to_imgmsg(pub_raw, 'bgr8') 
    #         pub_img.header.stamp = self.get_clock().now().to_msg()
    #         self.raw_img_pub_.publish(pub_img) 
   
    def timer_callback(self):
        # self.time_old = self.time_now
        # self.time_now = time.time()
        # print(self.time_now - self.time_old)

        self.err = self.zed.grab(self.runtime)
        if self.err == sl.ERROR_CODE.SUCCESS :
            image_sl_left = sl.Mat()
            self.zed.retrieve_image(image_sl_left, sl.VIEW.RIGHT, sl.MEM.CPU, self.image_size)
            image_cv_left = image_sl_left.get_data()

            image_sl_right = sl.Mat()
            self.zed.retrieve_image(image_sl_right, sl.VIEW.LEFT, sl.MEM.CPU, self.image_size)
            image_cv_right = image_sl_right.get_data()
            
            sbs_image = np.concatenate((image_cv_left, image_cv_right), axis=1)   
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"

            ret, pub_img = cv2.imencode(".jpg", sbs_image, [int(cv2.IMWRITE_JPEG_QUALITY),90])
            msg.data = pub_img.tostring()
            
            # print(self.image_size.height)
            # print(len(pub_img.data))
            
            self.commpressed_img_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    zedx_image_publisher = ZedXImagePublisher()
    
    rclpy.spin(zedx_image_publisher)
    
    zedx_image_publisher.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
