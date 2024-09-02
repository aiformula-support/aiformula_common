import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import torch
import torchvision.transforms as transforms

from pathlib import Path
from ament_index_python.packages import get_package_prefix
package = str(Path(__file__).resolve().parent.name)
workspace = Path(get_package_prefix(package)).parents[1]
yolop_module_path = workspace/'..'/'YOLOP'
if str(yolop_module_path) not in sys.path:
    sys.path.insert(0, str(yolop_module_path))  # add ROOT to PATH

from lib.config import cfg
from lib.utils import letterbox_for_img
from lib.utils.utils import select_device
from lib.models import get_net
from lib.utils import show_seg_result


class RoadDetector(Node):

    def __init__(self):
        super().__init__('road_detector')
        buffer_size = 10
        self.annotated_mask_image_pub = self.create_publisher(Image, 'pub_annotated_mask_image', buffer_size)
        self.lane_mask_image_pub = self.create_publisher(Image, 'pub_mask_image', buffer_size)
        self.image_sub = self.create_subscription(
            Image, 'sub_image', self.image_callback, buffer_size)
        self.cv_bridge = CvBridge()
        # Set Device and Param
        self.declare_parameter('use_architecture')
        self.declare_parameter('weight_path')
        self.declare_parameter('mean')
        self.declare_parameter('standard_deviation')
        architecture = self.get_parameter('use_architecture').get_parameter_value().string_value
        weights = self.get_parameter('weight_path').get_parameter_value().string_value
        mean = np.array(self.get_parameter('mean').get_parameter_value().double_array_value)
        std = np.array(self.get_parameter('standard_deviation').get_parameter_value().double_array_value)
        self.architecture = select_device(device = architecture)
        self.half = (self.architecture.type != 'cpu')  # half precision only supported on CUDA
        # Load model
        self.model = get_net(cfg)
        checkpoint = torch.load(weights, map_location=self.architecture)
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model = self.model.to(self.architecture)
        if self.half:
            self.model.half()  # to FP16
        # Nomalization and Tensor
        normalize = transforms.Normalize(mean, std)
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            normalize,
        ])

    def image_callback(self, msg):
        try:
            time_stamp = msg.header.stamp
            undistorted_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Set Dataloader and Run inference
        padding_image, shapes = self.LoadImages(undistorted_image)
        normalize_image = self.transform(padding_image).to(self.architecture)
        input_image = normalize_image.half() if self.half else normalize_image.float()  # uint8 to fp16/32
        if input_image.ndimension() == 3:
            input_image = input_image.unsqueeze(0)
        # Inference
        _, _, ll_seg_out = self.model(input_image)  # ll_seg_out -> lane line segmentation output
        _, _, height, width = input_image.shape
        padding_w, padding_h = shapes[1][1]
        padding_w = int(padding_w)
        padding_h = int(padding_h)
        ratio = shapes[1][0][1]    # ratio -> ratio of undistorted image to input image
        # create Lane line segmentation masks
        ll_predict = ll_seg_out[:, :, padding_h:(height-padding_h), padding_w:(width-padding_w)]
        ll_seg_mask_raw = torch.nn.functional.interpolate(ll_predict, scale_factor=int(1/ratio), mode='bilinear')
        _, ll_seg_poits = torch.max(ll_seg_mask_raw, 1)
        ll_seg_mask = ll_seg_poits.int().squeeze().cpu().numpy()
        # visualization
        annotated_image = show_seg_result(undistorted_image, (ll_seg_mask, ll_seg_mask), _, _, is_demo=True)
        # ROS Publish
        bridge = CvBridge()
        ll_array = np.array(ll_seg_mask, dtype='uint8')
        ll_image = bridge.cv2_to_imgmsg(ll_array, "mono8")
        ll_image.header.stamp = time_stamp
        self.lane_mask_image_pub.publish(ll_image)
        self.annotated_mask_image_pub.publish(bridge.cv2_to_imgmsg(annotated_image, "bgr8"))

    def LoadImages(self, image):
        image_height, image_width = image.shape[:2]
        # Padded resize        
        padding_image, _, padding = letterbox_for_img(image, new_shape=640, auto=True)
        padding_height, padding_width = padding_image.shape[:2]
        shapes = (padding_height, padding_width), ((padding_height / image_height, padding_width / image_width), padding)
        # Convert
        convarted_image = np.ascontiguousarray(padding_image)
        return convarted_image, shapes

def main():
    with torch.no_grad():
        rclpy.init()
        road_detector = RoadDetector()
        rclpy.spin(road_detector)
        road_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
