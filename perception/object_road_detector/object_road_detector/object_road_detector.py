import os.path as osp
import copy
import sys
from pathlib import Path
from typing import Any
import random
import numpy as np
import torch
import torchvision.transforms as transforms
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_prefix

package_name = str(Path(__file__).resolve().parent.name)
workspace_dir = Path(get_package_prefix(package_name)).parents[1]
yolop_dir = osp.join(workspace_dir, "..")
if yolop_dir not in sys.path:
    sys.path.insert(0, yolop_dir)
    sys.path.insert(0, osp.join(yolop_dir, "YOLOP"))  # add ROOT to PATH

from YOLOP.lib.config import cfg                 # noqa: E402
from YOLOP.lib.utils.utils import select_device  # noqa: E402
from YOLOP.lib.utils import letterbox_for_img    # noqa: E402
from YOLOP.lib.models import get_net             # noqa: E402
from YOLOP.lib.core.general import non_max_suppression, scale_coords  # noqa: E402

from common_python.get_ros_parameter import get_ros_parameter  # noqa: E402
from aiformula_interfaces.msg import RectMultiArray  # noqa: E402
from .object_road_detector_util import show_seg_result, to_rect, draw_bounding_box  # noqa: E402


class ObjectRoadDetector(Node):

    def __init__(self):
        super().__init__('object_road_detector')
        device, path_to_weights, mean, stdev = self.get_params()
        self.init_detector(device, path_to_weights, mean, stdev)
        self.cv_bridge = CvBridge()
        buffer_size = 10
        self.image_sub = self.create_subscription(
            Image, 'sub_image', self.image_callback, buffer_size)
        self.annotated_mask_image_pub = self.create_publisher(Image, 'pub_annotated_mask_image', buffer_size)
        self.lane_mask_image_pub = self.create_publisher(Image, 'pub_mask_image', buffer_size)
        self.rects_pub = self.create_publisher(RectMultiArray, 'pub_bbox', buffer_size)

    def get_params(self) -> Any:  # get ros parameters
        device = get_ros_parameter(self, 'use_device')
        path_to_weights = get_ros_parameter(self, 'weight_path')
        mean = get_ros_parameter(self, 'normalization.mean')
        stdev = get_ros_parameter(self, 'normalization.standard_deviation')
        self.confidence_threshold = get_ros_parameter(self, 'confidence_threshold')
        self.iou_threshold = get_ros_parameter(self, 'iou_threshold')
        return device, path_to_weights, mean, stdev

    def init_detector(self, device, path_to_weights, mean, stdev):   # initialize detector and parameters
        self.load_detector(device, path_to_weights)
        # Normalization and Tensor
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean, stdev),
        ])
        # Get names and colors
        self.bbox_names = self.detector.module.names if hasattr(self.detector, 'module') else self.detector.names
        self.bbox_colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(self.bbox_names))]

    def load_detector(self, device, path_to_weights):  # detector load
        self.use_device = select_device(device=str(device))
        self.use_half_precision = (self.use_device.type != 'cpu')  # half precision only supported on CUDA
        self.detector = get_net(cfg)
        checkpoint = torch.load(path_to_weights, map_location=self.use_device)
        self.detector.load_state_dict(checkpoint['state_dict'])
        if self.use_device.type == 'cuda':
            self.detector = self.detector.to(self.use_device)
            self.detector.half()  # to FP16
        self.detector.eval()

    def image_callback(self, msg):
        try:
            undistorted_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Padded resize
        paded_image, (ratio_to_padding, _), (dw, dh) = letterbox_for_img(undistorted_image,
                                                                         new_shape=640, auto=True)  # ratio_to_padding (width, height)
        normalized_tensor = self.transform(paded_image).to(self.use_device)
        # Input image
        input_image = normalized_tensor.half() if self.use_half_precision else normalized_tensor.float()  # uint8 to fp16/32
        input_image = input_image.unsqueeze(0)
        # Inference
        detect_out, _, ll_seg_out = self.detector(input_image)  # ll_seg_out -> lane line segmentation output
        ll_seg_mask = self.decode_lane_line_output(ll_seg_out, input_image, ratio_to_padding, dw, dh)
        object_detect = self.decode_object_output(detect_out)
        # Publish
        self.publish_lane_line(ll_seg_mask, msg.header)
        self.publish_rects(input_image, undistorted_image, object_detect, msg.header)
        self.publish_result_image(undistorted_image, input_image, ll_seg_mask,
                                  object_detect, msg.header, self.bbox_names, self.bbox_colors)

    def decode_lane_line_output(self, ll_seg_out, input_image, ratio_to_padding, dw, dh) -> np.ndarray:   # Lane Line decodor
        _, _, height, width = input_image.shape
        top, bottom = round(dh - 0.1), round(dh + 0.1)
        left, right = round(dw - 0.1), round(dw + 0.1)
        ll_predict = ll_seg_out[:, :, top:(height-bottom), left:(width-right)]
        ll_seg_mask_raw = torch.nn.functional.interpolate(
            ll_predict, scale_factor=int(1/ratio_to_padding), mode='bilinear')
        _, ll_seg_points = torch .max(ll_seg_mask_raw, 1)
        ll_seg_mask = ll_seg_points.int().squeeze().cpu().numpy()
        return ll_seg_mask

    def decode_object_output(self, detect_out) -> torch.Tensor:  # Object decodor
        object_out, _ = detect_out
        detect_predict = non_max_suppression(object_out, conf_thres=self.confidence_threshold,
                                             iou_thres=self.iou_threshold, classes=None, agnostic=False)
        detect = detect_predict[0]
        return detect

    def publish_lane_line(self, ll_seg_mask, header):   # publish lane line mask image
        ll_seg_mask = np.array(ll_seg_mask, dtype='uint8')
        ll_seg_mask_msg = self.cv_bridge.cv2_to_imgmsg(ll_seg_mask, "mono8")
        ll_seg_mask_msg.header = header
        self.lane_mask_image_pub.publish(ll_seg_mask_msg)

    def publish_rects(self, input_image, undistorted_image, object_detect, header):   # punlish rects for bbox
        bbox_detect = copy.deepcopy(object_detect)
        bbox_detect[:, :4] = scale_coords(input_image.shape[2:], bbox_detect[:, :4], undistorted_image.shape).round()
        bbox_msg = RectMultiArray()
        for *bboxes_coords, _, _ in reversed(bbox_detect):
            bbox_msg.rects.append(to_rect(bboxes_coords))
        bbox_msg.header = header
        self.rects_pub.publish(bbox_msg)

    # Publish result image for visualization
    def publish_result_image(self, undistorted_image, input_image, ll_seg_mask, object_detect, header, bbox_names, bbox_colors):
        annotated_lane_image = show_seg_result(undistorted_image, (ll_seg_mask, None), None, None, is_demo=True)
        annotated_box_image = draw_bounding_box(input_image, annotated_lane_image,
                                                object_detect, bbox_names, bbox_colors)
        annotated_mask_image = self.cv_bridge.cv2_to_imgmsg(annotated_box_image, "bgr8")
        annotated_mask_image.header = header
        self.annotated_mask_image_pub.publish(annotated_mask_image)


def main():
    rclpy.init()
    object_road_detector = ObjectRoadDetector()
    try:
        with torch.no_grad():
            rclpy.spin(object_road_detector)
    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt (Ctrl+C), shutting down...")
    finally:
        object_road_detector.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
