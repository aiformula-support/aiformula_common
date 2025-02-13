import sys
import numpy as np
import os.path as osp
from numpy import random
import copy
from pathlib import Path
from typing import List
from dataclasses import dataclass
import torch
import torchvision.transforms as transforms
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from ament_index_python.packages import get_package_prefix

package_name = str(Path(__file__).resolve().parent.name)
workspace_dir = Path(get_package_prefix(package_name)).parents[1]
yolop_dir = osp.join(workspace_dir, "..")
if yolop_dir not in sys.path:
    sys.path.insert(0, yolop_dir)
    sys.path.insert(0, osp.join(yolop_dir, "YOLOP"))  # add ROOT to PATH

from YOLOP.lib.config import cfg  # noqa: E402
from YOLOP.lib.utils import letterbox_for_img  # noqa: E402
from YOLOP.lib.utils.utils import select_device  # noqa: E402
from YOLOP.lib.models import get_net  # noqa: E402
from YOLOP.lib.core.general import non_max_suppression, scale_coords  # noqa: E402

from aiformula_interfaces.msg import Rect, RectMultiArray  # noqa: E402
from common_python.get_ros_parameter import get_ros_parameter  # noqa: E402
from .object_road_detector_util import show_seg_result, plot_one_box  # noqa: E402


@dataclass(frozen=True)
class ObjectRoadDetectorParams:
    architecture: str
    path_to_weights: str
    mean: List[float]
    stdev: List[float]
    confidence_threshold: float
    iou_threshold: float


class ObjectRoadDetector(Node):

    def __init__(self):
        super().__init__('object_road_detector')
        self.get_params()
        self.parameters = ObjectRoadDetectorParams(architecture=self.architecture, path_to_weights=self.path_to_weights, mean=self.mean,
                                                   stdev=self.stdev, confidence_threshold=self.confidence_threshold, iou_threshold=self.iou_threshold)
        self.init_detector()
        self.cv_bridge = CvBridge()
        buffer_size = 10
        self.image_sub = self.create_subscription(
            Image, 'sub_image', self.image_callback, buffer_size)
        self.annotated_mask_image_pub = self.create_publisher(Image, 'pub_annotated_mask_image', buffer_size)
        self.lane_mask_image_pub = self.create_publisher(Image, 'pub_mask_image', buffer_size)
        self.rects_pub = self.create_publisher(RectMultiArray, 'pub_object_pose', buffer_size)
        # Get names and colors
        self.bbox_names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.bbox_colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(self.bbox_names))]

    def get_params(self):
        self.architecture = get_ros_parameter(self, 'use_architecture')
        self.path_to_weights = get_ros_parameter(self, 'weight_path')
        self.mean = get_ros_parameter(self, 'normalization.mean')
        self.stdev = get_ros_parameter(self, 'normalization.standard_deviation')
        self.confidence_threshold = get_ros_parameter(self, 'confidence_threshold')
        self.iou_threshold = get_ros_parameter(self, 'iou_threshold')

    def init_detector(self):
        self.load_model()
        # Nomalization and Tensor
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(self.parameters.mean, self.parameters.stdev),
        ])

    def load_model(self):
        self.architecture = select_device(device=str(self.parameters.architecture))
        self.use_half_precision = (self.architecture.type != 'cpu')  # half precision only supported on CUDA
        self.model = get_net(cfg)
        checkpoint = torch.load(self.parameters.path_to_weights, map_location=self.architecture)
        self.model.load_state_dict(checkpoint['state_dict'])
        if self.architecture.type == 'cuda':
            self.model = self.model.to(self.architecture)
            self.model.half()  # to FP16
        self.model.eval()

    def image_callback(self, msg):
        try:
            undistorted_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Padded resize
        paded_image, ratio_to_padding, top, bottom, left, right = self.pad_image(undistorted_image)
        normalize_image = self.transform(paded_image).to(self.parameters.architecture)
        # Input image
        input_image = normalize_image.half() if self.use_half_precision else normalize_image.float()  # uint8 to fp16/32
        if input_image.ndimension() == 3:
            input_image = input_image.unsqueeze(0)
        # Inference
        detect_out, _, ll_seg_out = self.model(input_image)  # ll_seg_out -> lane line segmentation output
        ll_seg_mask = self.decode_lane_line_output(ll_seg_out, input_image, ratio_to_padding, top, bottom, left, right)
        object_detect = self.decode_object_output(detect_out)
        # Publish
        self.publish_lane_line(ll_seg_mask, msg.header)
        self.publish_rects(input_image, undistorted_image, object_detect, msg.header)
        self.publish_result(undistorted_image, input_image, ll_seg_mask, object_detect, msg.header)

    def pad_image(self, image):
        paded_image, (ratio_to_pad, _), (dw, dh) = letterbox_for_img(
            image, new_shape=640, auto=True)    # ratio_to_padding (width, height)
        top, bottom = round(dh - 0.1), round(dh + 0.1)
        left, right = round(dw - 0.1), round(dw + 0.1)
        return np.ascontiguousarray(paded_image), ratio_to_pad, top, bottom, left, right

    def decode_lane_line_output(self, ll_seg_out, input_image, ratio_to_padding, top, bottom, left, right) -> np.ndarray:
        _, _, height, width = input_image.shape
        ll_predict = ll_seg_out[:, :, top:(height-bottom), left:(width-right)]
        ll_seg_mask_raw = torch.nn.functional.interpolate(
            ll_predict, scale_factor=int(1/ratio_to_padding), mode='bilinear')
        _, ll_seg_poits = torch .max(ll_seg_mask_raw, 1)
        ll_seg_mask = ll_seg_poits.int().squeeze().cpu().numpy()
        return ll_seg_mask

    def decode_object_output(self, detect_out) -> torch.Tensor:
        object_out, _ = detect_out
        detect_predict = non_max_suppression(object_out, conf_thres=self.parameters.confidence_threshold,
                                             iou_thres=self.parameters.iou_threshold, classes=None, agnostic=False)
        detect = detect_predict[0]
        return detect

    def publish_result(self, undistorted_image, input_image, ll_seg_mask, object_detect, header):
        # Publish annotated image
        annotated_lane_image = show_seg_result(undistorted_image, (ll_seg_mask, None), None, None, is_demo=True)
        annotated_box_image = self.draw_bounding_box(input_image, annotated_lane_image, object_detect)
        annotated_mask_image = self.cv_bridge.cv2_to_imgmsg(annotated_box_image, "bgr8")
        annotated_mask_image.header = header
        self.annotated_mask_image_pub.publish(annotated_mask_image)

    def publish_lane_line(self, ll_seg_mask, header):
        ll_seg_mask = np.array(ll_seg_mask, dtype='uint8')
        ll_seg_mask_msg = self.cv_bridge.cv2_to_imgmsg(ll_seg_mask, "mono8")
        ll_seg_mask_msg.header = header
        self.lane_mask_image_pub.publish(ll_seg_mask_msg)

    def publish_rects(self, input_image, undistorted_image, object_detect, header):
        bbox_poses = []
        bbox_detect = copy.deepcopy(object_detect)
        bbox_detect[:, :4] = scale_coords(input_image.shape[2:], bbox_detect[:, :4], undistorted_image.shape).round()
        for *xyxy, _, _ in reversed(bbox_detect):
            bbox_poses.append(xyxy)
        rect_array = self.listToRects(bbox_poses)
        rect_array.header = header
        self.rects_pub.publish(rect_array)

    def draw_bounding_box(self, image, annotated_image, object_detect) -> np.ndarray:
        detect = object_detect
        detect[:, :4] = scale_coords(image.shape[2:], detect[:, :4], annotated_image.shape).round()
        for *xyxy, conf, cls in reversed(detect):
            label_detect_predict = f'{self.bbox_names[int(cls)]} {conf:.2f}'
            plot_one_box(xyxy, annotated_image, label=label_detect_predict,
                         color=self.bbox_colors[int(cls)], line_thickness=2)
        return annotated_image

    def listToRects(self, object_poses) -> RectMultiArray:
        rect_array = RectMultiArray()
        for i in range(len(object_poses)):
            rect = Rect()
            x, y, height, width = object_poses[i][0], object_poses[i][1], abs(
                object_poses[i][1] - object_poses[i][3]), abs(object_poses[i][0] - object_poses[i][2])
            rect.x = float(x)
            rect.y = float(y)
            rect.height = float(height)
            rect.width = float(width)
            rect_array.rects.append(rect)
        return rect_array


def main():
    with torch.no_grad():
        rclpy.init()
        object_road_detector = ObjectRoadDetector()
        rclpy.spin(object_road_detector)
        object_road_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
