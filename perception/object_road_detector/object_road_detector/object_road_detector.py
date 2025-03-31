import copy
from typing import List
from dataclasses import dataclass
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from aiformula_interfaces.msg import RectMultiArray  # noqa: E402
from .object_road_detector_util import get_params, init_detector, pad_image, decode_lane_line_output, decode_object_output, publish_lane_line, publish_rects, publish_result  # noqa: E402


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
        self.architecture, self.path_to_weights, self.mean, self.stdev, self.confidence_threshold, self.iou_threshold = get_params(
            self)
        self.parameters = ObjectRoadDetectorParams(architecture=self.architecture, path_to_weights=self.path_to_weights, mean=self.mean,
                                                   stdev=self.stdev, confidence_threshold=self.confidence_threshold, iou_threshold=self.iou_threshold)
        init_detector(self)
        self.cv_bridge = CvBridge()
        buffer_size = 10
        self.image_sub = self.create_subscription(
            Image, 'sub_image', self.image_callback, buffer_size)
        self.annotated_mask_image_pub = self.create_publisher(Image, 'pub_annotated_mask_image', buffer_size)
        self.lane_mask_image_pub = self.create_publisher(Image, 'pub_mask_image', buffer_size)
        self.rects_pub = self.create_publisher(RectMultiArray, 'pub_object_pose', buffer_size)

    def image_callback(self, msg):
        try:
            undistorted_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        # Padded resize
        paded_image, ratio_to_padding, top, bottom, left, right = pad_image(undistorted_image)
        normalize_image = self.transform(paded_image).to(self.parameters.architecture)
        # Input image
        input_image = normalize_image.half() if self.use_half_precision else normalize_image.float()  # uint8 to fp16/32
        if input_image.ndimension() == 3:
            input_image = input_image.unsqueeze(0)
        # Inference
        detect_out, _, ll_seg_out = self.model(input_image)  # ll_seg_out -> lane line segmentation output
        ll_seg_mask = decode_lane_line_output(self, ll_seg_out, input_image, ratio_to_padding, top, bottom, left, right)
        object_detect = decode_object_output(self, detect_out)
        bbox_detect = copy.deepcopy(object_detect)
        # Publish
        publish_lane_line(self, ll_seg_mask, msg.header)
        publish_rects(self, input_image, undistorted_image, bbox_detect, msg.header)
        publish_result(self, undistorted_image, input_image, ll_seg_mask, object_detect, msg.header)


def main():
    with torch.no_grad():
        rclpy.init()
        object_road_detector = ObjectRoadDetector()
        rclpy.spin(object_road_detector)
        object_road_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
