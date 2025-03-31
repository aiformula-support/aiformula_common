from typing import Any
import os.path as osp
from pathlib import Path
import sys
import cv2
import numpy as np
import random
from ament_index_python.packages import get_package_prefix
import torch
import torchvision.transforms as transforms

package_name = str(Path(__file__).resolve().parent.name)
workspace_dir = Path(get_package_prefix(package_name)).parents[1]
yolop_dir = osp.join(workspace_dir, "..")
if yolop_dir not in sys.path:
    sys.path.insert(0, yolop_dir)
    sys.path.insert(0, osp.join(yolop_dir, "YOLOP"))  # add ROOT to PATH


from YOLOP.lib.config import cfg                 # noqa: E402
from YOLOP.lib.utils import letterbox_for_img    # noqa: E402
from YOLOP.lib.utils.utils import select_device  # noqa: E402
from YOLOP.lib.models import get_net             # noqa: E402
from YOLOP.lib.core.general import non_max_suppression, scale_coords  # noqa: E402


from common_python.get_ros_parameter import get_ros_parameter  # noqa: E402
from aiformula_interfaces.msg import Rect, RectMultiArray      # noqa: E402


def get_params(self) -> Any:  # get ros parameters
    architecture = get_ros_parameter(self, 'use_architecture')
    path_to_weights = get_ros_parameter(self, 'weight_path')
    mean = get_ros_parameter(self, 'normalization.mean')
    stdev = get_ros_parameter(self, 'normalization.standard_deviation')
    confidence_threshold = get_ros_parameter(self, 'confidence_threshold')
    iou_threshold = get_ros_parameter(self, 'iou_threshold')
    return architecture, path_to_weights, mean, stdev, confidence_threshold, iou_threshold


def init_detector(self):   # initialize model and parameters
    load_model(self)
    # Nomalization and Tensor
    self.transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize(self.parameters.mean, self.parameters.stdev),
    ])
    # Get names and colors
    self.bbox_names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
    self.bbox_colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(self.bbox_names))]


def load_model(self):  # model load
    self.architecture = select_device(device=str(self.parameters.architecture))
    self.use_half_precision = (self.architecture.type != 'cpu')  # half precision only supported on CUDA
    self.model = get_net(cfg)
    checkpoint = torch.load(self.parameters.path_to_weights, map_location=self.architecture)
    self.model.load_state_dict(checkpoint['state_dict'])
    if self.architecture.type == 'cuda':
        self.model = self.model.to(self.architecture)
        self.model.half()  # to FP16
    self.model.eval()


def pad_image(image) -> Any:  # Pad image
    paded_image, (ratio_to_pad, _), (dw, dh) = letterbox_for_img(
        image, new_shape=640, auto=True)    # ratio_to_padding (width, height)
    top, bottom = round(dh - 0.1), round(dh + 0.1)
    left, right = round(dw - 0.1), round(dw + 0.1)
    return np.ascontiguousarray(paded_image), ratio_to_pad, top, bottom, left, right


def decode_lane_line_output(self, ll_seg_out, input_image, ratio_to_padding, top, bottom, left, right) -> np.ndarray:   # Lane Line decodor
    _, _, height, width = input_image.shape
    ll_predict = ll_seg_out[:, :, top:(height-bottom), left:(width-right)]
    ll_seg_mask_raw = torch.nn.functional.interpolate(
        ll_predict, scale_factor=int(1/ratio_to_padding), mode='bilinear')
    _, ll_seg_poits = torch .max(ll_seg_mask_raw, 1)
    ll_seg_mask = ll_seg_poits.int().squeeze().cpu().numpy()
    return ll_seg_mask


def decode_object_output(self, detect_out) -> torch.Tensor:  # Object decodor
    object_out, _ = detect_out
    detect_predict = non_max_suppression(object_out, conf_thres=self.parameters.confidence_threshold,
                                         iou_thres=self.parameters.iou_threshold, classes=None, agnostic=False)
    detect = detect_predict[0]
    return detect


def publish_lane_line(self, ll_seg_mask, header):   # publish lane line mask image
    ll_seg_mask = np.array(ll_seg_mask, dtype='uint8')
    ll_seg_mask_msg = self.cv_bridge.cv2_to_imgmsg(ll_seg_mask, "mono8")
    ll_seg_mask_msg.header = header
    self.lane_mask_image_pub.publish(ll_seg_mask_msg)


def publish_rects(self, input_image, undistorted_image, bbox_detect, header):   # punlish rects for bbox
    bbox_poses = []
    bbox_detect[:, :4] = scale_coords(input_image.shape[2:], bbox_detect[:, :4], undistorted_image.shape).round()
    for *xyxy, _, _ in reversed(bbox_detect):
        bbox_poses.append(xyxy)
    rect_array = listToRects(bbox_poses)
    rect_array.header = header
    self.rects_pub.publish(rect_array)


def listToRects(object_poses) -> RectMultiArray:
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


def publish_result(self, undistorted_image, input_image, ll_seg_mask, object_detect, header):   # Publish result image for visualization
    annotated_lane_image = show_seg_result(undistorted_image, (ll_seg_mask, None), None, None, is_demo=True)
    annotated_box_image = draw_bounding_box(self, input_image, annotated_lane_image, object_detect)
    annotated_mask_image = self.cv_bridge.cv2_to_imgmsg(annotated_box_image, "bgr8")
    annotated_mask_image.header = header
    self.annotated_mask_image_pub.publish(annotated_mask_image)


def show_seg_result(img, result, index, epoch, save_dir=None, is_ll=False, palette=None, is_demo=False, is_gt=False) -> np.uint8:
    if palette is None:
        palette = np.random.randint(
            0, 255, size=(3, 3))
    palette[0] = [0, 0, 0]
    palette[1] = [0, 255, 0]
    palette[2] = [255, 0, 0]
    palette = np.array(palette)
    assert palette.shape[0] == 3  # len(classes)
    assert palette.shape[1] == 3
    assert len(palette.shape) == 2

    color_area = np.zeros(
        (result[0].shape[0], result[0].shape[1], 3), dtype=np.uint8)
    color_area[result[0] == 1] = [0, 255, 0]
    color_area[result[1] == 1] = [255, 0, 0]
    color_seg = color_area
    # convert to BGR
    color_seg = color_seg[..., ::-1]
    color_mask = np.mean(color_seg, 2)
    img[color_mask != 0] = img[color_mask != 0] * \
        0.5 + color_seg[color_mask != 0] * 0.5
    img = img.astype(np.uint8)
    return img


def draw_bounding_box(self, image, annotated_image, object_detect) -> np.ndarray:   # draw bbox with opencv
    detect = object_detect
    detect[:, :4] = scale_coords(image.shape[2:], detect[:, :4], annotated_image.shape).round()
    for *xyxy, conf, cls in reversed(detect):
        label_detect_predict = f'{self.bbox_names[int(cls)]} {conf:.2f}'
        plot_one_box(xyxy, annotated_image, label=label_detect_predict,
                     color=self.bbox_colors[int(cls)], line_thickness=2)
    return annotated_image


def plot_one_box(x, img, color=None, label=None, line_thickness=None):   # Plots one bounding box on image img
    tl = line_thickness or round(
        0.0001 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
