import os.path as osp
from pathlib import Path
import sys
import cv2
import numpy as np
import torch
from ament_index_python.packages import get_package_prefix

package_name = str(Path(__file__).resolve().parent.name)
workspace_dir = Path(get_package_prefix(package_name)).parents[1]
yolop_dir = osp.join(workspace_dir, "..")
if yolop_dir not in sys.path:
    sys.path.insert(0, yolop_dir)
    sys.path.insert(0, osp.join(yolop_dir, "YOLOP"))  # add ROOT to PATH

from YOLOP.lib.core.general import scale_coords  # noqa: E402
from aiformula_interfaces.msg import Rect        # noqa: E402


def to_rect(object_poses: torch.Tensor) -> Rect:
    rect = Rect()
    rect.x = float(object_poses[0])
    rect.y = float(object_poses[1])
    rect.height = float(abs(object_poses[1] - object_poses[3]))
    rect.width = float(abs(object_poses[0] - object_poses[2]))
    return rect


def draw_lane_lines(image: np.ndarray, ll_seg_mask: np.ndarray, alpha: bool = 0.5) -> None:
    overlay = np.zeros_like(image, dtype=np.uint8)
    LANE_LINE_VALUE = 1
    overlay[ll_seg_mask == LANE_LINE_VALUE] = [0, 255, 0]  # Green in BGR
    mask = ll_seg_mask.astype(bool)
    image[mask] = cv2.addWeighted(image[mask], 1 - alpha, overlay[mask], alpha, gamma=0.0)


def draw_bounding_boxes(image: np.ndarray, objects: torch.Tensor, input_image_shepe: torch.Size) -> None:   # draw bbox with opencv
    bboxes_coords = scale_coords(input_image_shepe, objects[:, :4], image.shape).round()
    for bboxes_coord in bboxes_coords:
        plot_one_box(image, bboxes_coord)


def plot_one_box(image: np.ndarray, bboxes_coords: torch.Tensor) -> None:   # Plots one bounding box on image
    top_left = (int(bboxes_coords[0]), int(bboxes_coords[1]))
    bottom_right = (int(bboxes_coords[2]), int(bboxes_coords[3]))
    color = [255, 0, 0]
    thickness = int(image.shape[0] * 0.002)
    cv2.rectangle(image, top_left, bottom_right, color, thickness, lineType=cv2.LINE_AA)
