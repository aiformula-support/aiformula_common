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


def draw_lane_lines(image: np.ndarray, result: np.ndarray) -> np.uint8:
    palette = np.random.randint(0, 255, size=(3, 3))
    palette[0] = [0, 0, 0]
    palette[1] = [0, 255, 0]
    palette = np.array(palette)
    assert palette.shape[0] == 3  # len(classes)
    assert palette.shape[1] == 3
    assert len(palette.shape) == 2
    color_area = np.zeros(
        (result.shape[0], result.shape[1], 3), dtype=np.uint8)
    color_area[result == 1] = [0, 255, 0]
    color_seg = color_area
    # convert to BGR
    color_seg = color_seg[..., ::-1]
    color_mask = np.mean(color_seg, 2)
    image[color_mask != 0] = image[color_mask != 0] * \
        0.5 + color_seg[color_mask != 0] * 0.5
    image = image.astype(np.uint8)
    return image


def draw_bounding_boxes(image_shepe: torch.Size, annotated_image: np.ndarray, objects: torch.Tensor) -> None:   # draw bbox with opencv
    bboxes_coords = scale_coords(image_shepe, objects[:, :4], annotated_image.shape).round()
    for bboxes_coord in bboxes_coords:
        plot_one_box(bboxes_coord, annotated_image)


def plot_one_box(bboxes_coords: torch.Tensor, image: np.ndarray) -> None:   # Plots one bounding box on image
    line_thickness = round(0.0001 * (image.shape[0] + image.shape[1]) / 2) + 1  # line/font thickness
    color = [255, 0, 0]
    top_left, bottom_right = (int(bboxes_coords[0]), int(bboxes_coords[1])
                              ), (int(bboxes_coords[2]), int(bboxes_coords[3]))
    cv2.rectangle(image, top_left, bottom_right, color, line_thickness, lineType=cv2.LINE_AA)
