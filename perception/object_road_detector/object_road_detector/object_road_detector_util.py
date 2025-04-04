import os.path as osp
from pathlib import Path
import sys
import cv2
import numpy as np
import random
from ament_index_python.packages import get_package_prefix

package_name = str(Path(__file__).resolve().parent.name)
workspace_dir = Path(get_package_prefix(package_name)).parents[1]
yolop_dir = osp.join(workspace_dir, "..")
if yolop_dir not in sys.path:
    sys.path.insert(0, yolop_dir)
    sys.path.insert(0, osp.join(yolop_dir, "YOLOP"))  # add ROOT to PATH

from YOLOP.lib.core.general import scale_coords  # noqa: E402
from aiformula_interfaces.msg import Rect        # noqa: E402


def to_rect(object_poses) -> Rect:
    rect = Rect()
    x, y, height, width = object_poses[0], object_poses[1], abs(
        object_poses[1] - object_poses[3]), abs(object_poses[0] - object_poses[2])
    rect.x = float(x)
    rect.y = float(y)
    rect.height = float(height)
    rect.width = float(width)
    return rect


def draw_bounding_box(image, annotated_image, object_detect, bbox_names, bbox_colors) -> np.ndarray:   # draw bbox with opencv
    detect = object_detect
    detect[:, :4] = scale_coords(image.shape[2:], detect[:, :4], annotated_image.shape).round()
    for *bboxes_coords, conf, cls in reversed(detect):
        label_detect_predict = f'{bbox_names[int(cls)]} {conf:.2f}'
        plot_one_box(bboxes_coords, annotated_image, label=label_detect_predict,
                     color=bbox_colors[int(cls)], line_thickness=2)
    return annotated_image


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


def plot_one_box(x, img, color=None, label=None, line_thickness=None):   # Plots one bounding box on image img
    tl = line_thickness or round(
        0.0001 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
