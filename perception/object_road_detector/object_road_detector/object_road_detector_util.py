import cv2
import numpy as np
import random


def show_seg_result(img, result, index, epoch, save_dir=None, is_ll=False, palette=None, is_demo=False, is_gt=False):
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


def plot_one_box(x, img, color=None, label=None, line_thickness=None):
    # Plots one bounding box on image img
    tl = line_thickness or round(
        0.0001 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    cv2.drawMarker(img, position=c1, color=(0, 255, 0), markerType=cv2.MARKER_CROSS,
                   markerSize=20, thickness=2, line_type=cv2.LINE_4)
