import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.utilities import remove_ros_args
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import torch
import torchvision.transforms as transforms

from pathlib import Path
from ament_index_python.packages import get_package_prefix
package = str(Path(__file__).resolve().parent.name)
print('package:', package)
workspace = Path(get_package_prefix(package)).parents[1]
ROOT = workspace/'..'/'YOLOP'
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))  # add ROOT to PATH
    ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from lib.config import cfg
from lib.utils import letterbox_for_img
from lib.utils.utils import create_logger, select_device
from lib.models import get_net
from lib.utils import show_seg_result

normalize = transforms.Normalize(
    mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
)

transform = transforms.Compose([
    transforms.ToTensor(),
    normalize,
])


class RoadDetector(Node):

    def __init__(self):
        super().__init__('road_detector')
        self.pub = self.create_publisher(Image, 'pub_annotated_mask_image', 10)
        self.ll_pub = self.create_publisher(Image, 'pub_mask_image', 10)
        self.image = self.create_subscription(
            Image, 'sub_image', self.callback_detect, 10)
        self.cv_bridge = CvBridge()

        logger, _, _ = create_logger(
            cfg, cfg.LOG_DIR, 'demo')

        # Set Device and Param
        self.declare_parameter('use_device')
        self.declare_parameter('weight_path')
        device = self.get_parameter('use_device').get_parameter_value().string_value
        weights = self.get_parameter('weight_path').get_parameter_value().string_value
        self.img_size = 640
        self.device = select_device(logger, device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA
        # Load model
        self.model = get_net(cfg)
        checkpoint = torch.load(weights, map_location=self.device)
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model = self.model.to(self.device)
        if self.half:
            self.model.half()  # to FP16
        img = torch.zeros((1, 3, self.img_size, self.img_size), device=self.device)  # init img
        self.model(img.half() if self.half else img) if self.device.type != 'cpu' else None  # run once
        self.model.eval()

    def callback_detect(self, data):
        try:
            self.image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Set Dataloader and Run interfence
        img, img_det, shapes = self.LoadImages(self.image)
        img = transform(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        # Inference
        det_out, da_seg_out, ll_seg_out = self.model(img)
        _, _, height, width = img.shape
        # h, w, _ = img_det.shape
        pad_w, pad_h = shapes[1][1]
        pad_w = int(pad_w)
        pad_h = int(pad_h)
        pad_h = int(pad_h)
        ratio = shapes[1][0][1]

        ll_predict = ll_seg_out[:, :, pad_h:(height-pad_h), pad_w:(width-pad_w)]
        ll_seg_mask = torch.nn.functional.interpolate(ll_predict, scale_factor=int(1/ratio), mode='bilinear')
        _, ll_seg_mask = torch.max(ll_seg_mask, 1)
        ll_seg_mask = ll_seg_mask.int().squeeze().cpu().numpy()

        img_det = show_seg_result(img_det, (ll_seg_mask, ll_seg_mask), _, _, is_demo=True)
        ll_seg_mask = np.array(ll_seg_mask, dtype='uint8')
        # ROS Publish
        bridge = CvBridge()
        ll_image = bridge.cv2_to_imgmsg(ll_seg_mask, "mono8")
        ll_image.header.stamp = self.get_clock().now().to_msg()
        self.ll_pub.publish(ll_image)
        self.pub.publish(bridge.cv2_to_imgmsg(img_det, "bgr8"))

    def LoadImages(self, img):
        img0 = img
        h0, w0 = img0.shape[:2]
        # Padded resize
        img, ratio, pad = letterbox_for_img(img0, new_shape=self.img_size, auto=True)
        h, w = img.shape[:2]
        shapes = (h0, w0), ((h / h0, w / w0), pad)
        # Convert
        img = np.ascontiguousarray(img)
        return img, img0, shapes


def main():
    # opt = parse_opt()
    with torch.no_grad():
        rclpy.init()
        road_detector = RoadDetector()
        # road_detector = RoadDetector(cfg, opt)
        rclpy.spin(road_detector)
        road_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
