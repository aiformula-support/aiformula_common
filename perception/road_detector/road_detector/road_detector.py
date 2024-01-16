import argparse
import os, sys
import platform
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from pathlib import Path
from rclpy.utilities import remove_ros_args

import cv2
from cv_bridge import CvBridge,CvBridgeError

import torch
import numpy as np
import torchvision.transforms as transforms

#ワークスペースの直下にディレクトリyolopがあることを仮定
package = str(Path(__file__).resolve().parent.name)
print('package:', package)
from ament_index_python.packages import get_package_prefix
workspace = Path(get_package_prefix(package)).parents[1]
ROOT = workspace / 'YOLOP'
if str(ROOT) not in sys.path:
   sys.path.insert(0, str(ROOT))  # add ROOT to PATH
   ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from lib.config import cfg
from lib.utils.utils import create_logger, select_device
from lib.models import get_net
from lib.dataset.DemoDataset_ros import LoadImages
from lib.utils import plot_one_box,show_seg_result

#from models.common import DetectMultiBackend

normalize = transforms.Normalize(
    mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
    )

transform=transforms.Compose([
    transforms.ToTensor(),
    normalize,
        ])

class Detection(Node):

    def __init__(self,cfg, opt):
        super().__init__('aiformula_perception/road_detector')
        self.pub = self.create_publisher(Image, 'aiformula_perception/road_detector/result_image', 10)
        self.ll_pub = self.create_publisher(Image, 'aiformula_perception/road_detector/mask_image', 10)
        self.image = self.create_subscription(Image,'aiformula_sensing/zedx_image_publisher/image_raw', self.callback_detect, 10)
        self.cv_bridge = CvBridge() 

        logger, _, _ = create_logger(
            cfg, cfg.LOG_DIR, 'demo')
            
        self.opt = opt
        self.device = select_device(logger,self.opt.device)
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA
        # Load model
        self.model = get_net(cfg)
        print(self.device)
        checkpoint = torch.load(self.opt.weights, map_location= self.device)
        self.model.load_state_dict(checkpoint['state_dict'])
        self.model = self.model.to(self.device)
        if self.half:
            self.model.half()  # to FP16

    def callback_detect(self,data):
        try:
            self.image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
            self.image = cv2.resize(self.image,(640,480))
        except CvBridgeError as e:
            print(e)

        # Set Dataloader
        dataset = LoadImages(self.image, img_size=self.opt.img_size)

        # Run inference
        img = torch.zeros((1, 3, self.opt.img_size, self.opt.img_size), device=self.device)  # init img
        self.model(img.half() if self.half else img) if self.device.type != 'cpu' else None  # run once
        self.model.eval()

        img, img_det, shapes = next(dataset)
    
        img = transform(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        if img.ndimension() == 3:
            img = img.unsqueeze(0)
        # Inference
        det_out, da_seg_out,ll_seg_out= self.model(img)

        _, _, height, width = img.shape
        h,w,_=img_det.shape
        pad_w, pad_h = shapes[1][1]
        pad_w = int(pad_w)
        pad_h = int(pad_h)
        pad_h = int(pad_h)
        ratio = shapes[1][0][1]

        da_predict = da_seg_out[:, :, pad_h:(height-pad_h),pad_w:(width-pad_w)]
        da_seg_mask = torch.nn.functional.interpolate(da_predict, scale_factor=int(1/ratio), mode='bilinear')
        _, da_seg_mask = torch.max(da_seg_mask, 1)
        da_seg_mask = da_seg_mask.int().squeeze().cpu().numpy()
#        da_seg_mask = morphological_process(da_seg_mask, kernel_size=7)
        
        ll_predict = ll_seg_out[:, :,pad_h:(height-pad_h),pad_w:(width-pad_w)]
        ll_seg_mask = torch.nn.functional.interpolate(ll_predict, scale_factor=int(1/ratio), mode='bilinear')
        _, ll_seg_mask = torch.max(ll_seg_mask, 1)
        ll_seg_mask = ll_seg_mask.int().squeeze().cpu().numpy()

        img_det = show_seg_result(img_det, (da_seg_mask, ll_seg_mask), _, _, is_demo=True)
        ll_seg_mask = np.array(ll_seg_mask, dtype='uint8')

        bridge=CvBridge()
        self.pub.publish(bridge.cv2_to_imgmsg(img_det,"bgr8"))
        self.ll_pub.publish(bridge.cv2_to_imgmsg(ll_seg_mask, "mono8"))

def parse_opt():
    sys.argv = remove_ros_args(args=sys.argv)
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', nargs='+', type=str, default= ROOT/'weights/End-to-end.pth', help='model.pth path(s)')
    parser.add_argument('--source', type=str, default='videos', help='source')  # file/folder   ex:inference/images
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
    parser.add_argument('--device', default='cpu', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--save-dir', type=str, default='output', help='directory to save results')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    parser.add_argument('--update', action='store_true', help='update all models')
    opt = parser.parse_args()
    return opt

def main():
    opt = parse_opt()
    with torch.no_grad():
        rclpy.init()
        detection = Detection(cfg,opt)
        rclpy.spin(detection)
        detection.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
