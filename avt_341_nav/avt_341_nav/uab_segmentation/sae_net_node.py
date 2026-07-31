#!/usr/bin/env python3
"""
Minimal ROS 2 node for running sae_net Pytorch segmentation network
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import torch
import yaml

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from avt_341_nav.uab_perception_py_parameters import avt_341_nav_params_uab_perception_py
from avt_341_nav.uab_segmentation.matlab_resize import imresize_bicubic, imresize_nearest
from avt_341_nav.uab_segmentation.sae_net import SaeNet

# Model weights and export metadata install location below the package share directory
WEIGHTS_SHARE_SUBDIR = "ml_weights/uab_segmentation"

# Blend factor of the segmentation mask in the overlay image
OVERLAY_ALPHA = 0.3


class SaeNetNode(Node):
    def __init__(self):
        super().__init__("sae_segmentation")

        self.param_listener = avt_341_nav_params_uab_perception_py.ParamsListener(self)
        params = self.param_listener.get_params()

        self.brightness = float(params.brightness_offset)
        self.use_matlab_resize = bool(params.use_matlab_resize)

        device = str(params.device)
        if device.startswith("cuda") and not torch.cuda.is_available():
            self.get_logger().warning("device 'cuda' requested but CUDA is not available "
                                      "in this torch build -- falling back to 'cpu'")
            device = "cpu"
        self.device = torch.device(device)

        weights_path = Path(get_package_share_directory("avt_341_nav")) / WEIGHTS_SHARE_SUBDIR / str(params.weights_file)

        # Class colormap, class count and network input size from the export metadata next to the weights.
        meta_path = weights_path.with_name(f"{weights_path.stem}_meta.yaml")
        meta = yaml.safe_load(meta_path.read_text())
        self.label_lut = np.asarray(meta["labelIDs"], dtype=np.uint8).reshape(-1, 3)
        self.network_hw = tuple(meta["inputSize"][:2])

        self.model = SaeNet(num_classes=self.label_lut.shape[0])
        self.model.load_state_dict(torch.load(str(weights_path), map_location="cpu"))
        self.model.to(self.device).eval()

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, "avt_341/camera/segmentation", 1)
        self.overlay_pub = (self.create_publisher(Image, "avt_341/camera/segmentation_overlay", 1)
                            if bool(params.publish_overlay) else None)
        self.sub = self.create_subscription(Image, "avt_341/camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f"ready: weights={weights_path}, device={self.device}, "f"brightness=+{self.brightness:g}")

    def resize_bicubic(self, img: np.ndarray, out_hw) -> np.ndarray:
        if self.use_matlab_resize:
            return imresize_bicubic(img, out_hw)
        return cv2.resize(img, (int(out_hw[1]), int(out_hw[0])), interpolation=cv2.INTER_CUBIC)

    def resize_nearest(self, img: np.ndarray, out_hw) -> np.ndarray:
        if self.use_matlab_resize:
            return imresize_nearest(img, out_hw)
        return cv2.resize(img, (int(out_hw[1]), int(out_hw[0])), interpolation=cv2.INTER_NEAREST)

    @torch.no_grad()
    def on_image(self, msg: Image) -> None:

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except CvBridgeError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
            return

        resized = self.resize_bicubic(img, self.network_hw)
        resized = np.clip(resized.astype(np.int16) + int(round(self.brightness)), 0, 255).astype(np.uint8)
        x = torch.from_numpy(resized.astype(np.float32)).permute(2, 0, 1).unsqueeze(0)
        labels = self.model(x.to(self.device)).argmax(dim=1)[0].cpu().numpy()
        mask = self.label_lut[labels]

        mask = self.resize_nearest(mask, (msg.height, msg.width))
        out = self.bridge.cv2_to_imgmsg(mask, encoding="rgb8")
        out.header = msg.header
        self.pub.publish(out)

        if self.overlay_pub is not None:
            brightened = np.clip(img.astype(np.int16) + int(round(self.brightness)), 0, 255).astype(np.uint8)
            a = OVERLAY_ALPHA
            overlay = ((1 - a) * brightened.astype(np.float64) + a * mask.astype(np.float64)).astype(np.uint8)
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="rgb8")
            overlay_msg.header = msg.header
            self.overlay_pub.publish(overlay_msg)


def main() -> None:
    rclpy.init()
    node = SaeNetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
