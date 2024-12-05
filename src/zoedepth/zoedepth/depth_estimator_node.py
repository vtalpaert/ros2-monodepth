#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

from zoedepth.models.builder import build_model
from zoedepth.utils.config import get_config
from zoedepth.utils.misc import colorize

class DepthEstimatorNode(Node):
    """ROS2 node for monocular depth estimation using ZoeDepth."""

    def __init__(self):
        super().__init__("depth_estimator_node")

        # Declare parameters
        self.declare_parameter("model_repo", "isl-org/ZoeDepth")
        self.declare_parameter("model_type", "NK")  # Options: N, K, NK
        self.declare_parameter("normalize_depth", False)
        self.declare_parameter("colorize_output", True)

        # Get parameters
        model_repo = self.get_parameter("model_repo").value
        model_type = self.get_parameter("model_type").value

        # Validate model type
        if model_type not in ["N", "K", "NK"]:
            raise ValueError("model_type must be one of: N, K, NK")

        self.get_logger().info(
            f"Loading ZoeDepth model type {model_type} from {model_repo}..."
        )

        # Setup device
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        try:
            # Model configuration
            model_configs = {
                "N": {
                    "name": "ZoeD_N",
                    "weights": "https://github.com/isl-org/ZoeDepth/releases/download/v1.0/ZoeD_M12_N.pt"
                },
                "K": {
                    "name": "ZoeD_K",
                    "weights": "https://github.com/isl-org/ZoeDepth/releases/download/v1.0/ZoeD_M12_K.pt"
                },
                "NK": {
                    "name": "ZoeD_NK",
                    "weights": "https://github.com/isl-org/ZoeDepth/releases/download/v1.0/ZoeD_M12_NK.pt"
                }
            }

            config = model_configs[model_type]
            
            # Load model
            model = torch.hub.load(model_repo, config["name"], pretrained=False)
            pretrained_dict = torch.hub.load_state_dict_from_url(
                config["weights"],
                map_location=self.device,
            )
            model.load_state_dict(pretrained_dict["model"], strict=False)
            
            # Apply Identity to drop_path
            for b in model.core.core.pretrained.model.blocks:
                b.drop_path = torch.nn.Identity()

            self.model = model.to(self.device)
            self.get_logger().info("Model loaded successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {str(e)}")
            raise

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            Image, "image_raw", self.image_callback, 10
        )
        self.publisher = self.create_publisher(Image, "depth/image_raw", 10)

    def image_callback(self, msg):
        """Process incoming image and publish depth estimation."""
        try:
            # Convert ROS Image to CV2
            pil_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

            # Infer depth
            depth_numpy = self.model.infer_pil(pil_image)

            # Process depth output
            if self.get_parameter("colorize_output").value:
                depth_output = colorize(depth_numpy)
                encoding = "bgr8"
            elif self.get_parameter("normalize_depth").value:
                # Normalize depth for visualization (0-255)
                depth_output = (
                    (depth_numpy - depth_numpy.min())
                    * (255 / (depth_numpy.max() - depth_numpy.min()))
                ).astype(np.uint8)
                encoding = "mono8"
            else:
                depth_output = depth_numpy
                encoding = "32FC1"

            # Convert depth map to ROS Image message
            depth_msg = self.bridge.cv2_to_imgmsg(depth_output, encoding=encoding)
            depth_msg.header = msg.header

            # Publish depth image
            self.publisher.publish(depth_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
