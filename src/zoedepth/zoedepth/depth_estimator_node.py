#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2


class DepthEstimatorNode(Node):
    """ROS2 node for monocular depth estimation using ZoeDepth."""

    def __init__(self):
        super().__init__('depth_estimator_node')
        
        # Declare parameters
        self.declare_parameter('model_repo', 'isl-org/ZoeDepth')
        self.declare_parameter('model_name', 'ZoeD_NK')
        self.declare_parameter('normalize_depth', True)
        
        # Get parameters
        model_repo = self.get_parameter('model_repo').value
        model_name = self.get_parameter('model_name').value
        
        self.get_logger().info(f'Loading ZoeDepth model {model_name} from {model_repo}...')
        
        # Setup device
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f'Using device: {self.device}')
        
        try:
            self.model = torch.hub.load(model_repo, model_name, pretrained=True)
            self.model.to(self.device)
            self.get_logger().info('Model loaded successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')
            raise

        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(
            Image,
            'depth/image_raw',
            10)
            
    def image_callback(self, msg):
        """Process incoming image and publish depth estimation."""
        try:
            # Convert ROS Image to CV2
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            
            # Convert to PIL Image
            pil_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            
            # Infer depth
            depth_numpy = self.model.infer_pil(pil_image)
            
            # Get depth output
            if self.get_parameter('normalize_depth').value:
                # Normalize depth for visualization (0-255)
                depth_output = ((depth_numpy - depth_numpy.min()) * (255 / (depth_numpy.max() - depth_numpy.min()))).astype(np.uint8)
                encoding = 'mono8'
            else:
                depth_output = depth_numpy
                encoding = '32FC1'
            
            # Convert depth map to ROS Image message
            depth_msg = self.bridge.cv2_to_imgmsg(depth_output, encoding=encoding)
            depth_msg.header = msg.header
            
            # Publish depth image
            self.publisher.publish(depth_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

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

if __name__ == '__main__':
    main()