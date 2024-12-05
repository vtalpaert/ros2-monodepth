#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import torch


class DepthEstimatorNode(Node):
    """ROS2 node for monocular depth estimation using ZoeDepth."""

    def __init__(self):
        super().__init__('depth_estimator_node')
        
        # Declare parameters
        self.declare_parameter('model_repo', 'isl-org/ZoeDepth')
        self.declare_parameter('model_name', 'ZoeD_NK')
        
        # Get parameters
        model_repo = self.get_parameter('model_repo').value
        model_name = self.get_parameter('model_name').value
        
        self.get_logger().info(f'Loading ZoeDepth model {model_name} from {model_repo}...')
        
        try:
            self.model = torch.hub.load(model_repo, model_name, pretrained=True)
            self.get_logger().info('Model loaded successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')
            raise

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
