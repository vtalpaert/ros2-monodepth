#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class WebcamPublisherNode(Node):
    """ROS2 node for publishing webcam frames as ROS Images."""

    def __init__(self):
        super().__init__('webcam_publisher_node')
        
        # Declare parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('crop_width', -1)  # -1 means no cropping
        self.declare_parameter('crop_height', -1)
        self.declare_parameter('publish_rate', 30.0)  # Hz
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Setup webcam
        device_id = self.get_parameter('device_id').value
        self.cap = cv2.VideoCapture(device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open webcam device {device_id}')
            raise RuntimeError(f'Could not open webcam device {device_id}')
            
        self.get_logger().info(f'Successfully opened webcam device {device_id}')
        
        # Create publisher
        self.publisher = self.create_publisher(Image, 'image_raw', 10)
        
        # Create timer for publishing
        period = 1.0 / self.get_parameter('publish_rate').value
        self.timer = self.create_timer(period, self.timer_callback)
        
    def timer_callback(self):
        """Capture and publish webcam frame."""
        ret, frame = self.cap.read()
        if ret:
            # Crop if requested
            crop_width = self.get_parameter('crop_width').value
            crop_height = self.get_parameter('crop_height').value
            
            if crop_width > 0 and crop_height > 0:
                h, w = frame.shape[:2]
                start_x = max(0, w//2 - crop_width//2)
                start_y = max(0, h//2 - crop_height//2)
                frame = frame[start_y:start_y+crop_height, 
                            start_x:start_x+crop_width]
            
            # Convert to ROS Image and publish
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing image: {str(e)}')
        else:
            self.get_logger().warn('Failed to capture webcam frame')
    
    def destroy_node(self):
        """Release webcam when node is destroyed."""
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
