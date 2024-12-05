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
        self.declare_parameter('device_id', 3)
        self.declare_parameter('target_width', 256)  # Target width for resizing
        self.declare_parameter('target_height', 256)  # Target height for resizing
        self.declare_parameter('force_square_crop', False)  # Force square output by cropping
        self.declare_parameter('publish_rate', 15.0)  # Hz
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Setup webcam
        device_id = self.get_parameter('device_id').value
        
        # Try different device paths
        device_paths = [
            device_id,  # Try numeric index first
            f"/dev/video{device_id}",  # Try explicit device path
            f"/dev/v4l/by-id/*{device_id}*",  # Try video4linux by-id path
        ]
        
        for device in device_paths:
            self.get_logger().info(f'Attempting to open camera device: {device}')
            self.cap = cv2.VideoCapture(device)
            if self.cap.isOpened():
                self.get_logger().info(f'Successfully opened camera device: {device}')
                break
        
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open any camera device')
            self.get_logger().error('Available video devices:')
            import glob
            devices = glob.glob('/dev/video*')
            for dev in devices:
                self.get_logger().error(f'  - {dev}')
            raise RuntimeError('Could not open any camera device')
        
        # Create publisher
        self.publisher = self.create_publisher(Image, 'image_raw', 10)
        
        # Create timer for publishing
        period = 1.0 / self.get_parameter('publish_rate').value
        self.timer = self.create_timer(period, self.timer_callback)
        
    def timer_callback(self):
        """Capture and publish webcam frame."""
        ret, frame = self.cap.read()
        if ret:
            # Resize while maintaining aspect ratio if requested
            target_width = self.get_parameter('target_width').value
            target_height = self.get_parameter('target_height').value
            
            if target_width > 0 and target_height > 0:
                h, w = frame.shape[:2]
                force_square = self.get_parameter('force_square_crop').value
                
                if force_square:
                    # Determine the square size (minimum of target dimensions)
                    square_size = min(target_width, target_height)
                    
                    # Calculate center crop to square
                    if w > h:
                        start_x = (w - h) // 2
                        start_y = 0
                        frame = frame[:, start_x:start_x + h]
                    else:
                        start_x = 0
                        start_y = (h - w) // 2
                        frame = frame[start_y:start_y + w, :]
                    
                    # Resize the square crop to target size
                    frame = cv2.resize(frame, (square_size, square_size))
                else:
                    # Calculate target size maintaining aspect ratio
                    aspect = w / h
                    if aspect > target_width / target_height:  # Width limited
                        new_width = target_width
                        new_height = int(target_width / aspect)
                    else:  # Height limited
                        new_height = target_height
                        new_width = int(target_height * aspect)
                    
                    # Resize to fit within target dimensions
                    frame = cv2.resize(frame, (new_width, new_height))
                    
                    # Center crop to exact target size if needed
                    if new_width != target_width or new_height != target_height:
                        start_x = max(0, new_width//2 - target_width//2)
                        start_y = max(0, new_height//2 - target_height//2)
                        frame = frame[start_y:start_y+target_height, 
                                    start_x:start_x+target_width]
            
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
