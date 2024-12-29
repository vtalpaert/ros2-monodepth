#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import torch
from sensor_msgs.msg import Image
from collections import deque
from time import time

# Check for TensorRT availability
# Note: TensorRT backend requires manual installation via: pip3 install torch-tensorrt
TENSORRT_AVAILABLE = False
try:
    import torch_tensorrt

    TENSORRT_AVAILABLE = True
except ImportError:
    pass
from cv_bridge import CvBridge
import numpy as np
import cv2

from zoedepth.utils.misc import colorize
from zoedepth.zoedepth_loader import zoedepth_loader

from zoedepth.export_torch_to_onnx import create_session, predict


import numpy as np
import sys

name_to_dtypes = {
    "rgb8": (np.uint8, 3),
    "rgba8": (np.uint8, 4),
    "rgb16": (np.uint16, 3),
    "rgba16": (np.uint16, 4),
    "bgr8": (np.uint8, 3),
    "bgra8": (np.uint8, 4),
    "bgr16": (np.uint16, 3),
    "bgra16": (np.uint16, 4),
    "mono8": (np.uint8, 1),
    "mono16": (np.uint16, 1),
    # for bayer image (based on cv_bridge.cpp)
    "bayer_rggb8": (np.uint8, 1),
    "bayer_bggr8": (np.uint8, 1),
    "bayer_gbrg8": (np.uint8, 1),
    "bayer_grbg8": (np.uint8, 1),
    "bayer_rggb16": (np.uint16, 1),
    "bayer_bggr16": (np.uint16, 1),
    "bayer_gbrg16": (np.uint16, 1),
    "bayer_grbg16": (np.uint16, 1),
    # OpenCV CvMat types
    "8UC1": (np.uint8, 1),
    "8UC2": (np.uint8, 2),
    "8UC3": (np.uint8, 3),
    "8UC4": (np.uint8, 4),
    "8SC1": (np.int8, 1),
    "8SC2": (np.int8, 2),
    "8SC3": (np.int8, 3),
    "8SC4": (np.int8, 4),
    "16UC1": (np.uint16, 1),
    "16UC2": (np.uint16, 2),
    "16UC3": (np.uint16, 3),
    "16UC4": (np.uint16, 4),
    "16SC1": (np.int16, 1),
    "16SC2": (np.int16, 2),
    "16SC3": (np.int16, 3),
    "16SC4": (np.int16, 4),
    "32SC1": (np.int32, 1),
    "32SC2": (np.int32, 2),
    "32SC3": (np.int32, 3),
    "32SC4": (np.int32, 4),
    "32FC1": (np.float32, 1),
    "32FC2": (np.float32, 2),
    "32FC3": (np.float32, 3),
    "32FC4": (np.float32, 4),
    "64FC1": (np.float64, 1),
    "64FC2": (np.float64, 2),
    "64FC3": (np.float64, 3),
    "64FC4": (np.float64, 4),
}


def image_to_numpy(msg):
    if not msg.encoding in name_to_dtypes:
        raise TypeError("Unrecognized encoding {}".format(msg.encoding))

    dtype_class, channels = name_to_dtypes[msg.encoding]
    dtype = np.dtype(dtype_class)
    dtype = dtype.newbyteorder(">" if msg.is_bigendian else "<")
    shape = (msg.height, msg.width, channels)

    data = np.fromstring(msg.data, dtype=dtype).reshape(shape)
    data.strides = (msg.step, dtype.itemsize * channels, dtype.itemsize)

    if channels == 1:
        data = data[..., 0]
    return data


def numpy_to_image(arr, encoding):
    if not encoding in name_to_dtypes:
        raise TypeError("Unrecognized encoding {}".format(encoding))

    im = Image(encoding=encoding)

    # extract width, height, and channels
    dtype_class, exp_channels = name_to_dtypes[encoding]
    dtype = np.dtype(dtype_class)
    if len(arr.shape) == 2:
        im.height, im.width, channels = arr.shape + (1,)
    elif len(arr.shape) == 3:
        im.height, im.width, channels = arr.shape
    else:
        raise TypeError("Array must be two or three dimensional")

    # check type and channels
    if exp_channels != channels:
        raise TypeError(
            "Array has {} channels, {} requires {}".format(
                channels, encoding, exp_channels
            )
        )
    if dtype_class != arr.dtype.type:
        raise TypeError(
            "Array is {}, {} requires {}".format(arr.dtype.type, encoding, dtype_class)
        )

    # make the array contiguous in memory, as mostly required by the format
    contig = np.ascontiguousarray(arr)
    im.data = contig.tostring()
    im.step = contig.strides[0]
    im.is_bigendian = (
        arr.dtype.byteorder == ">"
        or arr.dtype.byteorder == "="
        and sys.byteorder == "big"
    )

    return im


class DepthEstimatorNode(Node):
    """ROS2 node for monocular depth estimation using ZoeDepth."""

    def __init__(self):
        super().__init__("depth_estimator_node")

        # Declare parameters
        self.declare_parameter("model_repo", "isl-org/ZoeDepth")
        self.declare_parameter("model_type", "NK")  # Options: N, K, NK
        self.declare_parameter("normalize_depth", False)
        self.declare_parameter("colorize_output", False)
        self.declare_parameter("measure_latency", False)
        self.declare_parameter("use_compiler", False)
        self.declare_parameter(
            "compiler_backend", "inductor"
        )  # Options: inductor, eager, aot_eager

        # Initialize latency tracking
        self.latency_window = deque(maxlen=100)  # Track last 100 measurements

        # Get parameters
        model_repo = self.get_parameter("model_repo").value
        model_type = self.get_parameter("model_type").value

        self.get_logger().info(f"Loading model type {model_type} from {model_repo}...")

        # Setup device
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.get_logger().info(f"Using device: {self.device}")

        """
        try:
            model = zoedepth_loader(model_repo, model_type, self.device)
            model = model.to(self.device)

            # Apply model compilation if enabled
            if self.get_parameter("use_compiler").value:
                backend = self.get_parameter("compiler_backend").value
                if backend == "tensorrt" and not TENSORRT_AVAILABLE:
                    raise ValueError(
                        "TensorRT backend requested but torch_tensorrt is not available"
                    )
                self.get_logger().info(f"Compiling model with backend: {backend}")
                model = torch.compile(model, backend=backend)

            self.get_logger().info("Model loaded successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {str(e)}")
            raise

        # self.model = model
        """

        self.session = create_session("ZoeD_K.onnx")

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
            start_time = time() if self.get_parameter("measure_latency").value else None
            # Convert ROS Image to CV2
            # pil_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            # Infer depth
            # depth_numpy = self.model.infer_pil(pil_image)

            # input_tensor = transforms.ToTensor()(pil_image).unsqueeze(0)

            numpy_array = image_to_numpy(msg)
            input_tensor = torch.from_numpy(numpy_array).unsqueeze(0)

            out_tensor = predict(self.session, input_tensor)
            out_16bit_numpy = (out_tensor.squeeze().cpu().numpy() * 256).astype(
                np.uint16
            )
            depth_numpy = out_16bit_numpy

            # Process depth output
            if self.get_parameter("colorize_output").value:
                depth_output = colorize(depth_numpy, cmap="magma_r")
                # Convert RGBA to BGR
                depth_output = cv2.cvtColor(depth_output, cv2.COLOR_RGBA2BGR)
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
            # depth_msg = self.bridge.cv2_to_imgmsg(depth_output, encoding=encoding)
            depth_msg = numpy_to_image(depth_numpy, "32FC1")
            depth_msg.header = msg.header

            # Publish depth image
            self.publisher.publish(depth_msg)

            # Calculate and log latency if enabled
            if self.get_parameter("measure_latency").value:
                latency = time() - start_time
                self.latency_window.append(latency)
                mean_latency = sum(self.latency_window) / len(self.latency_window)
                self.get_logger().info(f"Processing latency: {mean_latency:.3f}s")

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
