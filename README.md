# ROS2 Monodepth

**Note: This project was generated with the assistance of artificial intelligence.**

This ROS2 project implements monocular depth estimation using the ZoeDepth model. It provides real-time depth estimation from single RGB images.

## Installation

1. Install dependencies:
```bash
rosdep install -i --from-path src --rosdistro humble -y --ignore-src
```

2. Build the workspace:
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage

1. Start the webcam publisher node:
```bash
ros2 run zoedepth webcam_publisher
```

2. In another terminal, start the depth estimator node:
```bash
ros2 run zoedepth depth_estimator
```

### Configuration

Both nodes support various parameters that can be set via the command line:

Webcam Publisher Parameters:
- `device_id` (default: 0): Webcam device ID or path
- `target_width` (default: 256): Target width for resizing
- `target_height` (default: 256): Target height for resizing
- `force_square_crop` (default: false): Force square output by cropping to shortest dimension before resizing
- `publish_rate` (default: 15.0): Publishing rate in Hz

Depth Estimator Parameters:
- `model_repo` (default: 'isl-org/ZoeDepth'): Model repository
- `model_type` (default: 'NK'): Model type (N, K, or NK)
- `normalize_depth` (default: false): Whether to normalize depth output to 0-255 range
- `colorize_output` (default: false): Whether to apply colorization to the depth map using magma colormap
- `measure_latency` (default: false): Whether to measure and log processing latency
- `use_compiler` (default: true): Whether to use PyTorch's compiler
- `compiler_backend` (default: 'inductor'): Compiler backend to use. Options:
  - 'inductor': Default PyTorch 2.0 compiler
  - 'eager': Traditional PyTorch eager execution
  - 'aot_eager': Ahead-of-time compilation with eager execution
  - 'tensorrt': TensorRT acceleration (requires torch-tensorrt package)

Note: To use the TensorRT backend, you must first install the torch-tensorrt package:
```bash
pip3 install torch-tensorrt
```

### Performance

Performance measurements were conducted on an NVIDIA RTX 4070 GPU. Initial testing shows:
- Average latency of ~120ms per frame with default inductor backend
- Similar performance with TensorRT backend in initial tests
- Testing on NVIDIA Jetson Orin platforms is planned

These numbers are preliminary and may vary based on your specific hardware configuration and input resolution.

Example:
```bash
ros2 run zoedepth webcam_publisher --ros-args -p device_id:=1 -p publish_rate:=15.0
```

## Topics

- `/image_raw` (sensor_msgs/Image): Raw RGB images from webcam
- `/depth/image_raw` (sensor_msgs/Image): Estimated depth maps

## License

This project is licensed under the MIT License - see the LICENSE file for details.
