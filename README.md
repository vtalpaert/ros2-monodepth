# ROS2 Monodepth

**Note: This project was generated with the assistance of artificial intelligence.**

This ROS2 project implements monocular depth estimation using the ZoeDepth model. It provides real-time depth estimation from single RGB images.

## Prerequisites

- ROS2 Humble
- Python 3.8+
- CUDA-capable GPU (recommended)
- OpenCV
- PyTorch
- sensor_msgs
- cv_bridge

## Installation

1. Create a ROS2 workspace if you don't have one:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone this repository into the src directory:
```bash
git clone https://github.com/vtalpaert/ros2_monodepth.git
```

3. Install dependencies:
```bash
sudo apt-get update
sudo apt-get install python3-opencv ros-humble-cv-bridge
pip3 install torch torchvision
```

4. Build the workspace:
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
- `device_id` (default: 0): Webcam device ID
- `crop_width` (default: -1): Width to crop image to (-1 for no cropping)
- `crop_height` (default: -1): Height to crop image to (-1 for no cropping)
- `publish_rate` (default: 30.0): Publishing rate in Hz

Depth Estimator Parameters:
- `model_repo` (default: 'isl-org/ZoeDepth'): Model repository
- `model_name` (default: 'ZoeD_NK'): Model name
- `normalize_depth` (default: true): Whether to normalize depth output

Example:
```bash
ros2 run zoedepth webcam_publisher --ros-args -p device_id:=1 -p publish_rate:=15.0
```

## Topics

- `/image_raw` (sensor_msgs/Image): Raw RGB images from webcam
- `/depth/image_raw` (sensor_msgs/Image): Estimated depth maps

## License

This project is licensed under the MIT License - see the LICENSE file for details.
