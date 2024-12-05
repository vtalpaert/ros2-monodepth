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
- `device_id` (default: 0): Webcam device ID
- `crop_width` (default: -1): Width to crop image to (-1 for no cropping)
- `crop_height` (default: -1): Height to crop image to (-1 for no cropping)
- `publish_rate` (default: 30.0): Publishing rate in Hz

Depth Estimator Parameters:
- `model_repo` (default: 'isl-org/ZoeDepth'): Model repository
- `model_type` (default: 'NK'): Model type (N, K, or NK)
- `normalize_depth` (default: false): Whether to normalize depth output
- `colorize_output` (default: false): Whether to apply colorization to the depth map

Example:
```bash
ros2 run zoedepth webcam_publisher --ros-args -p device_id:=1 -p publish_rate:=15.0
```

## Topics

- `/image_raw` (sensor_msgs/Image): Raw RGB images from webcam
- `/depth/image_raw` (sensor_msgs/Image): Estimated depth maps

## License

This project is licensed under the MIT License - see the LICENSE file for details.
