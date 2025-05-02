# ROS2 Monodepth

This ROS2 project implements monocular depth estimation using the ZoeDepth model. It provides real-time absolute depth estimation from single RGB images.

## Python node

### Installation

1. Install project dependencies:

```bash
rosdep install -i --from-path src --rosdistro humble -y --ignore-src
```

2. Build the workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Usage

1. Start the webcam publisher node:

```bash
ros2 run zoedepth webcam_publisher --ros-args -p device_id:=0
```

2. In another terminal, start the depth estimator node:

```bash
ros2 run zoedepth depth_estimator --ros-args -p compiler_backend:='aot_eager'
```

### Configuration

Both nodes support various parameters that can be set via the command line:

Webcam Publisher Parameters:

- `device_id` (default: 0): Webcam device ID or path
- `target_width` (default: 640): Target width for resizing
- `target_height` (default: 480): Target height for resizing
- `force_square_crop` (default: false): Force square output by cropping to shortest dimension before resizing
- `publish_rate` (default: 15.0): Publishing rate in Hz

Depth Estimator Parameters:

- `model_repo` (default: 'isl-org/ZoeDepth'): Model repository
- `model_name` (default: 'ZoeD_NK'): Model name (ZoeD_N, ZoeD_K, or ZoeD_NK)
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

- Average latency of ~120ms per frame at 8Hz with no compiler backend
- Similar performance with Inductor or TensorRT backend in initial tests, so no visible acceleration
- Testing on NVIDIA Jetson Orin platforms is planned

These numbers are preliminary and may vary based on your specific hardware configuration and input resolution.

## C++ node

### Network optimisation

The ZoeDepth network is a PyTorch program in Python. For faster inference, we export the program to ONNX following these steps:

1. Export to ONNX from Pytorch
   1. ZoeD_K using KITTI dataset validation data (image domain for "outside")
   1. ZoeD_N using NYU dataset validation data (image domain for "inside")
   1. ZoeD_NK using both
1. Optimise for TensorRT inference using ONNX runtime
1. Load and execute the ONNX model in a C++ program
   1. Use DLA
   1. Quantization
1. Create a composable node to associate with depth_image_proc

### Local setup (unmainted)

This is my setup to obtain the optimised network

- NVIDIA Driver Version: 570.124.06
- CUDA 12.8
- cuDNN 9
- TensorRT 10.10
- Pytorch 2.5
- ONNX runtime 1.20

```bash
# Starting from CUDA 12.x (check with nvidia-smi)
sudo apt update
sudo apt install nvidia-cuda-toolkit python3-pip python3-venv
# cuDNN 9.6
wget https://developer.download.nvidia.com/compute/cuda/repos/<distro>/<arch>/cuda-keyring_1.1-1_all.deb
# wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install cudnn
# TensorRT 10.5 (Python API only can be entirely installed in pip)
wget https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/10.5.0/local_repo/nv-tensorrt-local-repo-ubuntu2204-10.5.0-cuda-12.6_1.0-1_amd64.deb
sudo dpkg -i nv-tensorrt-local-repo-ubuntu2204-10.5.0-cuda-12.6_1.0-1_amd64.deb
sudo cp /var/nv-tensorrt-local-repo-ubuntu2204-10.5.0-cuda-12.6/*-keyring.gpg /usr/share/keyrings/
sudo apt update
sudo apt-get install tensorrt
# Python dependencies
python3 -m venv .env
source .env/bin/activate
pip3 intall -r requirements.txt
```

Though it seems like everything can be installed through pip ??

### Docker setup (maintained)

```bash
docker build -t zoedepth --target rosdep-pip .
docker run --rm -it zoedepth
```

## Topics

- `/image_raw` (sensor_msgs/Image): Raw RGB images from webcam
- `/depth/image_raw` (sensor_msgs/Image): Estimated depth maps

## License

This project is licensed under the MIT License - see the LICENSE file for details.
