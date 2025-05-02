
ARG IMAGE_NAME_CUDA=nvidia/cuda
ARG IMAGE_TAG_CUDACUDNN=12.8.1-cudnn-runtime-ubuntu22.04
ARG PLATFORM=amd64
FROM --platform=${PLATFORM} ${IMAGE_NAME_CUDA}:${IMAGE_TAG_CUDACUDNN} AS base

ARG DEBIAN_FRONTEND=noninteractive
ARG USER_NAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN groupadd --gid ${USER_GID} ${USER_NAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USER_NAME} \
    && apt update \
    && apt install -y curl locales python3-venv sudo \
    && rm -rf /var/lib/apt/lists/* \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && echo ${USER_NAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USER_NAME} \
    && chmod 0440 /etc/sudoers.d/${USER_NAME}
ENV LANG=en_US.UTF-8

FROM base AS tensorrt

# Version information at https://developer.nvidia.com/tensorrt/download
ARG OS_CODE="ubuntu2204"
ARG TENSORRT_VERSION="10.10.0"
ARG TENSORTRT_TAG="10.10.0-cuda-12.9"

RUN wget https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/${TENSORRT_VERSION}/local_repo/nv-tensorrt-local-repo-${OS_CODE}-${TENSORTRT_TAG}_1.0-1_${PLATFORM}.deb \
    && sudo dpkg -i nv-tensorrt-local-repo-${OS_CODE}-${TENSORTRT_TAG}_1.0-1_amd64.deb \
    && sudo cp /var/nv-tensorrt-local-repo-${OS_CODE}-${TENSORTRT_TAG}/*-keyring.gpg /usr/share/keyrings/ \
    && sudo apt update \
    && sudo apt-get install -y tensorrt \
    && rm -rf /var/lib/apt/lists/*

# Use base here if tensorrt python version is sufficient, otherwise use FROM tensorrt
FROM base AS ros-install

ARG ROS_DISTRO=humble
RUN apt update \
    && apt install -y software-properties-common \
    && add-apt-repository universe \
    && apt update \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt update \
    && apt upgrade -y \
    && apt install -y ros-${ROS_DISTRO}-desktop python3-rosdep python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

FROM ros-install AS rosdep-pip

USER ${USER_NAME}
SHELL ["/bin/bash", "-c"]
WORKDIR /home/${USER_NAME}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && sudo rosdep init \
    && rosdep update \
    && python3 -m venv .env

# The requirements.txt should be simplified if the package only relies on pytorch or requires onnx
COPY requirements.txt requirements.txt
RUN source .env/bin/activate \
    && pip3 install -r requirements.txt \
    && rm requirements.txt

#COPY src src
#RUN sudo apt update \
#    && source /opt/ros/${ROS_DISTRO}/setup.bash \
#    && source .env/bin/activate \
#    && rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} -y --ignore-src \
#    && rm -rf /var/lib/apt/lists/*

FROM rosdep-pip AS build

COPY src src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source .env/bin/activate \
    && colcon build --packages-up-to zoedepth \
    && rm -rf build log
