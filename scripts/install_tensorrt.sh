#!/usr/bin/env bash

wget https://developer.nvidia.com/downloads/compute/machine-learning/tensorrt/10.5.0/local_repo/nv-tensorrt-local-repo-ubuntu2204-10.5.0-cuda-12.6_1.0-1_amd64.deb
sudo dpkg -i nv-tensorrt-local-repo-ubuntu2204-10.5.0-cuda-12.6_1.0-1_amd64.deb
sudo cp /var/nv-tensorrt-local-repo-ubuntu2204-10.5.0-cuda-12.6/*-keyring.gpg /usr/share/keyrings/
sudo apt update
sudo apt-get install -y tensorrt
