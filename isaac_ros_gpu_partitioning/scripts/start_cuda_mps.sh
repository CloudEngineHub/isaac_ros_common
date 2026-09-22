#!/bin/sh
export CUDA_MPS_PIPE_DIRECTORY=/tmp/nvidia-mps
export CUDA_MPS_LOG_DIRECTORY=/tmp/nvidia-mps
sudo -E nvidia-cuda-mps-control -s -d -q
