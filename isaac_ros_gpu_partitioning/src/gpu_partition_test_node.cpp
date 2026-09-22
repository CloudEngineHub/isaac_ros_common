// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include "gpu_partition_test_node.hpp"

#include <cuda.h>
#include <cuda_runtime.h>

#include <iostream>
#include <memory>
#include <set>
#include <vector>

#define CUDA_CHECK_ERROR(expr) \
  do { \
    CUresult result = (expr); \
    if(CUDA_SUCCESS != result) { \
      std::cout << "cuda API [{}:{}] failed with {} " << __FILE__ << ":" << __LINE__ << ":" << \
        result << std::endl; \
    } \
    assert(CUDA_SUCCESS == result); \
  } while(0)

#define CUDA_CHECK_RESULT(expr) \
  do { \
    cudaError_t result = (expr); \
    if(CUDA_SUCCESS != static_cast<int>(result)) { \
      std::cout << "cuda API [{}:{}] failed with {} " << __FILE__ << ":" << __LINE__ << ":" << \
        result << std::endl; \
    } \
  } while(0)

typedef struct
{
  int total_sm_count;
  int mps_enabled;
} GpuSmInfo;

int get_gpu_sm_info(int device_index, GpuSmInfo & gpu_sm_info)
{
  CUresult result;

  int runtimeVersion;
  cudaError_t err = cudaRuntimeGetVersion(&runtimeVersion);
  if (err == cudaSuccess) {
    printf("CUDA Runtime Version: %d.%d\n", runtimeVersion / 1000, (runtimeVersion % 100) / 10);
  } else {
    fprintf(stderr, "Error getting CUDA Runtime version: %s\n", cudaGetErrorString(err));
    return 1;
  }

  result = cuInit(0);
  if (result != CUDA_SUCCESS) {
    std::cout << "cuInit failed with error code " << result << std::endl;
    return 1;
  }
  // Get the number of visible devices
  int deviceCount;
  result = cuDeviceGetCount(&deviceCount);
  if (result != CUDA_SUCCESS) {
    std::cerr << "cuDeviceGetCount failed with error code: " << result << std::endl;
    return 1;
  }

  std::cout << "Device Count: " << deviceCount << std::endl;
  if (deviceCount == 0) {
    std::cerr << "No CUDA devices found" << std::endl;
    return 1;
  }
  // 2. Get a handle to the first available CUDA device
  CUdevice device;
  result = cuDeviceGet(&device, device_index);
  if (result != CUDA_SUCCESS) {
    std::cout << "cuDeviceGet failed with error code " << result << std::endl;
    return 1;
  }

  // Check if MPS is enabled
  int mps_enabled = 0;
  result = cuDeviceGetAttribute(&mps_enabled, CU_DEVICE_ATTRIBUTE_MPS_ENABLED, device);
  if (result != CUDA_SUCCESS) {
    std::cout << "cuDeviceGetAttribute failed with error code " << result << std::endl;
    return 1;
  }
  gpu_sm_info.mps_enabled = mps_enabled;

  int total_sm_count = 0;
  result = cuDeviceGetAttribute(
    &total_sm_count, CU_DEVICE_ATTRIBUTE_MULTIPROCESSOR_COUNT, device);
  if (result != CUDA_SUCCESS) {
    std::cout << "cuDeviceGetAttribute(MULTIPROCESSOR_COUNT) failed with error code "
              << result << std::endl;
    return 1;
  }
  printf("Detected device has %d streaming multiprocessors (SMs).\n", total_sm_count);
  gpu_sm_info.total_sm_count = total_sm_count;

  return 0;
}

namespace nvidia
{

GpuPartitionTestNode::GpuPartitionTestNode(const rclcpp::NodeOptions options)
: Node("gpu_partition_test_node", options)
{
  RCLCPP_INFO(get_logger(), "[GpuPartitionTestNode] In GpuPartitionTestNode's constructor");

  // print the current pid
  pid_t pid = getpid();

  GpuSmInfo gpu_sm_info = {0, 0};
  get_gpu_sm_info(0, gpu_sm_info);
  RCLCPP_INFO(get_logger(), "[PID: %d] Total SM count: %d", pid, gpu_sm_info.total_sm_count);
  RCLCPP_INFO(get_logger(), "[PID: %d] MPS enabled: %d", pid, gpu_sm_info.mps_enabled);
  RCLCPP_INFO(get_logger(), "[PID: %d] End of GPU Partition Info", pid);

  // Create a message to publish the SM count
  auto message = std::make_shared<std_msgs::msg::Int32MultiArray>();
  message->data.clear();
  message->data.push_back(pid);
  message->data.push_back(gpu_sm_info.mps_enabled);
  message->data.push_back(gpu_sm_info.total_sm_count);

  publisher_ = create_publisher<std_msgs::msg::Int32MultiArray>(
    "gpu_partition_info", rclcpp::QoS(10).transient_local());
  if (!publisher_) {
    RCLCPP_ERROR(get_logger(), "Failed to create publisher");
    return;
  }
  publisher_->publish(*message);
}

GpuPartitionTestNode::~GpuPartitionTestNode() {}

}  // namespace nvidia

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::GpuPartitionTestNode)
