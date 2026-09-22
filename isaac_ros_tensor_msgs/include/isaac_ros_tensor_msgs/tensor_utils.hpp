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

#ifndef ISAAC_ROS_TENSOR_MSGS__TENSOR_UTILS_HPP_
#define ISAAC_ROS_TENSOR_MSGS__TENSOR_UTILS_HPP_

#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

#include "isaac_ros_tensor_msgs/msg/tensor_list.hpp"
#include "tensor_msgs/msg/experimental_tensor.hpp"

namespace isaac_ros_tensor_msgs
{

inline const tensor_msgs::msg::ExperimentalTensor * FindTensorByName(
  const msg::TensorList & tensor_list, const std::string & name)
{
  for (std::size_t i = 0;
    i < tensor_list.names.size() && i < tensor_list.tensors.size(); ++i)
  {
    if (tensor_list.names[i] == name) {
      return &tensor_list.tensors[i];
    }
  }
  return nullptr;
}

inline std::size_t StrideInElements(
  const tensor_msgs::msg::ExperimentalTensor & tensor, std::size_t dimension)
{
  if (dimension >= tensor.shape.size()) {
    throw std::out_of_range("Tensor stride index is out of range");
  }
  if (!tensor.strides.empty()) {
    if (tensor.strides.size() != tensor.shape.size() || tensor.strides[dimension] < 0) {
      throw std::invalid_argument("Tensor strides must match tensor rank and be nonnegative");
    }
    return static_cast<std::size_t>(tensor.strides[dimension]);
  }

  std::size_t stride = 1;
  for (std::size_t i = dimension + 1; i < tensor.shape.size(); ++i) {
    if (tensor.shape[i] <= 0) {
      throw std::invalid_argument("Tensor dimensions must be positive");
    }
    const std::size_t extent = static_cast<std::size_t>(tensor.shape[i]);
    if (stride > std::numeric_limits<std::size_t>::max() / extent) {
      throw std::overflow_error("Tensor stride overflow");
    }
    stride *= extent;
  }
  return stride;
}

inline std::size_t RequiredStorageElements(
  const tensor_msgs::msg::ExperimentalTensor & tensor)
{
  if (tensor.shape.empty()) {
    throw std::invalid_argument("Tensor shape is empty");
  }

  std::size_t max_offset = 0;
  std::size_t contiguous_stride = 1;
  for (std::size_t reverse_i = tensor.shape.size(); reverse_i > 0; --reverse_i) {
    const std::size_t i = reverse_i - 1;
    if (tensor.shape[i] <= 0) {
      throw std::invalid_argument("Tensor dimensions must be positive");
    }
    if (!tensor.strides.empty() && tensor.strides.size() != tensor.shape.size()) {
      throw std::invalid_argument("Tensor strides must match tensor rank");
    }
    if (!tensor.strides.empty() && tensor.strides[i] < 0) {
      throw std::invalid_argument("Tensor strides must be nonnegative");
    }

    const std::size_t extent = static_cast<std::size_t>(tensor.shape[i] - 1);
    const std::size_t stride = tensor.strides.empty() ?
      contiguous_stride : static_cast<std::size_t>(tensor.strides[i]);
    if (extent != 0 && stride > std::numeric_limits<std::size_t>::max() / extent) {
      throw std::overflow_error("Tensor storage size overflow");
    }
    const std::size_t contribution = extent * stride;
    if (max_offset > std::numeric_limits<std::size_t>::max() - contribution) {
      throw std::overflow_error("Tensor storage size overflow");
    }
    max_offset += contribution;

    const std::size_t dimension = static_cast<std::size_t>(tensor.shape[i]);
    if (i != 0) {
      if (contiguous_stride > std::numeric_limits<std::size_t>::max() / dimension) {
        throw std::overflow_error("Tensor stride overflow");
      }
      contiguous_stride *= dimension;
    }
  }
  if (max_offset == std::numeric_limits<std::size_t>::max()) {
    throw std::overflow_error("Tensor storage size overflow");
  }
  return max_offset + 1;
}

inline bool IsContiguousRowMajor(
  const tensor_msgs::msg::ExperimentalTensor & tensor)
{
  if (tensor.strides.empty()) {
    return true;
  }
  if (tensor.strides.size() != tensor.shape.size()) {
    return false;
  }

  std::size_t expected_stride = 1;
  for (std::size_t reverse_i = tensor.shape.size(); reverse_i > 0; --reverse_i) {
    const std::size_t i = reverse_i - 1;
    if (tensor.shape[i] <= 0 || tensor.strides[i] < 0 ||
      static_cast<std::size_t>(tensor.strides[i]) != expected_stride)
    {
      return false;
    }
    if (i != 0) {
      const std::size_t dimension = static_cast<std::size_t>(tensor.shape[i]);
      if (expected_stride > std::numeric_limits<std::size_t>::max() / dimension) {
        return false;
      }
      expected_stride *= dimension;
    }
  }
  return true;
}

}  // namespace isaac_ros_tensor_msgs

#endif  // ISAAC_ROS_TENSOR_MSGS__TENSOR_UTILS_HPP_
