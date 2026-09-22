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

#ifndef ISAAC_ROS_TENSOR_MSGS__TENSOR_MSG_HPP_
#define ISAAC_ROS_TENSOR_MSGS__TENSOR_MSG_HPP_

#include "isaac_ros_tensor_msgs/msg/tensor.hpp"

namespace isaac_ros_tensor_msgs
{

using TensorMsg = msg::Tensor;

}  // namespace isaac_ros_tensor_msgs

namespace nvidia::isaac_ros::isaac_ros_tensor_msgs
{

using TensorMsg = ::isaac_ros_tensor_msgs::TensorMsg;

}  // namespace nvidia::isaac_ros::isaac_ros_tensor_msgs

#endif  // ISAAC_ROS_TENSOR_MSGS__TENSOR_MSG_HPP_
