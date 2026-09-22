#!/usr/bin/env python3

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from isaac_ros_gpu_partitioning.gpu_partitioning import GpuPartition
from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate a launch description for the gpu partition test node."""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    config_file = os.path.join(current_dir, 'cuda_mps.yaml')
    gpu_partition = GpuPartition(config_file=config_file, clear_partitions=True, start_server=True)
    name = 'partition_1'
    result = gpu_partition.get_partition_id(name=name)
    if result is None:
        raise ValueError(f'Partition ID not found for name: {name}')
    partition_id, sm_count = result
    print(f'Partition {name} has ID: {partition_id} and SM Count: {sm_count}')

    node = ComposableNode(
        name='test_node',
        package='isaac_ros_gpu_partitioning',
        plugin='nvidia::GpuPartitionTestNode',
    )
    container = ComposableNodeContainer(
        name='gpu_partition_test_container',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[node],
        namespace='',
        output='screen',
    )
    action = gpu_partition.gpu_partition_launch_action(name=name)

    def stop_server(_):
        gpu_partition.stop_cuda_mps_server()
        return []

    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[OpaqueFunction(function=stop_server)]
        )
    )
    launch_description = LaunchDescription([shutdown_handler, action, container])
    return launch_description
