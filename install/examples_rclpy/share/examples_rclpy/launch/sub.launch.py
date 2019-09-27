#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim, Pyo

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 0: Default QoSProfile(depth=10)
    qos_profile = LaunchConfiguration('qos_profile', default=0)

    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('examples_rclpy'),
            'param',
            'examples_parameter.yaml'))

    namespace = LaunchConfiguration('ns', default='example')

    return LaunchDescription([
        LogInfo(msg=['Execute subscriber!!']),

        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Specifying parameter direction'),

        DeclareLaunchArgument(
            'qos_profile',
            default_value=qos_profile,
            description='Specifying qos_profile to publisher. Default QoSProfile(depth=10)'),

        DeclareLaunchArgument(
            'namespace',
            default_value='ns',
            description='Specifying namespace to node'),

        Node(
            node_namespace=namespace,
            package='examples_rclpy',
            node_executable='subscriber',
            #  node_name='observer',
            parameters=[param_dir],
            arguments=['-q', qos_profile],
            #  remappings=[('count', 'count')],
            output='screen'),
    ])
