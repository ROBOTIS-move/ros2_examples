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
# Authors: Darby Lim

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    auto_activate = LaunchConfiguration('auto_activate', default='False')
    namespace = LaunchConfiguration('ns', default='example')

    return LaunchDescription([
        LogInfo(msg=['Execute lifecycle node!!']),

        DeclareLaunchArgument(
            'auto_activate',
            default_value=auto_activate,
            description='Specifying auto activate node'),

        DeclareLaunchArgument(
            'ns',
            default_value=namespace,
            description='Specifying namespace to node'),

        Node(
            node_namespace=namespace,
            package='examples_lifecycle',
            node_executable='robot',
            parameters=[{'robot_name': 'C3PO'}],
            arguments=['-a', auto_activate],
            output='screen'),
    ])
