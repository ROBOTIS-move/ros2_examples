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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # [RECOMMENDED] If you want handle arguments out of this launch file,
    # you have to set LaunchConfiguration
    # If you set arguments below, you can't access topic_name CLI or other launch files
    # qos_profile = 0

    # 0: Default QoSProfile(depth=10)
    qos_profile = LaunchConfiguration('qos_profile', default=0)
    namespace = LaunchConfiguration('ns', default='example')

    return LaunchDescription([
        LogInfo(msg=['Execute two ''publisher''s has different node name!!']),

        # [RECOMMENDED] This func allows you to expose the arguments
        DeclareLaunchArgument(
            'topic_name',
            default_value='count',
            description='Specifying topic name to publisher'),

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
            node_executable='publisher',
            node_name='first_pub',
            parameters=[{'message': 'First Pub'}],
            arguments=['-q', qos_profile],
            output='screen'),

        Node(
            node_namespace=namespace,
            package='examples_rclpy',
            node_executable='publisher',
            node_name='second_pub',
            parameters=[{'message': 'Second Pub'}],
            arguments=['-q', qos_profile],
            output='screen'),
    ])
