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

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_action_status_default
from rclpy.qos import qos_profile_parameter_events
from rclpy.qos import qos_profile_parameters
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_services_default
from rclpy.qos import qos_profile_system_default

from examples_msgs.msg import Count


class Counter(Node):

    def __init__(self, comment, qos_profile):
        super().__init__('counter')

        self.get_logger().debug('Test debug message')

        self.i = 0
        self.comment = comment

        self.declare_parameter('comment', 'Hello', ParameterDescriptor())

        qos = self.get_qos(qos_profile)
        self.pub = self.create_publisher(Count, 'count', qos)

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.i += 1

        msg = Count()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.count = self.i

        self.get_logger().info('[{0}] Counting: \'{1}\''.format(self.comment, msg.count))

        self.pub.publish(msg)

    def get_qos(self, i):
        return {
            0: QoSProfile(depth=10),
            1: qos_profile_sensor_data,
            2: qos_profile_parameters,
            3: qos_profile_services_default,
            4: qos_profile_parameter_events,
            5: qos_profile_system_default,
            6: qos_profile_action_status_default}[i]
