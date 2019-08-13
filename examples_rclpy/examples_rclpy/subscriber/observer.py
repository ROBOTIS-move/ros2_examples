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
from rcl_interfaces.msg import ParameterEvent
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_action_status_default
from rclpy.qos import qos_profile_parameter_events
from rclpy.qos import qos_profile_parameters
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_services_default
from rclpy.qos import qos_profile_system_default

from examples_msgs.msg import Count


class Observer(Node):

    def __init__(self, qos_profile):
        super().__init__('observer')

        self.get_logger().debug('Test debug message')

        self.init_parameters()
        self.parameter_event_callback()

        self.i = 0
        self.blind_count = 0

        qos = self.get_qos(qos_profile)
        self.sub = self.create_subscription(Count, 'count', self.counter_callback, qos)

    def counter_callback(self, msg):
        self.i += 1

        if self.blind:
            if self.blind_count >= 10:
                self.set_parameters([Parameter('blind', Parameter.Type.BOOL, False)])
                self.blind = self.get_parameter('blind').value
                self.blind_count = 0

            self.get_logger().info('Blind mode')
            self.blind_count += 1
        else:
            self.get_logger().info(
                '[{0}] Observed: \'{1}\''.format(self.comment, msg.count + self.offset))

    def param_event_callback(self, event):
        for new_parameter in event.new_parameters:
            self.get_logger().info('New parameter name : {0}'.format(new_parameter.name))

        for changed_parameter in event.changed_parameters:
            self.get_logger().info('Changed parameter name : {0}'.format(changed_parameter.name))

            if changed_parameter.name == 'blind':
                self.blind = Parameter.from_parameter_msg(changed_parameter).value
            elif changed_parameter.name == 'offset':
                self.offset = Parameter.from_parameter_msg(changed_parameter).value
            elif changed_parameter.name == 'comment':
                self.comment = Parameter.from_parameter_msg(changed_parameter).value

        for deleted_parameter in event.deleted_parameters:
            self.get_logger().info('Deleted parameter name : {0}'.format(deleted_parameter.name))

    def init_parameters(self):
        self.declare_parameter('comment', 'No comments', ParameterDescriptor())
        self.declare_parameter('blind', False, ParameterDescriptor())
        self.declare_parameter('offset', 0, ParameterDescriptor())
        self.declare_parameter('favorite/numbers_integers', [1, 2], ParameterDescriptor())
        self.declare_parameter('favorite/numbers_doubles', [1.0, 2.0], ParameterDescriptor())

        self.comment = self.get_parameter('comment').value
        self.blind = self.get_parameter('blind').value
        self.offset = self.get_parameter('offset').value

    def parameter_event_callback(self):
        self.param_event_sub = self.create_subscription(
            ParameterEvent,
            'parameter_events',
            self.param_event_callback,
            qos_profile_parameter_events)

    def get_qos(self, i):
        return {
            0: QoSProfile(depth=10),
            1: qos_profile_sensor_data,
            2: qos_profile_parameters,
            3: qos_profile_services_default,
            4: qos_profile_parameter_events,
            5: qos_profile_system_default,
            6: qos_profile_action_status_default}[i]
