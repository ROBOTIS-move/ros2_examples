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

from rclpy.node import Node

from examples_msgs.srv import Calculation


class Calculator(Node):

    def __init__(self):
        super().__init__('calculator')

        self.message_info()

        self.server = self.create_service(Calculation, 'calculate', self.calculator_callback)

    def message_info(self):
        self.get_logger().debug('Test debug message')

        self.get_logger().info('Calculator is ready to get calculation!')
        self.get_logger().info('Please use CLI or Service Caller in rqt to call service server')
        self.get_logger().info('Examples of CLI')
        self.get_logger().info('ros2 service call /calculate examples_msgs/srv/Calculation "{a: 1, b: 2, arithmetic_operator: plus}"')

    def calculator_callback(self, request, response):
        self.get_logger().info('Incomming request')
        self.get_logger().info(
            'a = {0}, b = {1}, calculator(a {2} b)'.format(
                request.a,
                request.b,
                request.arithmetic_operator))

        response.result = self.calculation(request.a, request.b, request.arithmetic_operator)

        return response

    def calculation(self, a, b, arithmetic_operator):
        if arithmetic_operator == 'plus':
            return a + b
        elif arithmetic_operator == 'minus':
            return a - b
        elif arithmetic_operator == 'multiply':
            return a * b
        elif arithmetic_operator == 'division':
            try:
                return a // b
            except ZeroDivisionError as e:
                self.get_logger().error(e)
                return 0
        else:
            self.get_logger().error(
                'Please make sure arithmetic_operator(plus, minus, multiply, division)')
            return 0
        return 0
