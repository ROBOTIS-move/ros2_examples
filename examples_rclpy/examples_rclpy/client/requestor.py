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

import rclpy
from rclpy.node import Node

from examples_msgs.srv import Calculation


class Requestor(Node):

    def __init__(self, a, b, arithmetic_operator):
        super().__init__('requestor')

        self.message_info()

        client = self.create_client(Calculation, 'calculate')

        req = Calculation.Request()
        req.a = a
        req.b = b
        req.arithmetic_operator = arithmetic_operator

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        future = client.call_async(req)

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                if future.result() is not None:
                    self.get_logger().info(
                        'Result of calculation: {0}'.format(future.result().result))
                else:
                    self.get_logger().error(
                        'Exception while calling service: {0}'.format(future.exception()))
                break

        self.destroy_node()

        rclpy.shutdown()

    def message_info(self):
        self.get_logger().debug('Test debug message')

        self.get_logger().info('Requestor calls calculator!')
