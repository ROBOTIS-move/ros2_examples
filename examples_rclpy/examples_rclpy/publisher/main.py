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

import argparse
import sys

import rclpy

from examples_rclpy.publisher.counter import Counter


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(description='For publisher(counter) node:')

    parser.add_argument(
        '-c',
        '--comment',
        type=str,
        default='Hello',
        help='Specify a comment. Defaults to `Hello`')
    parser.add_argument(
        '-q',
        '--qos_profile',
        type=int,
        default=0,
        help='''
            Specify the QoS profile(0~6) on which to publish. Defaults to SystemDefaultsQoS.
            0 : Default QoSProfile (depth=10)
            1 : SensorDataQoS
            2 : ParametersQoS
            3 : ServicesQoS
            4 : ParameterEventsQoS
            5 : SystemDefaultsQoS
            6 : ActionStatusDefaultQoS
            ref) https://github.com/ros2/rclpy/blob/master/rclpy/rclpy/qos.py
            ''')
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable')

    args = parser.parse_args()
    rclpy.init(args=args.argv)

    node = Counter(args.comment, args.qos_profile)

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
