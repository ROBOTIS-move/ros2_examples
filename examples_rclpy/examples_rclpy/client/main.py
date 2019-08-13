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

from examples_rclpy.client.requestor import Requestor


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(description='For client(requestor) node:')

    parser.add_argument(
        '-a',
        '--one',
        type=int,
        default=1,
        help='Specify the number. Defaults to 1.')
    parser.add_argument(
        '-b',
        '--another',
        type=int,
        default=2,
        help='Specify the number. Defaults to 2.')
    parser.add_argument(
        '-o',
        '--operator',
        type=str,
        default='plus',
        help='Specify the arithmetic operator(plus, minus, multiply, division). Defaults to plus.')
    parser.add_argument(
        'argv', nargs=argparse.REMAINDER,
        help='Pass arbitrary arguments to the executable')

    args = parser.parse_args()
    rclpy.init(args=args.argv)

    node = Requestor(args.one, args.another, args.operator)


if __name__ == '__main__':
    main()
