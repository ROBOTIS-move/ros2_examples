#!/usr/bin/env python3

import glob
import os

from setuptools import find_packages
from setuptools import setup

package_name = 'examples_rclpy'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/param', glob.glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools','launch'],
    zip_safe=True,
    author='Darby Lim, Pyo',
    author_email='thlim@robotis.com, pyo@robotis.com',
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    keywords=['ROS', 'ROS2', 'examples', 'rclpy'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Examples of ROS 2 Python nodes'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'publisher = examples_rclpy.publisher.main:main',
            'subscriber = examples_rclpy.subscriber.main:main',
            'server = examples_rclpy.server.main:main',
            'client = examples_rclpy.client.main:main',
        ],
    },
)
