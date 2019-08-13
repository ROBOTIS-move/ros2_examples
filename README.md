# ROS Packages for reference (ros2_examples)

## Build Status
|Travis CI (master)|Travis CI (develop)|
|:---:|:---:|
|[![Build Status](https://travis-ci.com/ROBOTIS-Platform/ros2_examples.svg?token=yZqTAoWyh1dviVKqstdx&branch=master)](https://travis-ci.com/ROBOTIS-Platform/ros2_examples)|[![Build Status](https://travis-ci.com/ROBOTIS-Platform/ros2_examples.svg?token=yZqTAoWyh1dviVKqstdx&branch=develop)](https://travis-ci.com/ROBOTIS-Platform/ros2_examples)|

## examples_msgs

### Message file
- Count.msg

### Service file
- Calculation.srv

### Action file
- Led.action

### examples_rclcpp

### Publisher
```bash
$ ros2 run examples_rclcpp publisher -c ${comment} -q ${qos_profile}
```

### Subscriber
```bash
$ ros2 run examples_rclcpp subscriber -q ${qos_profile}
```

### Server
```bash
$ ros2 run examples_rclcpp server
$ ros2 service call /calculate examples_msgs/Calculation "{a: 1, b: 2, arithmetic_operator: "plus"}"
```

${arithmetic_operator} : plus, minus, multiply, division

### Client
```bash
$ ros2 run examples_rclcpp client -a ${number} -b ${number} -o ${arithmetic_operator}
```

${arithmetic_operator} : plus, minus, multiply, division

### Launch
```bash
$ ros2 launch examples_rclcpp pub.launch.py
```

```bash
$ ros2 launch examples_rclcpp sub.launch.py
```

```bash
$ ros2 launch examples_rclcpp multiple_node.launch.py
```

```bash
$ ros2 launch examples_rclcpp multiple_launch.launch.py
```

## examples_rclpy

### Publisher
```bash
$ ros2 run examples_rclpy publisher -q ${qos_profile}
```

### Subscriber
```bash
$ ros2 run examples_rclpy subscriber -q ${qos_profile}
```

### Server
```bash
$ ros2 run examples_rclpy server
$ ros2 service call /calculate examples_msgs/Calculation "{a: 1, b: 2, arithmetic_operator: "plus"}"
```

### Client
```bash
$ ros2 run examples_rclpy client -a ${number} -b ${number} -o ${arithmetic_operator}
```

${arithmetic_operator} : plus, minus, multiply, division

### Launch
```bash
$ ros2 launch examples_rclpy pub.launch.py
```

```bash
$ ros2 launch examples_rclpy sub.launch.py
```

```bash
$ ros2 launch examples_rclpy multiple_node.launch.py
```

```bash
$ ros2 launch examples_rclpy multiple_launch.launch.py
```

## examples_rqt

### Install dependency software for development
- Qt Creator 4.5.x Based on Qt 5.9.x (GCC 7.3.x, 64 bit)
```bash
$ sudo apt install qtcreator
```

### Install dependency package for test
- teleop_twist_keyboard package
```bash
$ cd PATH/THE/YOUR/ROS2_WORKSPACE/src
$ git clone https://github.com/ros2/teleop_twist_keyboard.git
$ cd ..
$ colcon build --symlink-install --packages-select teleop_twist_keyboard
```

### Run Method 1: run examples_rqt
```bash
$ ros2 run examples_rqt examples_rqt
```

### Run Method 2: run rqt and add a examples_rqt as plugin
- Menu > Plugins > Visualization > Viewer
```bash
$ rqt
```

### Test: run examples_rqt, teleop_keyboard node, call service
```bash
$ ros2 run examples_rqt examples_rqt
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
$ ros2 topic echo /cmd_vel
$ ros2 service call /led_control std_srvs/srv/SetBool '{data: True}'
$ ros2 service call /led_control std_srvs/srv/SetBool '{data:False}'
```

## examples_tf

### Run examples_tf
```bash
$ ros2 run examples_tf2 broadcaster
$ ros2 run examples_tf2 listener
$ ros2 run examples_tf2 static_broadcaster
```

```bash
$ ros2 run rviz2 rviz2 -d examples_tf2/rviz/arm.rviz
```

```bash
$ ros2 service call state std_srvs/srv/SetBool "data: false"
```
