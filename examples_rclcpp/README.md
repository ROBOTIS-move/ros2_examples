# ROS Packages for reference (examples_rclcpp)

## Run examples_rclcpp

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
