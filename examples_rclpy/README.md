# ROS Packages for reference (examples_rclpy)

## Run examples_rclpy

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
