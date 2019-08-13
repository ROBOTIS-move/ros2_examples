# ROS Packages for reference (examples_rqt)

## Install dependency software for development
- Qt Creator 4.5.x Based on Qt 5.9.x (GCC 7.3.x, 64 bit)
```bash
$ sudo apt install qtcreator
```

## Install dependency package for test
- teleop_twist_keyboard package
```bash
$ cd PATH/THE/YOUR/ROS2_WORKSPACE/src
$ git clone https://github.com/ros2/teleop_twist_keyboard.git
$ cd ..
$ colcon build --symlink-install --packages-select teleop_twist_keyboard
```

## Run Method 1: run examples_rqt
```bash
$ ros2 run examples_rqt examples_rqt
```

## Run Method 2: run rqt and add a examples_rqt as plugin
- Menu > Plugins > Visualization > Viewer
```bash
$ rqt
```

## Test: run examples_rqt, teleop_keyboard node, call service
```bash
$ ros2 run examples_rqt examples_rqt
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
$ ros2 topic echo /cmd_vel
$ ros2 service call /led_control std_srvs/srv/SetBool '{data: True}'
$ ros2 service call /led_control std_srvs/srv/SetBool '{data:False}'
```


