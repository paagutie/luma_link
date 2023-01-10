# luma_link 
ROS2 plugin for optical communication between two systems by means of Hydromea LUMA optical modems.

## Requirements
- [ROS2](https://docs.ros.org/en/humble/Installation.html) - humble
- [luma_msgs](https://github.com/paagutie/luma_msgs)

## Installation
- Clone the repositories on the surface computer and on the vehicle (robot) and compile them:
```
$ source /opt/ros/humble/setup.bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ https://github.com/paagutie/luma_link.git
$ cd ..
$ colcon build
```

### Usage
- On the surface computer:
- In a terminal console launch the joystick package (joy_node) or use the [remap_joy](https://github.com/paagutie/remap_joy) to remap the joy_node topic name.
```
$ ros2 run joy joy_node
or 
$ ros2 launch remap_joy remap_joy.launch.py topic_name:='name'
```
- In a second terminal console:
```
$ cd ~/ros2_ws
$ source install/setup.bash
$ ros2 run luma_link luma_link_surface --ros-args --params-file ~/ros2_ws/src/luma_link/config/params.yaml
```
- On the vehicle:
```
$ cd ~/ros2_ws
$ source install/setup.bash
$ ros2 run luma_link luma_link_vehicle --ros-args --params-file ~/ros2_ws/src/luma_link/config/params.yaml
```

