# Gyakuenki
[![latest version](https://img.shields.io/github/v/release/ichiro-its/ninshiki.svg)](https://github.com/ichiro-its/gyakuenki/releases/)

This package implements Inverse Perspective Mapping for the [ROS 2](https://docs.ros.org/en/foxy/index.html) soccer project using Python.

## Dependencies
For running the IPM function, this package needs the humanoid base footprint package and IPM.
```
git clone ${ros2-ws}/src/ipm https://github.com/ros-sports/ipm.git
git clone ${ros2-ws}/src/humanoid_base_footprint https://github.com/ros-sports/humanoid_base_footprint.git
```
Then, those packages needs biped_interfaces, rotation conversion library, and vision_msgs
```
git clone ${ros2-ws}/src/biped_interfaces https://github.com/ros-sports/biped_interfaces.git
git clone ${ros2-ws}/src/rot_conv_lib https://github.com/AIS-Bonn/rot_conv_lib.git
git clone ${ros2-ws}/src/vision_msgs https://github.com/ros-perception/vision_msgs
```

## Build
```
source /opt/ros/${ros-distro}/setup.bash
colcon build --packages-up-to gyakuenki
```
