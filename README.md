# ROS-SLAM-Docker

## Installed/Built Packages

```
vim
wget
python3-pip
python3-venv
ros-humble-desktop
ros-humble-perception
ros-dev-tools
ros-humble-joint-state-publisher-gui
ros-humble-gazebo-ros-pkgs
ros-humble-ros-gz
ros-humble-ros2-control
ros-humble-ros2-controllers
ros-humble-control-toolbox
ros-humble-realtime-tools
ros-humble-control-msgs
ros-humble-ros2-controllers-test-nodes
YDLidar-SDK
ydlidar_ros2_driver
libserial-dev
platformio
```

## Usage

- Set up NVIDIA hardware support: `./setup_nvidia.sh`

- Automatically decide x86_64 or ARM64v8: `./compose.sh`

## Credits

[ROS Development in Docker by Kevin DeMarco](https://www.kevindemarco.com/ros/docker/docker-compose/robotics/programming/development/2022/12/28/ros-docker.html)
