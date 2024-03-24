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

1. Set up NVIDIA hardware support: `./setup_nvidia.sh`

2. Start docker container: `./compose.sh`

3. `export ROS_ARCH=$(arch | sed s/aarch64/arm64/ | sed s/x86_64/amd64/)` before using Dev Container.

## Credits

[ROS Development in Docker by Kevin DeMarco](https://www.kevindemarco.com/ros/docker/docker-compose/robotics/programming/development/2022/12/28/ros-docker.html)
