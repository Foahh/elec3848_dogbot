# ROS-SLAM-Docker

## Installed/Built Packages

```
vim
python3-pip
python3-venv
YDLidar-SDK
ydlidar_ros2_driver
libserial-dev
platformio
ros-desktop-full
ros-dev-tools
ros-joint-state-publisher-gui
ros-gazebo-ros-pkgs
ros-ros-gz
ros-ros2-control
ros-ros2-controllers
ros-control-toolbox
ros-realtime-tools
ros-control-msgs
ros-ros2-controllers-test-nodes
```

## Usage

1. Set up NVIDIA hardware support: `./setup_nvidia.sh`

2. Start docker container: `./compose.sh`

3. Enter the container: `./attach.sh` or `Ctrl+Shift+P` `> Dev Cotainers` commands in vscode

## Credits

[ROS Development in Docker by Kevin DeMarco](https://www.kevindemarco.com/ros/docker/docker-compose/robotics/programming/development/2022/12/28/ros-docker.html)
