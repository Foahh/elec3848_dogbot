# ROS-SLAM-Docker

Self-using docker for Jetson Nano

## Usage

1. **Set up udev and Nvidia first:** `./setup_nvidia.sh && ./setup_udev.sh`

2. Start docker container: `./compose.sh`

3. Enter the container: `./attach.sh` or `Ctrl+Shift+P` `> Dev Cotainers` commands in vscode

## Credits

[ROS Development in Docker by Kevin DeMarco](https://www.kevindemarco.com/ros/docker/docker-compose/robotics/programming/development/2022/12/28/ros-docker.html)
