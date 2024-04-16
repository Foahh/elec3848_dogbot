# ROS Humble Docker

Docker for Jetson Nano

## Usage

1. **Set up udev and Nvidia first:** `./setup_nvidia.sh && ./setup_udev.sh`

2. Start docker container: `docker compose up -d`

3. Enter the container: `./attach.sh`

4. Support Dev Container in vscode

## Credits

[ROS Development in Docker by Kevin DeMarco](https://www.kevindemarco.com/ros/docker/docker-compose/robotics/programming/development/2022/12/28/ros-docker.html)
