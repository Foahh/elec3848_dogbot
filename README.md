# ELEC3848 Project - DogBot

## Introduction

DogBot is a simple bot designed for performing tasks like navigation running on ROS2 Humble, object detection with jetson inferences and mechanical arm controls.

## Implementation

The core functionality of the bot is powered by ros2_control, slam_toolbox and nav2.

## Hardware connected with Jetson Nano

* ICM20948 - installed on the i2c pin of Jetson Nano.
* Arduino MEGA2560
* YDLiDar X2

## Docker Usage

1. **Set up udev first:** `./container/tools/setup_udev.sh`

2. Start docker container: `docker compose up -d`

3. Enter the container: `./container/tools/attach.sh`

## Build

```sh
mkdir dogbot_ws && cd dogbot_ws
git clone https://github.com/NFHr/elec3848_dogbot.git
mv ./elec3848_dogbot/software ./src
colcon build
```

## Start

`
ros2 launch dogbot_hardware dogbot.launch.py
`

## Note

I did this project while learning the ROS2 from various online sources. The code or implementation quality is not ensured.

## Credits

The development of DogBot has been greatly influenced by several outstanding repositories.

[Linorobot](https://linorobot.org/)
[ros2_control_demos](https://github.com/ros-controls/ros2_control_demos)
[diffdrive_arduino](https://github.com/joshnewans/diffdrive_arduino/tree/humble)
[ROS Development in Docker by Kevin DeMarco](https://www.kevindemarco.com/ros/docker/docker-compose/robotics/programming/development/2022/12/28/ros-docker.html)