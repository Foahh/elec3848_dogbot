# ELEC2848 Project - DogBot

## Introduction

DogBot is a simple bot mainly designed for slam and navigation running on ROS2 Humble.

## Implementation

The core functionality of the bot is powered by ros2_control, slam_toolbox and nav2.

Firmware for Arduino: https://github.com/NFHr/elec3848_dogbot_firmware.git

## Hardware connected with Jetson Nano

* ICM20948 - installed on the i2c pin of Jetson Nano.
* Arduino MEGA2560
* YDLiDar X2

## Build

Development environment: `https://github.com/NFHr/humble-docker.git`

```sh
mkdir dogbot_ws && cd dogbot_ws
git clone https://github.com/NFHr/elec3848_dogbot.git ./src
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
