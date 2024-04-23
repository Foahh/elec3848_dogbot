#!/bin/bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard speed:=0.1 turn:=0.5 --ros-args --remap /cmd_vel:=/cmd_vel_raw