#!/bin/bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard speed:=0.1 turn:=0.5 stamped:=true --ros-args --remap /cmd_vel:=/dogbot_base_controller/cmd_vel