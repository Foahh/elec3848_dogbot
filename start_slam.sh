#!/bin/bash
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
ros2 launch slam_toolbox online_async_launch.py slam_params_file:="$SCRIPTPATH"/dogbot_hardware/bringup/config/navigation.yaml
