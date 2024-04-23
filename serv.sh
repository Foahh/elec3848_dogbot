# cd /home/nvidia/dogbot-container/colcon_ws/src
# git pull
# cd /home/nvidia/dogbot-container
# docker compose up -d
# cd tools
# ./attach.sh
source install/setup.bash
colcon build
# ros2 run dogbot_server dogbot_server
ros2 launch dogbot_bringup dogbot_launch.py
# ros2 run 

