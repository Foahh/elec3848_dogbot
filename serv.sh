cd /home/nvidia/dogbot-container/colcon_ws/src
git pull
cd /home/nvidia/dogbot-container
docker compose up -d
cd tools
./attach.sh
# source install/setup.bash
# ros2 run dogbot_server

# colcon build
