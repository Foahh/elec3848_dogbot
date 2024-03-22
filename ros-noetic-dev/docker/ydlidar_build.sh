cd /home/$USERNAME
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build
cd build
cmake ..
make -j4
sudo make install

# Need some idea to fix catkin_make not found in docker build
# cd /home/$USERNAME
# git clone https://github.com/YDLIDAR/ydlidar_ros_driver.git ydlidar_ws/src/ydlidar_ros_driver
# cd ~/ydlidar_ws/src/
# catkin_init_workspace
# cd ~/ydlidar_ws
# catkin_make
# echo 'source ~/ydlidar_ws/devel/setup.bash --extend' >> /home/$USERNAME/.bashrc
# cd ~/ydlidar_ws/src/ydlidar_ros_driver-master/startup
# chmod 760 initenv.sh
# sudo initenv.sh