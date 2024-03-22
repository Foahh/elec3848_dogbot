cd /home/$USERNAME
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd /home/$USERNAME/YDLidar-SDK
mkdir build
cd build
cmake ..
make -j4
sudo make install