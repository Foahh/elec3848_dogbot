#!/bin/bash
flag=0

if [ ! -f /etc/udev/rules.d/ydlidar.rules ]; then
    echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' | sudo tee /etc/udev/rules.d/ydlidar.rules > /dev/null
    echo '"/etc/udev/rules.d/ydlidar.rules" Created' 
    flag=1
fi

if [ ! -f /etc/udev/rules.d/ydlidar-V2.rules ]; then
    echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' | sudo tee /etc/udev/rules.d/ydlidar-V2.rules > /dev/null
    echo '"/etc/udev/rules.d/ydlidar-V2.rules" Created'
    flag=1
fi

if [ ! -f /etc/udev/rules.d/ydlidar-2303.rules ]; then
    sudo echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' | sudo tee /etc/udev/rules.d/ydlidar-2303.rules > /dev/null
    echo '"/etc/udev/rules.d/ydlidar-2303.rules" Created'
    flag=1
fi

if [ ! -f /etc/udev/rules.d/arduino.rules ]; then
    echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="2341", MODE:="0666", GROUP:="dialout", SYMLINK+="arduino"' | sudo tee /etc/udev/rules.d/arduino.rules > /dev/null
    echo '"/etc/udev/rules.d/arduino.rules" Created'
    flag=1
fi

if [ $flag -eq 1 ]; then
    service udev reload
    sleep 2
    service udev restart
fi

export ROS_ARCH=$(arch | sed s/aarch64/arm64/ | sed s/x86_64/amd64/)
docker compose build || exit
docker compose up -d