#!/bin/bash
flag=0

if [ ! -f /etc/udev/rules.d/ydlidar.rules ]; then
    sudo echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' > /etc/udev/rules.d/ydlidar.rules
    flag=1
fi

if [ ! -f /etc/udev/rules.d/ydlidar-V2.rules ]; then
    sudo echo 'KERNEL=="ttyACM*", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' > /etc/udev/rules.d/ydlidar-V2.rules
    flag=1
fi

if [ ! -f /etc/udev/rules.d/ydlidar-2303.rules ]; then
    sudo echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' > /etc/udev/rules.d/ydlidar-2303.rules
    flag=1
fi

if [ $flag -eq 1 ]; then
    service udev reload
    sleep 2
    service udev restart
fi

if [ "$(arch)" == "x86_64" ];
then
    cd amd64/ || exit
elif [ "$(arch)" == "aarch64" ]
then
    cd arm64/ || exit
fi

docker compose build || exit
docker compose up