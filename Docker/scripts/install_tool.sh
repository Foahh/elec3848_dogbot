#!/bin/bash

# Base tools
apt-get update && 
apt-get install -y \
     vim \
     wget \
     python3-pip \
     python3-venv \
     lsb-release \
     gnupg

# PlatformIO
cd /home/"$USERNAME"/ || exit 1
wget -O get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
rm get-platformio.py