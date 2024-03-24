#!/bin/bash
cd /home/$USERNAME/ || exit 1
wget -O get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
sudo python3 get-platformio.py
rm get-platformio.py