#!/bin/bash
docker build -t foahh/ros-humble-desktop:$(arch | sed s/aarch64/arm64/ | sed s/x86_64/amd64/) -f ./Docker/desktop.dockerfile .