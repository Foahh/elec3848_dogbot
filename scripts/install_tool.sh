#!/bin/bash
apt-get update && apt-get install -y \
     vim \
     wget \
     python3-pip \
     python3-venv
rm -rf /var/lib/apt/lists/*

