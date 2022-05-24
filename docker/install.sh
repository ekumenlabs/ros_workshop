#!/usr/bin/env bash

apt-get update && apt-get install --no-install-recommends -y $(cat requirements.txt)

sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

apt-get update && apt-get install --no-install-recommends -y $(cat requirements_ros.txt)
