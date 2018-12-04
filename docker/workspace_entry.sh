#!/bin/bash

apt-get update
cat packages.list | xargs apt-get -y install

echo "source \"/opt/ros/$ROS_DISTRO/setup.bash\"" >> ~/.bashrc
echo "source \"/catkin_ws/devel/setup.bash\"" >> ~/.bashrc

source "/opt/ros/$ROS_DISTRO/setup.bash"

pushd /catkin_ws/
catkin_make

bash