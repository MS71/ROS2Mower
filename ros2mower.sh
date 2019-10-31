#!/bin/sh
cd $(dirname $(readlink -f "$0"))
echo "ROS2Mower $1 ..."
if [ $1 = "start" ] ; then
  ln -s /dev/i2c-0 /dev/i2c-ros
  git pull . 
  . /opt/ros/dashing/local_setup.sh
  ros2 run demo_nodes_cpp talker
fi
echo "ROS2Mower ... done"
