#!/bin/sh
cd $(dirname $(readlink -f "$0"))
echo "ROS2Mower $1 ..."
if [ $1 = "start" ] ; then
  ln -s /dev/i2c-0 /dev/i2c-ros
  git pull . 
  cd ros2_ws
  . /opt/ros/dashing/setup.sh
  colcon build --symlink-install  
  . ./install/local_setup.sh
  ros2 launch r2m_robot_bringup r2m_robot_bringup.launch.py
fi
echo "ROS2Mower ... done"
