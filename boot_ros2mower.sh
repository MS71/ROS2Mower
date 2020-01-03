#!/bin/sh
cd $(dirname $(readlink -f "$0"))
echo "boot ROS2Mower $1 ..."
if [ $1 = "start" ] ; then
  swapon /dev/mmcblk2p2
  mount /dev/mmcblk2p3 /data
  killall rtkrcv
  (cd /data/ROS2Mower; rtkrcv -s -p 20000 -m 21000 -o rtkrcv.conf &)
  /data/ROS2Mower/ros2mower.sh $1
fi
echo "boot ROS2Mower ... done"
