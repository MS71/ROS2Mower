# ROS2Mower
autonomous ROS2 lawn mower

Orange Pi Zero Plus H5 Setup (http://www.orangepi.org/OrangePiZeroPlus):
* git clone https://github.com/armbian/build.git armbian_build_h5
* cd armbian_build_h5
* git checkout sunxi-4.20 
* ./compile.sh BOARD=orangepizeroplus2-h5 BRANCH=next BUILD_DESKTOP=no KERNEL_ONLY=no KERNEL_CONFIGURE=no RELEASE=bionic
* dd if=output/images/Armbian_5.93_Orangepizeroplus2-h5_Ubuntu_bionic_next_4.19.66.img of=/dev/sdd bs=1M
* sync
* Change partitions without deleting content:
*  /dev/sdd1           8192  8396799  8388608    4G 83 Linux
*  /dev/sdd2        8396800 12591103  4194304    2G 82 Linux Swap / Solaris
*  /dev/sdd3       12591104 30449663 17858560  8,5G 83 Linux
=> start board and login via UART root/1234
* mkswap /dev/mmcblk0p2
* swapon /dev/mmcblk0p2
* mkfs.ext4 /dev/mmcblk0p3
* mkdir /data
* edit /etc/fstab:
* /dev/mmcblk0p2  none            swap    sw              0       0
* /dev/mmcblk0p3 /data            ext4    defaults,exec   0       0
* export TERM=vt100
* armbian-config
=> change hostname
=> connect to wifi
=> enable i2c0
=> enable uart1
=> reboot
* login via ssh
* apt-get update && apt-get upgrade
* apt-get install htop gpsd gpsd-clients joe sysbench minicom i2c-tools git libi2c-dev libgps-dev
* edit /etc/defaults/gpsd
=> GPSD_OPTIONS="-n -b -r"
=> DEVICES="/dev/ttyS1"
* check gps with gpsmon
* install ROS2:
* https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians
* sudo locale-gen en_US en_US.UTF-8
* sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
* export LANG=en_US.UTF-8
* sudo apt update && sudo apt install curl gnupg2 lsb-release
* curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
* sudo sh -c 'echo "deb [arch=arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
* sudo apt update
* apt-get install ros-dashing-ros-base ros-dashing-demo-nodes-cpp ros-dashing-demo-nodes-py ros-dashing-joy ros-dashing-nav2-dynamic-params ros-dashing-navigation2 ros-dashing-nav2-map-server ros-dashing-robot-state-publisher ros-dashing-ros2action ros-dashing-ros2bag ros-dashing-ros2component ros-dashing-ros2launch ros-dashing-ros2param ros-dashing-ros2service ros-dashing-ros2topic ros-dashing-vision-opencv
* apt install python3-colcon-common-extensions


Orange Pi Zero (H2+) Setup (http://www.orangepi.org/OrangePiZeroPlus):
* git clone https://github.com/armbian/build.git armbian_build_h5
* cd armbian_build_h5
* git checkout sunxi-4.20 
* ./compile.sh BOARD=orangepizero BRANCH=next BUILD_DESKTOP=no KERNEL_ONLY=no KERNEL_CONFIGURE=no RELEASE=bionic
* ...


