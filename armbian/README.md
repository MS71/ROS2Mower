# ROS2Mower
autonomous ROS2 lawn mower

Orange Pi Zero Plus H5 Setup (http://www.orangepi.org/OrangePiZeroPlus):
* git clone https://github.com/armbian/build.git armbian_build_h5
* cd armbian_build_h5
* git checkout sunxi-4.20 
* ./compile.sh BOARD=orangepizeroplus2-h5 BRANCH=next BUILD_DESKTOP=no KERNEL_ONLY=no KERNEL_CONFIGURE=no RELEASE=bionic
* dd if=output/images/Armbian_5.93_Orangepizeroplus2-h5_Ubuntu_bionic_next_4.19.66.img of=/dev/sdd bs=1M
* sync
* Gerät      Boot   Anfang     Ende Sektoren Größe Kn Typ
* /dev/sdd1           8192  8396799  8388608    4G 83 Linux
* /dev/sdd2        8396800 12591103  4194304    2G 82 Linux Swap / Solaris
* /dev/sdd3       12591104 30449663 17858560  8,5G 83 Linux
=> start board and login via UART root/1234
* mkswap /dev/mmcblk0p2
* swapon /dev/mmcblk0p2
* mkfs.ext4 /dev/mmcblk0p3
* mkdir /data
* edit /etc/fstab:
* /dev/mmcblk0p2  none            swap    sw              0       0
* /dev/mmcblk0p3 /data            ext4    defaults        0       0
* export TERM=vt100
* armbian-config
=> change hostname
=> connect to wifi
=> enable i2c0
=> enable uart1
=> reboot
* login via ssh
* apt-get update && apt-get upgrade
* apt-get install htop gpsd joe 
=> install ROS2
* https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians
* sudo locale-gen en_US en_US.UTF-8
* sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
* export LANG=en_US.UTF-8
* sudo apt update && sudo apt install curl gnupg2 lsb-release
* curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
* sudo sh -c 'echo "deb [arch=arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
* sudo apt update
* sudo apt install ros-dashing-ros-base











Orange Pi Zero H3 Setup (http://www.orangepi.org/orangepizerolts/):
* git clone https://github.com/armbian/build.git armbian_build_h3
* cd armbian_build_h3
* git checkout sunxi-4.20 
* ./compile.sh BOARD=orangepizeroplus2-h3 BRANCH=next BUILD_DESKTOP=no KERNEL_ONLY=no KERNEL_CONFIGURE=no RELEASE=bionic
...

