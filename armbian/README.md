# ROS2Mower
autonomous ROS2 lawn mower

Orange Pi Zero Plus H5 Setup (http://www.orangepi.org/OrangePiZeroPlus):
* git clone https://github.com/armbian/build.git armbian_build_h5
* cd armbian_build_h5
* git checkout sunxi-4.20 
* ./compile.sh BOARD=orangepizeroplus2-h5 BRANCH=next BUILD_DESKTOP=no KERNEL_ONLY=no KERNEL_CONFIGURE=no RELEASE=bionic


Orange Pi Zero H3 Setup (http://www.orangepi.org/orangepizerolts/):
* git clone https://github.com/armbian/build.git armbian_build_h3
* cd armbian_build_h3
* git checkout sunxi-4.20 
* ./compile.sh BOARD=orangepizeroplus2-h3 BRANCH=next KERNEL_ONLY=no KERNEL_CONFIGURE=no RELEASE=bionic

