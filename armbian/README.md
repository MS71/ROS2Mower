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




Orange Pi Zero H3 Setup (http://www.orangepi.org/orangepizerolts/):
* git clone https://github.com/armbian/build.git armbian_build_h3
* cd armbian_build_h3
* git checkout sunxi-4.20 
* ./compile.sh BOARD=orangepizeroplus2-h3 BRANCH=next BUILD_DESKTOP=no KERNEL_ONLY=no KERNEL_CONFIGURE=no RELEASE=bionic
...

