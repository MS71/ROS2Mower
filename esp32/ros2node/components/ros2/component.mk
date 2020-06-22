COMPONENT_ADD_INCLUDEDIRS := \
	src/espidf \
	src \
	src/uxr \
	src/uxr/client \
	src/uxr/client/profile \
	src/uxr/client/profile/discovery \
	src/uxr/client/profile/transport \
	src/uxr/client/profile/transport/serial \
	src/uxr/client/profile/transport/ip \
	src/uxr/client/profile/transport/ip/tcp \
	src/uxr/client/profile/transport/ip/udp \
	src/uxr/client/core \
	src/uxr/client/core/session \
	src/uxr/client/core/session/stream \
	src/uxr/client/core/type \
	src/uxr/client/core/communication \
	src/uxr/client/util \
	src/ros2 \
	src/ros2/tf2_msgs \
	src/ros2/diagnostic_msgs \
	src/ros2/builtin_interfaces \
	src/ros2/xrcedds \
	src/ros2/xrcedds/micro_xrce_dds \
	src/ros2/xrcedds/micro_xrce_dds/lib \
	src/ros2/xrcedds/micro_xrce_dds/lib/thirdparty \
	src/ros2/xrcedds/micro_xrce_dds/lib/thirdparty/microcdr \
	src/ros2/xrcedds/micro_xrce_dds/lib/thirdparty/microcdr/include \
	src/ros2/xrcedds/micro_xrce_dds/lib/thirdparty/microcdr/include/ucdr \
	src/ros2/xrcedds/micro_xrce_dds/lib/thirdparty/microcdr/src \
	src/ros2/xrcedds/micro_xrce_dds/lib/thirdparty/microcdr/src/c \
	src/ros2/xrcedds/micro_xrce_dds/lib/thirdparty/microcdr/src/c/types \
	src/ros2/xrcedds/micro_xrce_dds/lib/include \
	src/ros2/xrcedds/micro_xrce_dds/lib/include/uxr \
	src/ros2/xrcedds/micro_xrce_dds/lib/include/uxr/client \
	src/ros2/xrcedds/micro_xrce_dds/lib/include/uxr/client/profile \
	src/ros2/xrcedds/micro_xrce_dds/lib/include/uxr/client/profile/discovery \
	src/ros2/xrcedds/micro_xrce_dds/lib/include/uxr/client/profile/transport \
	src/ros2/xrcedds/micro_xrce_dds/lib/include/uxr/client/profile/transport/serial \
	src/ros2/xrcedds/micro_xrce_dds/lib/include/uxr/client/profile/transport/ip \
	src/ros2/xrcedds/micro_xrce_dds/lib/include/uxr/client/profile/transport/ip/tcp \
	src/ros2/xrcedds/micro_xrce_dds/lib/include/uxr/client/profile/transport/ip/udp \
	src/ros2/xrcedds/micro_xrce_dds/lib/include/uxr/client/core \
	src/ros2/xrcedds/micro_xrce_dds/lib/include/uxr/client/core/session \
	src/ros2/xrcedds/micro_xrce_dds/lib/include/uxr/client/core/session/stream \
	src/ros2/xrcedds/micro_xrce_dds/lib/include/uxr/client/core/type \
	src/ros2/xrcedds/micro_xrce_dds/lib/include/uxr/client/core/communication \
	src/ros2/xrcedds/micro_xrce_dds/lib/include/uxr/client/util \
	src/ros2/xrcedds/micro_xrce_dds/lib/src \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/profile \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/profile/discovery \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/profile/discovery/transport \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/profile/transport \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/profile/transport/serial \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/profile/transport/ip \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/profile/transport/ip/tcp \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/profile/transport/ip/udp \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/core \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/core/session \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/core/session/stream \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/core/serialization \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/core/log \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/util \
	src/ros2/geometry_msgs \
	src/ros2/sensor_msgs \
	src/ros2/nav_msgs \
	src/ros2/std_msgs \
	src/ros2/turtlebot3_msgs \
	src/ucdr

COMPONENT_SRCDIRS += \
	src/espidf \
	src/ros2 \
	src/ros2/xrcedds \
	src/ros2/xrcedds/micro_xrce_dds \
	src/ros2/xrcedds/micro_xrce_dds/lib/thirdparty/microcdr/src/c \
	src/ros2/xrcedds/micro_xrce_dds/lib/thirdparty/microcdr/src/c/types \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/profile/discovery \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/profile/transport/ip/udp \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/profile/transport/ip \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/core/session/stream \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/core/session \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/core/serialization \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/core/log \
	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/util 

CFLAGS += -DUSER_TRANSPORT_TYPE=USER_TRANSPORT_TYPE_UDP
CPPFLAGS += -DUSER_TRANSPORT_TYPE=USER_TRANSPORT_TYPE_UDP

#	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/profile/transport/serial \
#	src/ros2/xrcedds/micro_xrce_dds/lib/src/c/profile/transport/ip/tcp \
#./ros2/xrcedds/micro_xrce_dds/lib/src/c/profile/discovery/transport/udp_transport_datagram_posix.c
#./ros2/xrcedds/micro_xrce_dds/lib/src/c/profile/discovery/transport/udp_transport_datagram_windows.c

#	src/uxr/client/profile/transport 
#./uxr/client/profile/transport/serial/serial_transport_arduino.c
#./uxr/client/profile/transport/ip/tcp/tcp_transport_arduino.c
#./uxr/client/profile/transport/ip/udp/udp_transport_arduino.c

