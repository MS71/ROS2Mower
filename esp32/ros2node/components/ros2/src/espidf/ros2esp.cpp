/*
 * ros2arduino.cpp
 *
 *  Created on: Jul 6, 2018
 *      Author: kei
 */



#include "ros2esp.h"

#include <stdint.h>
#include <sys/time.h>

extern "C" uint32_t dds_getMilliseconds(void)
{
 	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000LL + (tv.tv_usec / 1000LL));
}

extern "C" uint32_t dds_getMicroseconds(void)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec * 1000000LL + (tv.tv_usec));
}

bool ros2::init(int fd, const char* p_server_ip, uint16_t server_port)
{
  return ros2::init((void*)fd, p_server_ip, server_port, false);
}

#if 0
bool ros2::init(Stream* comm_instance)
{
  return ros2::init((void*)comm_instance);
}

bool ros2::init(UDP* comm_instance, const char* p_server_ip, uint16_t server_port)
{
  return ros2::init((void*)comm_instance, p_server_ip, server_port, false);
}

bool ros2::init(Client* comm_instance, const char* p_server_ip, uint16_t server_port)
{
  return ros2::init((void*)comm_instance, p_server_ip, server_port, true);
}
#endif
