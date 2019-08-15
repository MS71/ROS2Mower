// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int32.hpp"

#define TWI_MEM_LOOPCNT   (0x00) /* 32Bit */
                       // (0x02)
#define TWI_MEM_RTC       (0x04) /* 32Bit */
                       // (0x06)
#define TWI_MEM_PMSW      (0x08)
                       
#define TWI_MEM_SHDWNCNT  (0x10)
#define TWI_MEM_PWRUPCNT  (0x12)

#define TWI_MEM_U1        (0x20)
#define TWI_MEM_U2        (0x22)
#define TWI_MEM_U3        (0x24)
#define TWI_MEM_U4        (0x26)

#define TWI_MEM_I1        (0x30)
#define TWI_MEM_I2        (0x32)
#define TWI_MEM_I3        (0x34)

int main(int argc, char * argv[])
{
  int device;
  unsigned long funcs;

  /* Geraetedatei oeffnen */
  printf("Opening device...");
  if ((device = open("/dev/i2c-0", O_RDWR)) < 0)
    {
	perror("open() failed");
    	exit (1);
    }

  /* Abfragen, ob die I2C-Funktionen da sind */
  if (ioctl(device,I2C_FUNCS,&funcs) < 0)
    {
    perror("ioctl() I2C_FUNCS failed");
    exit (1);
    }

  if (ioctl(device, I2C_SLAVE, 9) < 0)
  {
    perror("ioctl() powersupply I2C slave not found");
    exit (1);
  }

  rclcpp::init(argc, argv);
  //msg.header.frame_id = "r2m_powersupply";

  auto node = rclcpp::Node::make_shared("r2m_powersupply");

  std_msgs::msg::UInt32 msg_loopcnt;
  auto pub_loopcnt = node->create_publisher<std_msgs::msg::UInt32>("powersupply/loopcnt", 500);

  std_msgs::msg::Float32 msg_ubat;
  auto pub_ubat = node->create_publisher<std_msgs::msg::Float32>("powersupply/ubat", 500);

  rclcpp::WallRate loop_rate(500);

  rclcpp::TimeSource ts(node);
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts.attachClock(clock);

  //RCLCPP_INFO(node->get_logger(), "START");

  while (rclcpp::ok()) {
	  {
		/* TWI_MEM_LOOPCNT */
		uint8_t addr = TWI_MEM_LOOPCNT;
		uint8_t data[4] = {0};
		write(device, &addr, sizeof(addr));
		if( read(device, &data, sizeof(data)) == sizeof(data) )
		{
		  msg_loopcnt.data = (data[0]<<0)|(data[1]<<8)|(data[2]<<16)|(data[3]<<24);
		  RCLCPP_INFO(node->get_logger(), "UBat: %f",msg_loopcnt.data);
		  pub_loopcnt->publish(msg_loopcnt);
		}
	  }

	  {
		/* TWI_MEM_LOOPCNT */
		uint8_t addr = TWI_MEM_U1;
		uint8_t data[2] = {0};
		write(device, &addr, sizeof(addr));
		if( read(device, &data, sizeof(data)) == sizeof(data) )
		{
		  msg_ubat.data = (data[0]<<0)|(data[1]<<8);
		  RCLCPP_INFO(node->get_logger(), "UBat: %f",msg_ubat.data);
		  pub_ubat->publish(msg_ubat);
		}
	  }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
