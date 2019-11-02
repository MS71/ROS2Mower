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

#include "io_i2c.h"

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
  //unsigned long funcs;

  /* Geraetedatei oeffnen */
  printf("Opening device...");
  device = io_i2c_open(9);

  rclcpp::init(argc, argv);
  //msg.header.frame_id = "r2m_powersupply";

  auto node = rclcpp::Node::make_shared("r2m_powersupply");

  std_msgs::msg::UInt32 msg_loopcnt;
  auto pub_loopcnt = node->create_publisher<std_msgs::msg::UInt32>("powersupply/loopcnt", 500);

  std_msgs::msg::UInt32 msg_rtc;
  auto pub_rtc = node->create_publisher<std_msgs::msg::UInt32>("powersupply/rtc", 500);

  std_msgs::msg::Float32 msg_ubat;
  auto pub_ubat = node->create_publisher<std_msgs::msg::Float32>("powersupply/ubat", 500);

  std_msgs::msg::Float32 msg_uout;
  auto pub_uout = node->create_publisher<std_msgs::msg::Float32>("powersupply/uout", 500);

  std_msgs::msg::Float32 msg_ucharge;
  auto pub_ucharge = node->create_publisher<std_msgs::msg::Float32>("powersupply/ucharge", 500);

  std_msgs::msg::Float32 msg_usolar;
  auto pub_usolar = node->create_publisher<std_msgs::msg::Float32>("powersupply/usolar", 500);

  std_msgs::msg::Float32 msg_icharge;
  auto pub_icharge = node->create_publisher<std_msgs::msg::Float32>("powersupply/icharge", 500);

  std_msgs::msg::Float32 msg_iout;
  auto pub_iout = node->create_publisher<std_msgs::msg::Float32>("powersupply/iout", 500);

  std_msgs::msg::Float32 msg_isolar;
  auto pub_isolar = node->create_publisher<std_msgs::msg::Float32>("powersupply/isolar", 500);

  rclcpp::WallRate loop_rate(500);

  rclcpp::TimeSource ts(node);
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts.attachClock(clock);

  //RCLCPP_INFO(node->get_logger(), "START");

  while (rclcpp::ok()) 
  {
	  READ_AND_PUB_UINT32(device,TWI_MEM_LOOPCNT,msg_loopcnt,pub_loopcnt);
	  READ_AND_PUB_UINT32(device,TWI_MEM_RTC,msg_rtc,pub_rtc);
  
	  READ_AND_PUB_FLOAT32(device,TWI_MEM_U1,msg_ubat,pub_ubat);
	  READ_AND_PUB_FLOAT32(device,TWI_MEM_U2,msg_uout,pub_uout);
	  READ_AND_PUB_FLOAT32(device,TWI_MEM_U3,msg_ucharge,pub_ucharge);
	  READ_AND_PUB_FLOAT32(device,TWI_MEM_U4,msg_usolar,pub_usolar);

	  READ_AND_PUB_FLOAT32(device,TWI_MEM_I1,msg_icharge,pub_icharge);
	  READ_AND_PUB_FLOAT32(device,TWI_MEM_I2,msg_iout,pub_iout);
	  READ_AND_PUB_FLOAT32(device,TWI_MEM_I3,msg_isolar,pub_isolar);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  io_i2c_close(device);
  
  return 0;
}
