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

  std_msgs::msg::Float32 ubat_msg;
  auto ubat_pub = node->create_publisher<std_msgs::msg::Float32>("ubat", 100);
  ubat_msg.data = 0;

  rclcpp::WallRate loop_rate(100);

  rclcpp::TimeSource ts(node);
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts.attachClock(clock);

  //RCLCPP_INFO(node->get_logger(), "START");

  while (rclcpp::ok()) {
    uint8_t addr = 0;
    uint8_t data[4] = {0};
    write(device, &addr, sizeof(addr));
    if( read(device, &data, sizeof(data)) == sizeof(data) )
    {
      ubat_msg.data = (data[0]<<0)|(data[1]<<8)|(data[2]<<16)|(data[3]<<24);
      RCLCPP_INFO(node->get_logger(), "UBat: %f",ubat_msg.data);
      ubat_pub->publish(ubat_msg);
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
