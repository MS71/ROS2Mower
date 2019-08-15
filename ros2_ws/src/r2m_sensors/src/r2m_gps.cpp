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

#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <gps.h>

//#define DEG2RAD M_PI / 180.0

int main(int argc, char * argv[])
{
  int rc;
  sensor_msgs::msg::NavSatFix msg;
  struct gps_data_t gps_data;
  if ((rc = gps_open("localhost", "2947", &gps_data)) == -1) {
    printf("code: %d, reason: %s\n", rc, gps_errstr(rc));
    return EXIT_FAILURE;
  }
  gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);

  rclcpp::init(argc, argv);
  msg.header.frame_id = "r2m_gps";

  auto node = rclcpp::Node::make_shared("r2m_gps");

  auto gps_pub = node->create_publisher<sensor_msgs::msg::NavSatFix>("gps", 100);
  rclcpp::WallRate loop_rate(100);

  //RCLCPP_INFO(node->get_logger(), "START");

  rclcpp::TimeSource ts(node);
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  ts.attachClock(clock);

  while (rclcpp::ok()) {

        if ((rc = gps_read(&gps_data)) == -1) {
            printf("error occured reading gps data. code: %d, reason: %s\n", rc, gps_errstr(rc));
        } else {
            /* Display data from the GPS receiver. */
            if ((gps_data.status == STATUS_FIX) && 
                (gps_data.fix.mode == MODE_2D || gps_data.fix.mode == MODE_3D) &&
                !isnan(gps_data.fix.latitude) && 
                !isnan(gps_data.fix.longitude)) {
                    //gettimeofday(&tv, NULL); EDIT: tv.tv_sec isn't actually the timestamp!
                    printf("latitude: %f, longitude: %f, speed: %f, timestamp: %lf\n", gps_data.fix.latitude, gps_data.fix.longitude, gps_data.fix.speed, gps_data.fix.time); //EDIT: Replaced tv.tv_sec with gps_data.fix.time

		msg.header.stamp = clock->now();
		msg.status.status = 0 /*sensor_msgs::msg::NavSatStatus::STATUS_FIX*/;
		msg.status.service = 1 /* sensor_msgs::msg::NavSatStatus::SERVICE_GPS */;
		msg.latitude = gps_data.fix.latitude;
		msg.longitude = gps_data.fix.longitude;
		msg.altitude = gps_data.fix.altitude;
		msg.position_covariance_type = msg.COVARIANCE_TYPE_UNKNOWN;
		gps_pub->publish(msg);

            } else {
                printf("no GPS data available\n");
            }
        }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  /* When you are done... */
  gps_stream(&gps_data, WATCH_DISABLE, NULL);
  gps_close (&gps_data);

  rclcpp::shutdown();

  return 0;
}
