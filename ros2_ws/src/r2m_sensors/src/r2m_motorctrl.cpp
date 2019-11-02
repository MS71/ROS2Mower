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

#include <inttypes.h>
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_srvs/srv/empty.hpp"

#include "r2m_sensors/srv/add_two_ints.hpp"
#include "r2m_sensors/srv/set_motor_pid.hpp"

#include "io_i2c.h"

#define TWI_REG_U64_CLOCK_US	((0<<8)|8)
#define TWI_REG_U16_MODE  	    ((8<<8)|2)
#define TWI_REG_U16_AUTOBREAK   ((10<<8)|2)

// PID Parameter
#define TWI_REG_S16_PID_K_PID	((0x10<<8)|6)

// Motor A
#define TWI_REG_S16_MA_PID_SP	(((0x20+0)<<8)|2)
#define TWI_REG_S16_MA_PID_PV	(((0x20+2)<<8)|2)
#define TWI_REG_S16_MA_PID_OV	(((0x20+4)<<8)|2)
#define TWI_REG_U8_MA_DIR		(((0x20+6)<<8)|1)
#define TWI_REG_U8_MA_PWM		(((0x20+7)<<8)|1)

#define TWI_REG_S64_MA_ENC		(((0x30+0)<<8)|8)
#define TWI_REG_S16_MA_ENC		(((0x30+8)<<8)|2)

// Motor B
#define TWI_REG_S16_MB_PID_SP	(((0x40+0)<<8)|2)
#define TWI_REG_S16_MB_PID_PV	(((0x40+2)<<8)|2)
#define TWI_REG_S16_MB_PID_OV	(((0x40+4)<<8)|2)
#define TWI_REG_U8_MB_DIR		(((0x40+6)<<8)|1)
#define TWI_REG_U8_MB_PWM		(((0x40+7)<<8)|1)

#define TWI_REG_S64_MB_ENC		(((0x50+0)<<8)|8)
#define TWI_REG_S16_MB_ENC		(((0x50+8)<<8)|2)

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ROS2MowerMotorCtrl : public rclcpp::Node
{
private:
  int i2cfd;

  std_msgs::msg::UInt64 msg_U64_CLOCK_US;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pub_U64_CLOCK_US;
  std_msgs::msg::Int64 msg_S64_MA_ENC;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_S64_MA_ENC;
  std_msgs::msg::Int64 msg_S64_MB_ENC;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_S64_MB_ENC;

  std_msgs::msg::Int16 msg_S16_MA_PID_SP;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_S16_MA_PID_SP;
  std_msgs::msg::Int16 msg_S16_MA_PID_PV;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_S16_MA_PID_PV;
  std_msgs::msg::Int16 msg_S16_MA_PID_OV;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_S16_MA_PID_OV;

  std_msgs::msg::Int16 msg_S16_MB_PID_SP;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_S16_MB_PID_SP;
  std_msgs::msg::Int16 msg_S16_MB_PID_PV;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_S16_MB_PID_PV;
  std_msgs::msg::Int16 msg_S16_MB_PID_OV;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_S16_MB_PID_OV;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_set_MODE_OFF;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_set_MODE_TEST_01;
  rclcpp::Service<r2m_sensors::srv::SetMotorPID>::SharedPtr server_set_MOTOR_PID;

public:
  ROS2MowerMotorCtrl()
  : Node("r2m_motorctrl")
  {
	i2cfd = io_i2c_open(10);
    timer_ = create_wall_timer(100ms, std::bind(&ROS2MowerMotorCtrl::timer_callback, this));

    pub_U64_CLOCK_US = create_publisher<std_msgs::msg::UInt64>("motorctrl/u64_clock_us", 500);
    pub_S64_MA_ENC = create_publisher<std_msgs::msg::Int64>("motorctrl/s64_ma_enc", 100);
    pub_S64_MB_ENC = create_publisher<std_msgs::msg::Int64>("motorctrl/s64_mb_enc", 100);

    pub_S16_MA_PID_SP = create_publisher<std_msgs::msg::Int16>("motorctrl/s16_ma_sp", 100);
    pub_S16_MA_PID_PV = create_publisher<std_msgs::msg::Int16>("motorctrl/s16_ma_pv", 100);
    pub_S16_MA_PID_OV = create_publisher<std_msgs::msg::Int16>("motorctrl/s16_ma_ov", 100);

    pub_S16_MB_PID_SP = create_publisher<std_msgs::msg::Int16>("motorctrl/s16_mb_sp", 100);
    pub_S16_MB_PID_PV = create_publisher<std_msgs::msg::Int16>("motorctrl/s16_mb_pv", 100);
    pub_S16_MB_PID_OV = create_publisher<std_msgs::msg::Int16>("motorctrl/s16_mb_ov", 100);

    server_set_MODE_TEST_01 = create_service<std_srvs::srv::Empty>("r2m_motorctrl/MODE_TEST_01",
    [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response) -> void
    {
      set_MODE_TEST_01(request_header,request,response);
    });

    server_set_MODE_OFF = create_service<std_srvs::srv::Empty>("r2m_motorctrl/MODE_OFF",
    [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response) -> void
    {
      set_MODE_OFF(request_header,request,response);
    });

    server_set_MODE_OFF = create_service<std_srvs::srv::Empty>("r2m_motorctrl/MODE_OFF",
    [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response) -> void
    {
      set_MODE_OFF(request_header,request,response);
    });

    server_set_MOTOR_PID = create_service<r2m_sensors::srv::SetMotorPID>("r2m_motorctrl/SET_MOTOR_PID",
    [this](const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<r2m_sensors::srv::SetMotorPID::Request> request,
      std::shared_ptr<r2m_sensors::srv::SetMotorPID::Response> response) -> void
    {
      set_MOTOR_PID(request_header,request,response);
    });

    // go into MODE_PID mode
    I2C_SET_UINT16(i2cfd,TWI_REG_U16_MODE>>8,2 /*MODE_PID*/);
    I2C_SET_UINT16(i2cfd,TWI_REG_S16_MA_PID_SP>>8,0);
    I2C_SET_UINT16(i2cfd,TWI_REG_S16_MB_PID_SP>>8,0);

  }

  ~ROS2MowerMotorCtrl()
  {
    RCLCPP_INFO(this->get_logger(), "~ROS2MowerMotorCtrl()...");

    // go into MODE_OFF mode
    I2C_SET_UINT16(i2cfd,TWI_REG_U16_MODE>>8,0 /*MODE_OFF*/);

    RCLCPP_INFO(this->get_logger(), "~ROS2MowerMotorCtrl() ... done");
  }

  void set_MODE_OFF(const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
      RCLCPP_INFO(this->get_logger(), "set_MODE_OFF");
      (void)request_header;
      (void)request;
      (void)response;
      I2C_SET_UINT16(i2cfd,TWI_REG_U16_MODE>>8,0 /*MODE_OFF*/);
    };

    void set_MODE_TEST_01(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
      {
        RCLCPP_INFO(this->get_logger(), "set_MODE_TEST_01");
        (void)request_header;
        (void)request;
        (void)response;
        // go into MODE_TEST_01 mode
        I2C_SET_UINT16(i2cfd,TWI_REG_U16_MODE>>8,4 /*MODE_TEST_01*/);
      };

      void set_MOTOR_PID(const std::shared_ptr<rmw_request_id_t> request_header,
          const std::shared_ptr<r2m_sensors::srv::SetMotorPID::Request> request,
          std::shared_ptr<r2m_sensors::srv::SetMotorPID::Response> response)
        {
          (void)request_header;
          (void)response;

          int16_t i_p = (request->p*128.0);
          int16_t i_i = (request->i*128.0);
          int16_t i_d = (request->d*128.0);

          RCLCPP_INFO(this->get_logger(), "set_MOTOR_PID(p=%f(%d),i=%f(%d),d=%f(%d))",
          request->p,i_p,
          request->i,i_i,
          request->d,i_d);

          uint8_t data[1+6] = {TWI_REG_S16_PID_K_PID>>8,
            (uint8_t)((i_p>>0)&0xff),(uint8_t)((i_p>>8)&0xff),
            (uint8_t)((i_i>>0)&0xff),(uint8_t)((i_i>>8)&0xff),
            (uint8_t)((i_d>>0)&0xff),(uint8_t)((i_d>>8)&0xff)};
          io_i2c_write_bytes(i2cfd,data,sizeof(data));
        };

private:
  void timer_callback()
  {
    //RCLCPP_INFO(this->get_logger(), "timer_callback() ...");
    READ_AND_PUB_UINT64(i2cfd,TWI_REG_U64_CLOCK_US>>8,msg_U64_CLOCK_US,pub_U64_CLOCK_US);
    READ_AND_PUB_INT64(i2cfd,TWI_REG_S64_MA_ENC>>8,msg_S64_MA_ENC,pub_S64_MA_ENC);
    READ_AND_PUB_INT64(i2cfd,TWI_REG_S64_MB_ENC>>8,msg_S64_MB_ENC,pub_S64_MB_ENC);

    READ_AND_PUB_INT16(i2cfd,TWI_REG_S16_MA_PID_SP>>8,msg_S16_MA_PID_SP,pub_S16_MA_PID_SP);
    READ_AND_PUB_INT16(i2cfd,TWI_REG_S16_MA_PID_PV>>8,msg_S16_MA_PID_PV,pub_S16_MA_PID_PV);
    READ_AND_PUB_INT16(i2cfd,TWI_REG_S16_MA_PID_OV>>8,msg_S16_MA_PID_OV,pub_S16_MA_PID_OV);

    READ_AND_PUB_INT16(i2cfd,TWI_REG_S16_MB_PID_SP>>8,msg_S16_MB_PID_SP,pub_S16_MB_PID_SP);
    READ_AND_PUB_INT16(i2cfd,TWI_REG_S16_MB_PID_PV>>8,msg_S16_MB_PID_PV,pub_S16_MB_PID_PV);
    READ_AND_PUB_INT16(i2cfd,TWI_REG_S16_MB_PID_OV>>8,msg_S16_MB_PID_OV,pub_S16_MB_PID_OV);

    //RCLCPP_INFO(this->get_logger(), "timer_callback() ... done");
  }
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROS2MowerMotorCtrl>());
  rclcpp::shutdown();
  return 0;
}
