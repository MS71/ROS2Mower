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

#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"

#include "io_i2c.h"

extern "C" {
#include "epd/EPD_1in54.h"
#include "epd/GUI_Paint.h"
}

#define WHITE          0xFF

int EPD_BUSY_PIN = 0;
int EPD_RST_PIN = 1;
int EPD_DC_PIN = 2;
int EPD_CS_PIN = 3;

volatile int epfd = -1;
uint8_t epgpios = 0;

UBYTE DEV_Module_Init(void)
{
	epfd = io_i2c_open(12);
	//printf("DEV_Module_Init() %d\n",epfd);
	return 0;
}

/*------------------------------------------------------------------------------------------------------*/
void DEV_Digital_Write(UWORD Pin, UBYTE Value)
{
	//printf("DEV_Digital_Write(%d,%d)\n",Pin,Value);
	epgpios &= ~(1<<Pin);
	epgpios |= ((Value&1)<<Pin);
	io_i2c_write_bytes(epfd,&epgpios,1);
}

UBYTE DEV_Digital_Read(UWORD Pin)
{
    uint8_t tmp;
    io_i2c_read_bytes(epfd, &tmp, 1);
    //printf("DEV_Digital_Read(%d) => %d\n", Pin, ((tmp >> Pin) & 1));
    return ((tmp >> Pin) & 1);
}

void DEV_SPI_WriteByte(UBYTE Value)
{
 	//printf("DEV_SPI_WriteByte(%02x)\n",Value);
	uint8_t tmp[2];
	tmp[0] = epgpios;
	tmp[1] = Value;
	io_i2c_write_bytes(epfd,tmp,sizeof(tmp));
}

void DEV_SPI_Write_nByte(uint8_t* pData, uint32_t Len)
{
 	//printf("DEV_SPI_Write_nByte(Len=%d)\n",Len);
	uint8_t *tmp = (uint8_t*)malloc(1+Len);
	tmp[0] = epgpios;
	memcpy(&tmp[1],pData,Len);
	io_i2c_write_bytes(epfd,tmp,sizeof(tmp));
	free(tmp);
}

void EPD_1IN54_SendCommand(UBYTE Value)
{
	uint8_t tmp[2];
	tmp[0] = epgpios | (1<<6);
	tmp[1] = Value;
	io_i2c_write_bytes(epfd,tmp,sizeof(tmp));
}

void EPD_1IN54_SendData(UBYTE Value)
{
	uint8_t tmp[2];
	tmp[0] = epgpios | (1<<7);
	tmp[1] = Value;
	io_i2c_write_bytes(epfd,tmp,sizeof(tmp));
}

void DEV_Delay_ms(UDOUBLE xms)
{
	usleep(1000*xms);
}

void DEV_Module_Exit(void)
{
	io_i2c_close(epfd);
}

#if 0
int main(int argc, char * argv[])
{
  rclcpp::Clock ros_clock(RCL_ROS_TIME);
  rclcpp::Time t;
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("r2m_epd");

  if(DEV_Module_Init() != 0) {
    return -1;
  }

    printf("e-Paper Init and Clear...\r\n");
    EPD_1IN54_Init(EPD_1IN54_PART);
    EPD_1IN54_Clear();
    //DEV_Delay_ms(500);
	
    //Create a new image cache
    UBYTE *BlackImage;
    /* you have to edit the startup_stm32fxxx.s file and set a big enough heap size */
    UWORD Imagesize = ((EPD_1IN54_WIDTH % 8 == 0)? (EPD_1IN54_WIDTH / 8 ): (EPD_1IN54_WIDTH / 8 + 1)) * EPD_1IN54_HEIGHT;
    if((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL) {
        printf("Failed to apply for black memory...\r\n");
        return -1;
    }
	
    printf("Paint_NewImage\r\n");
    Paint_NewImage(BlackImage, EPD_1IN54_WIDTH, EPD_1IN54_HEIGHT, 0, WHITE);

  PAINT_TIME sPaint_time;
  sPaint_time.Hour = 12;
  sPaint_time.Min = 34;
  sPaint_time.Sec = 56;

  rclcpp::WallRate loop_rate(5000);
  t = ros_clock.now();
  while (rclcpp::ok()) {
	  if( ros_clock.now() > (t+) )
	  {
		t = ros_clock.now();
		
          sPaint_time.Sec = sPaint_time.Sec + 1;
        if (sPaint_time.Sec == 60) {
            sPaint_time.Min = sPaint_time.Min + 1;
            sPaint_time.Sec = 0;
            if (sPaint_time.Min == 60) {
                sPaint_time.Hour =  sPaint_time.Hour + 1;
                sPaint_time.Min = 0;
                if (sPaint_time.Hour == 24) {
                    sPaint_time.Hour = 0;
                    sPaint_time.Min = 0;
                    sPaint_time.Sec = 0;
                }
            }
        }
        Paint_ClearWindows(0, 0, 0 + Font24.Width * 7, 0 + Font24.Height, WHITE);
        Paint_DrawTime(0, 0, &sPaint_time, &Font24, WHITE, BLACK);
        EPD_1IN54_Display(BlackImage);

	  } 
  
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  //printf("Goto Sleep...\r\n");
  //EPD_1IN54_Sleep();

  // close 5V
  printf("close 5V, Module enters 0 power consumption ...\r\n");
  DEV_Module_Exit();

  rclcpp::shutdown();

  return 0;
}
#endif

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalTimer : public rclcpp::Node
{
private:
    UBYTE *BlackImage;
    PAINT_TIME sPaint_time;

public:
  MinimalTimer()
  : Node("r2m_epd")
  {
    timer_ = create_wall_timer(
      5000ms, std::bind(&MinimalTimer::timer_callback, this));
	// init EPD
	
    if(DEV_Module_Init() != 0) {
      return;
    }

    printf("e-Paper Init and Clear...\r\n");
    EPD_1IN54_Init(EPD_1IN54_FULL);
    EPD_1IN54_Clear();
	
    /* you have to edit the startup_stm32fxxx.s file and set a big enough heap size */
    UWORD Imagesize = ((EPD_1IN54_WIDTH % 8 == 0)? (EPD_1IN54_WIDTH / 8 ): (EPD_1IN54_WIDTH / 8 + 1)) * EPD_1IN54_HEIGHT;
    if((BlackImage = (UBYTE *)malloc(Imagesize)) == NULL) {
        printf("Failed to apply for black memory...\r\n");
        return;
    }
	
    printf("Paint_NewImage\r\n");
    Paint_NewImage(BlackImage, EPD_1IN54_WIDTH, EPD_1IN54_HEIGHT, 0, WHITE);

    sPaint_time.Hour = 12;
    sPaint_time.Min = 34;
    sPaint_time.Sec = 56;
	  
  }

  ~MinimalTimer()
  {
    RCLCPP_INFO(this->get_logger(), "~MinimalTimer()...");
	EPD_1IN54_Sleep();

	// close 5V
	printf("close 5V, Module enters 0 power consumption ...\r\n");
	DEV_Module_Exit();
    RCLCPP_INFO(this->get_logger(), "~MinimalTimer() ... done");
  }

private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "timer_callback() ...");
	
          sPaint_time.Sec = sPaint_time.Sec + 1;
        if (sPaint_time.Sec == 60) {
            sPaint_time.Min = sPaint_time.Min + 1;
            sPaint_time.Sec = 0;
            if (sPaint_time.Min == 60) {
                sPaint_time.Hour =  sPaint_time.Hour + 1;
                sPaint_time.Min = 0;
                if (sPaint_time.Hour == 24) {
                    sPaint_time.Hour = 0;
                    sPaint_time.Min = 0;
                    sPaint_time.Sec = 0;
                }
            }
        }
        Paint_ClearWindows(0, 0, 0 + Font24.Width * 7, 0 + Font24.Height, WHITE);
        Paint_DrawTime(0, 0, &sPaint_time, &Font24, WHITE, BLACK);
        EPD_1IN54_Display(BlackImage);
	
    RCLCPP_INFO(this->get_logger(), "timer_callback() ... done");
  }
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalTimer>());
  rclcpp::shutdown();
  return 0;
}