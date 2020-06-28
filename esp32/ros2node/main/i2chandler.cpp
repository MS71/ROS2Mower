#define LOG_LOCAL_LEVEL ESP_LOG_INFO

#include <exception>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "../components/http_server/my_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#ifdef CONFIG_ENABLE_ROS2
#include <ros2esp.h>
#endif

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "ota_server.h"

#include "i2chandler.h"

#include "ssd1306.h"
#include "ssd1306_default_if.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"

static const char* TAG = "I2C";

#undef ENABLE_OLED

#define I2C_BUS_PORT 0
#define I2C_BUS_SDA 22
#define I2C_BUS_SCL 21
#define I2C_TIMEOUT_MS 2

#define PWRNODE_I2C_ADDR 0x09
#define MOTORNODE_I2C_ADDR 0x0a
//#define MOTOR_GEAR_N		16.0
#define MOTOR_GEAR_N		50.0
#define MOTOR_RPM(_rpm_) (int)(128.0*((_rpm_) * MOTOR_GEAR_N / 60.0))
#define MOTOR_MAX_RPM		120 
#define MOTOR_WHEEL_D 		0.175
#define MOTOR_START_RPM_L	 0.0
#define MOTOR_START_RPM_R        0.0

#define MOTOR_P			(0.15*128)
#define MOTOR_I			(0.01*128)
#define MOTOR_D			(0.01*128)
	
extern ip4_addr_t s_ip_addr;
struct SSD1306_Device I2CDisplay;

/**
 * @brief 
 * @param i2caddr
 * @param addr
 * @return 
 */
uint16_t i2cnode_get_u16(uint8_t i2caddr, uint8_t addr) throw(int)
{
    uint16_t v = 0;
    i2c_cmd_handle_t CommandHandle = NULL;
    if((CommandHandle = i2c_cmd_link_create()) != NULL) {
	uint8_t tmpbuf[2] = {};
	i2c_master_start(CommandHandle);
	i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(CommandHandle, addr, true);
	i2c_master_start(CommandHandle);
	i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_READ, true);
	i2c_master_read(CommandHandle, tmpbuf, 2, (i2c_ack_type_t)0x02);
	i2c_master_stop(CommandHandle);
	esp_err_t err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
	if(err != ESP_OK) {
	    ESP_LOGE(TAG, "i2cnode_get_u16 err=%02x", err);
	    i2c_cmd_link_delete(CommandHandle);
	    throw err;
	}
	i2c_cmd_link_delete(CommandHandle);
	v |= (tmpbuf[1] << 8);
	v |= (tmpbuf[0] << 0);
    }
    return v;
}

/**
 * @brief 
 * @param i2caddr
 * @param addr
 * @return 
 */
int16_t i2cnode_get_i16(uint8_t i2caddr, uint8_t addr) throw(int)
{
    int16_t v = 0;
    i2c_cmd_handle_t CommandHandle = NULL;
    if((CommandHandle = i2c_cmd_link_create()) != NULL) {
	uint8_t tmpbuf[2] = {};
	i2c_master_start(CommandHandle);
	i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(CommandHandle, addr, true);
	i2c_master_start(CommandHandle);
	i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_READ, true);
	i2c_master_read(CommandHandle, tmpbuf, 2, (i2c_ack_type_t)0x02);
	i2c_master_stop(CommandHandle);
	esp_err_t err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
	if(err != ESP_OK) {
	    ESP_LOGE(TAG, "i2cnode_get_i16 err=%02x", err);
	    i2c_cmd_link_delete(CommandHandle);
	    throw err;
	}
	i2c_cmd_link_delete(CommandHandle);
	v |= (tmpbuf[1] << 8);
	v |= (tmpbuf[0] << 0);
    }
    return v;
}

/**
 * @brief 
 * @param i2caddr
 * @param addr
 * @return 
 */
int8_t i2cnode_get_i8(uint8_t i2caddr, uint8_t addr) throw(int)
{
    int16_t v = 0;
    i2c_cmd_handle_t CommandHandle = NULL;
    if((CommandHandle = i2c_cmd_link_create()) != NULL) {
	uint8_t tmpbuf[1] = {};
	i2c_master_start(CommandHandle);
	i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(CommandHandle, addr, true);
	i2c_master_start(CommandHandle);
	i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_READ, true);
	i2c_master_read(CommandHandle, tmpbuf, 1, (i2c_ack_type_t)0x02);
	i2c_master_stop(CommandHandle);
	esp_err_t err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
	if(err != ESP_OK) {
	    ESP_LOGE(TAG, "i2cnode_get_i16 err=%02x", err);
	    i2c_cmd_link_delete(CommandHandle);
	    throw err;
	}
	i2c_cmd_link_delete(CommandHandle);
	v |= (tmpbuf[0] << 0);
    }
    return v;
}

/**
 * @brief 
 * @param i2caddr
 * @param addr
 * @return 
 */
int64_t i2cnode_get_i64(uint8_t i2caddr, uint8_t addr) throw(int)
{
    int64_t v = 0;
    i2c_cmd_handle_t CommandHandle = NULL;
    if((CommandHandle = i2c_cmd_link_create()) != NULL) {
	uint8_t tmpbuf[8] = {};
	i2c_master_start(CommandHandle);
	i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(CommandHandle, addr, true);
	i2c_master_start(CommandHandle);
	i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_READ, true);
	i2c_master_read(CommandHandle, tmpbuf, 8, (i2c_ack_type_t)0x02);
	i2c_master_stop(CommandHandle);
	esp_err_t err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
	if(err != ESP_OK) {
	    ESP_LOGE(TAG, "i2cnode_get_i64 err=%02x", err);
	    i2c_cmd_link_delete(CommandHandle);
	    throw err;
	}
	i2c_cmd_link_delete(CommandHandle);
	v |= ((int64_t)tmpbuf[7] << 56); 
	v |= ((int64_t)tmpbuf[6] << 48);
	v |= ((int64_t)tmpbuf[5] << 40);
	v |= ((int64_t)tmpbuf[4] << 32);
	v |= ((int64_t)tmpbuf[3] << 24);
	v |= ((int64_t)tmpbuf[2] << 16);
	v |= ((int64_t)tmpbuf[1] << 8);
	v |= ((int64_t)tmpbuf[0] << 0);
    }
    return v;
}

/**
 * @brief 
 * @param i2caddr
 * @param addr
 * @return 
 */
int64_t i2cnode_get_u64(uint8_t i2caddr, uint8_t addr) throw(int)
{
    uint64_t v = 0;
    i2c_cmd_handle_t CommandHandle = NULL;
    if((CommandHandle = i2c_cmd_link_create()) != NULL) {
	uint8_t tmpbuf[8] = {};
	i2c_master_start(CommandHandle);
	i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(CommandHandle, addr, true);
	i2c_master_start(CommandHandle);
	i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_READ, true);
	i2c_master_read(CommandHandle, tmpbuf, 8, (i2c_ack_type_t)0x02);
	i2c_master_stop(CommandHandle);
	esp_err_t err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
	if(err != ESP_OK) {
	    ESP_LOGE(TAG, "i2cnode_get_u64 err=%02x", err);
	    i2c_cmd_link_delete(CommandHandle);
	    throw err;
	}
	i2c_cmd_link_delete(CommandHandle);
	v |= ((uint64_t)tmpbuf[7] << 56);
	v |= ((uint64_t)tmpbuf[6] << 48);
	v |= ((uint64_t)tmpbuf[5] << 40);
	v |= ((uint64_t)tmpbuf[4] << 32);
	v |= ((uint64_t)tmpbuf[3] << 24);
	v |= ((uint64_t)tmpbuf[2] << 16);
	v |= ((uint64_t)tmpbuf[1] << 8);
	v |= ((uint64_t)tmpbuf[0] << 0);
    }
    return v;
}

/**
 * @brief 
 * @param i2caddr
 * @param addr
 * @return 
 */
uint32_t i2cnode_get_u32(uint8_t i2caddr, uint8_t addr) throw(int)
{
    uint32_t v = 0;
    i2c_cmd_handle_t CommandHandle = NULL;
    if((CommandHandle = i2c_cmd_link_create()) != NULL) {
	uint8_t tmpbuf[4] = {};
	i2c_master_start(CommandHandle);
	i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(CommandHandle, addr, true);
	i2c_master_start(CommandHandle);
	i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_READ, true);
	i2c_master_read(CommandHandle, tmpbuf, 4, (i2c_ack_type_t)0x02);
	i2c_master_stop(CommandHandle);
	esp_err_t err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
	if(err != ESP_OK) {
	    ESP_LOGE(TAG, "i2cnode_get_u32 err=%02x", err);
	    i2c_cmd_link_delete(CommandHandle);
	    throw err;
	}
	i2c_cmd_link_delete(CommandHandle);
	v |= (tmpbuf[3] << 24);
	v |= (tmpbuf[2] << 16);
	v |= (tmpbuf[1] << 8);
	v |= (tmpbuf[0] << 0);
    }
    return v;
}

/**
 * @brief 
 * @param i2caddr
 * @param addr
 * @param v
 */
void i2cnode_set_i16(uint8_t i2caddr, uint8_t addr, int16_t v) throw(int)
{
    i2c_cmd_handle_t CommandHandle = NULL;
    if((CommandHandle = i2c_cmd_link_create()) != NULL) {
	i2c_master_start(CommandHandle);
	i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(CommandHandle, addr, true);
	i2c_master_write_byte(CommandHandle, (v >> 0) & 0xff, true);
	i2c_master_write_byte(CommandHandle, (v >> 8) & 0xff, true);
	i2c_master_stop(CommandHandle);
	esp_err_t err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
	if(err != ESP_OK) {
	    ESP_LOGE(TAG, "i2cnode_set_i16 err=%02x", err);
	    i2c_cmd_link_delete(CommandHandle);
	    throw err;
	}
	i2c_cmd_link_delete(CommandHandle);
    }
}

/**
 * @brief 
 * @param i2caddr
 * @param addr
 * @param v
 */
void i2cnode_set_u16(uint8_t i2caddr, uint8_t addr, uint16_t v) throw(int)
{
    i2c_cmd_handle_t CommandHandle = NULL;
    if((CommandHandle = i2c_cmd_link_create()) != NULL) {
	i2c_master_start(CommandHandle);
	i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(CommandHandle, addr, true);
	i2c_master_write_byte(CommandHandle, (v >> 0) & 0xff, true);
	i2c_master_write_byte(CommandHandle, (v >> 8) & 0xff, true);
	i2c_master_stop(CommandHandle);
	esp_err_t err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
	if(err != ESP_OK) {
	    ESP_LOGE(TAG, "i2cnode_set_u16 err=%02x", err);
	    i2c_cmd_link_delete(CommandHandle);
	    throw err;
	}
	i2c_cmd_link_delete(CommandHandle);
    }
}

/**
 * @brief 
 * @param i2caddr
 * @param addr
 * @param v
 */
void i2cnode_set_u8(uint8_t i2caddr, uint8_t addr, uint8_t v) throw(int)
{
    i2c_cmd_handle_t CommandHandle = NULL;
    if((CommandHandle = i2c_cmd_link_create()) != NULL) {
	i2c_master_start(CommandHandle);
	i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(CommandHandle, addr, true);
	i2c_master_write_byte(CommandHandle, (v >> 0) & 0xff, true);
	i2c_master_stop(CommandHandle);
	esp_err_t err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
	if(err != ESP_OK) {
	    ESP_LOGE(TAG, "i2cnode_set_u8 err=%02x", err);
	    i2c_cmd_link_delete(CommandHandle);
	    throw err;
	}
	i2c_cmd_link_delete(CommandHandle);
    }
}

/**
 * @brief 
 * @param Address
 * @param IsCommand
 * @param Data
 * @param DataLength
 * @return 
 */
static bool I2CDefaultWriteBytes(int Address, bool IsCommand, const uint8_t* Data, size_t DataLength)
{
	static const int SSD1306_I2C_COMMAND_MODE = 0x80;
	static const int SSD1306_I2C_DATA_MODE = 0x40;
    i2c_cmd_handle_t CommandHandle = NULL;
    static uint8_t ModeByte = 0;

    NullCheck(Data, return false);

    if((CommandHandle = i2c_cmd_link_create()) != NULL) {
	ModeByte = (IsCommand == true) ? SSD1306_I2C_COMMAND_MODE : SSD1306_I2C_DATA_MODE;

	ESP_ERROR_CHECK_NONFATAL(i2c_master_start(CommandHandle), return false);
	ESP_ERROR_CHECK_NONFATAL(
	    i2c_master_write_byte(CommandHandle, (Address << 1) | I2C_MASTER_WRITE, true), return false);
	ESP_ERROR_CHECK_NONFATAL(i2c_master_write_byte(CommandHandle, ModeByte, true), return false);
	ESP_ERROR_CHECK_NONFATAL(i2c_master_write(CommandHandle, (uint8_t*)Data, DataLength, true), return false);
	ESP_ERROR_CHECK_NONFATAL(i2c_master_stop(CommandHandle), return false);

	esp_err_t err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
	if(err != ESP_OK) {
	    ESP_LOGE(TAG, "I2CDefaultWriteBytes err=%02x", err);
	}
	i2c_cmd_link_delete(CommandHandle);
    }

    return true;
}

/**
 * @brief 
 * @param Display
 * @param Command
 * @return 
 */
static bool I2CDefaultWriteCommand(struct SSD1306_Device* Display, SSDCmd Command)
{
    uint8_t CommandByte = (uint8_t)Command;

    NullCheck(Display, return false);
    return I2CDefaultWriteBytes(Display->Address, true, (const uint8_t*)&CommandByte, 1);
}

/**
 * @brief 
 * @param Display
 * @param Data
 * @param DataLength
 * @return 
 */
static bool I2CDefaultWriteData(struct SSD1306_Device* Display, const uint8_t* Data, size_t DataLength)
{
    NullCheck(Display, return false);
    NullCheck(Data, return false);

    return I2CDefaultWriteBytes(Display->Address, false, Data, DataLength);
}

/**
 * @brief 
 * @param Display
 * @return 
 */
static bool I2CDefaultReset(struct SSD1306_Device* Display)
{
    return true;
}

#ifdef CONFIG_ENABLE_ROS2
#define ROS2_NODENAME "ros2mower"

ros2::Publisher<std_msgs::String>* string_pub = NULL;
ros2::Publisher<std_msgs::Float32>* pub_ubat = NULL;
ros2::Publisher<std_msgs::Float32>* pub_isolar = NULL;
ros2::Publisher<std_msgs::Float32>* pub_iout = NULL;
ros2::Publisher<std_msgs::Float32>* pub_icharge = NULL;

ros2::Publisher<std_msgs::Int64>* pub_motor_l_enc = NULL;
ros2::Publisher<std_msgs::Int16>* pub_motor_l_pid_sv = NULL;
ros2::Publisher<std_msgs::Int16>* pub_motor_l_pid_ov = NULL;
ros2::Publisher<std_msgs::Int16>* pub_motor_l_pid_pv = NULL;

ros2::Publisher<std_msgs::Int64>* pub_motor_r_enc = NULL;
ros2::Publisher<std_msgs::Int16>* pub_motor_r_pid_sv = NULL;
ros2::Publisher<std_msgs::Int16>* pub_motor_r_pid_ov = NULL;
ros2::Publisher<std_msgs::Int16>* pub_motor_r_pid_pv = NULL;

ros2::Subscriber<geometry_msgs::Twist>* sub_cmdvel = NULL;

/**
 * @brief 
 * @param linear_x
 * @param linear_y
 * @param angular_z
 */
void cmd_vel_callback2(double linear_x, double linear_y, double angular_z)
{
    double wheel0_speed = 0;
    double wheel1_speed = 0;

    double wheelbase_ = 0.112;

    // *** Compute the current wheel speeds ***
    // First compute the Robot's linear and angular speeds
    double xspeed = linear_x;
    double yspeed = linear_y;
    double linear_speed = sqrt(xspeed * xspeed + yspeed * yspeed);
    double angular_speed = angular_z;

    if(linear_x >= 0) {
	// robot is moving forward
	wheel0_speed = linear_speed + angular_speed * wheelbase_ / 2.0;
	wheel1_speed = linear_speed - angular_speed * wheelbase_ / 2.0;
    } else {
	// robot is backing up
	wheel0_speed = -linear_speed + angular_speed * wheelbase_ / 2.0;
	wheel1_speed = -linear_speed - angular_speed * wheelbase_ / 2.0;
    }

    i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x20, 500 * wheel0_speed);
    i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x40, 500 * wheel1_speed);

    ESP_LOGI(TAG, "cmd_vel_callback() %f %f", wheel0_speed, wheel1_speed);
}

/**
 * @brief 
 * @param vel_cmd
 * @param arg
 */
void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd, void* arg)
{
    cmd_vel_callback2(vel_cmd.linear.x, vel_cmd.linear.y, vel_cmd.angular.z);
}
#endif

/**
 * @brief 
 * @param ros2node
 * @return 
 */
bool ros2ready(ros2::Node* ros2node)
{
    if(ros2node->getNodeRegisteredState()) {
	if(pub_ubat == NULL) {
	    ESP_LOGI(TAG, "ros2ready() => connected");
	    pub_ubat = ros2node->createPublisher<std_msgs::Float32>(ROS2_NODENAME "/ubat");
	    pub_isolar = ros2node->createPublisher<std_msgs::Float32>(ROS2_NODENAME "/isolar");
	    pub_iout = ros2node->createPublisher<std_msgs::Float32>(ROS2_NODENAME "/iout");
	    pub_icharge = ros2node->createPublisher<std_msgs::Float32>(ROS2_NODENAME "/icharge");
	    pub_motor_l_enc = ros2node->createPublisher<std_msgs::Int64>(ROS2_NODENAME "/motor_l_enc");
	    pub_motor_l_pid_sv = ros2node->createPublisher<std_msgs::Int16>(ROS2_NODENAME "/motor_l_pid_sv");
	    pub_motor_l_pid_ov = ros2node->createPublisher<std_msgs::Int16>(ROS2_NODENAME "/motor_l_pid_ov");
	    pub_motor_l_pid_pv = ros2node->createPublisher<std_msgs::Int16>(ROS2_NODENAME "/motor_l_pid_pv");
	    pub_motor_r_enc = ros2node->createPublisher<std_msgs::Int64>(ROS2_NODENAME "/motor_r_enc");
	    pub_motor_r_pid_sv = ros2node->createPublisher<std_msgs::Int16>(ROS2_NODENAME "/motor_r_pid_sv");
	    pub_motor_r_pid_ov = ros2node->createPublisher<std_msgs::Int16>(ROS2_NODENAME "/motor_r_pid_ov");
	    pub_motor_r_pid_pv = ros2node->createPublisher<std_msgs::Int16>(ROS2_NODENAME "/motor_r_pid_pv");
	    sub_cmdvel =
	        ros2node->createSubscriber<geometry_msgs::Twist>("cmd_vel", (ros2::CallbackFunc)cmd_vel_callback, NULL);
	}
	return true;
    } else {
	if(pub_ubat != NULL) {
	    ESP_LOGW(TAG, "ros2ready() => disconnected");
	    ros2node->deletePublisher(ROS2_NODENAME "/ubat");
	    pub_ubat = NULL;
	}
    }
    return false;
}

/**
 * @brief 
 * @param param
 */
static void i2c_task(void* param)
{
#ifdef CONFIG_ENABLE_ROS2
    static ros2::Node ros2node(ROS2_NODENAME);
#endif

    char tmpstr[64] = "";
    //int tmpcnt = 0;

#ifdef ENABLE_OLED
    SSD1306_Init_I2C(&I2CDisplay, 128, 64, 0x78 >> 1, -1, I2CDefaultWriteCommand, I2CDefaultWriteData, I2CDefaultReset);
    SSD1306_DisplayOn(&I2CDisplay);
    SSD1306_Clear(&I2CDisplay, SSD_COLOR_BLACK);
#endif

#if 0
		{			
			i2cnode_set_i16(MOTORNODE_I2C_ADDR,0x08,2);
			i2cnode_set_i16(MOTORNODE_I2C_ADDR,0x20,300);
			i2cnode_set_i16(MOTORNODE_I2C_ADDR,0x40,300);
			while(1) {
				try {
				int16_t mode = (int)i2cnode_get_u16(MOTORNODE_I2C_ADDR,0x08);
				if( mode != 2 ) vTaskDelay(100 / portTICK_PERIOD_MS);
#if 0
				ESP_LOGI(TAG, "%08x mode = %d %d %d",
					(int)i2cnode_get_u64(MOTORNODE_I2C_ADDR,0x00),
					mode,
					(int)i2cnode_get_u16(MOTORNODE_I2C_ADDR,0x20),
					(int)i2cnode_get_u16(MOTORNODE_I2C_ADDR,0x40));
#endif
				} catch (int err) {
				ESP_LOGE(TAG, "I2C exception err=0x%02x",
					err);	
				}

			}
		}
#endif

    try {
	i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x10, 30); // update TWI_MEM_SHDWNCNT 
	
	i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x20, (int16_t)MOTOR_P);
	i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x22, (int16_t)MOTOR_I);
	i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x24, (int16_t)MOTOR_D);
	i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x08, 2);
	i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x20, MOTOR_RPM(MOTOR_START_RPM_L));
	i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x40, MOTOR_RPM(MOTOR_START_RPM_R));
	//i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x0E, 7);
    } catch(int err) {
	ESP_LOGE(TAG, "I2C exception err=0x%02x", err);
    }

    while(1) {
	try {
	    uint16_t ubat_mV = i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x28);
	    int16_t isolar_mA = i2cnode_get_i16(PWRNODE_I2C_ADDR, 0x30);
	    int16_t iout_mA = i2cnode_get_i16(PWRNODE_I2C_ADDR, 0x32);
	    int16_t icharge_mA = i2cnode_get_i16(PWRNODE_I2C_ADDR, 0x34);

	    int64_t motor_l_enc = i2cnode_get_i64(MOTORNODE_I2C_ADDR, 0x30);
	    int64_t motor_r_enc = i2cnode_get_i64(MOTORNODE_I2C_ADDR, 0x50);

	    i2cnode_set_u16(MOTORNODE_I2C_ADDR, 0x0A, 0xffff); // TWI_REG_U16_AUTOBREAK

#ifdef ENABLE_OLED
	    int y = 0;
	    int s = 5;
	    SSD1306_Clear(&I2CDisplay, SSD_COLOR_BLACK);

	    SSD1306_SetFont(&I2CDisplay, &Font_droid_sans_mono_7x13);
	    uint32_t pm_rtc = i2cnode_get_u32(PWRNODE_I2C_ADDR, 0x04);
	    sprintf(tmpstr, "P:%d T:%03d:%02d:%02d:%02d", ((esp_ota_get_running_partition()->address) >> 20) & 0x7,
	        (pm_rtc / (60 * 60 * 24)) % 1000, (pm_rtc / (60 * 60)) % 24, (pm_rtc / 60) % 60, (pm_rtc) % 60);
	    SSD1306_FontDrawString(&I2CDisplay, 0, y, tmpstr, SSD_COLOR_WHITE);
	    y += (s + 7);

	    SSD1306_SetFont(&I2CDisplay, &Font_droid_sans_mono_7x13);
	    sprintf(tmpstr, "IP:%d.%d.%d.%d", (s_ip_addr.addr >> 0) & 0xff, (s_ip_addr.addr >> 8) & 0xff,
	        (s_ip_addr.addr >> 16) & 0xff, (s_ip_addr.addr >> 24) & 0xff);
	    SSD1306_FontDrawString(&I2CDisplay, 0, y, tmpstr, SSD_COLOR_WHITE);
	    y += (s + 7);

	    SSD1306_SetFont(&I2CDisplay, &Font_droid_sans_mono_13x24);
	    sprintf(tmpstr, "Ub:%2.2fV", ubat_mV / 1000.0);
	    SSD1306_FontDrawString(&I2CDisplay, 0, y, tmpstr, SSD_COLOR_WHITE);
	    y += (s + 13);

	    SSD1306_SetFont(&I2CDisplay, &Font_droid_sans_fallback_15x17);
	    sprintf(tmpstr, "I: %3d %3d %4d", isolar_mA, iout_mA, icharge_mA);
	    SSD1306_FontDrawString(&I2CDisplay, 0, y, tmpstr, SSD_COLOR_WHITE);
	    y += (s + 13);

	    SSD1306_Update(&I2CDisplay);
#endif

            wifi_ap_record_t wifi_info = {};
	    esp_wifi_sta_get_ap_info(&wifi_info);
#if 1
		ESP_LOGI(TAG, "rssi=%d Ubat=%2.1fV Isol=%1.1fmA Iout=%1.3fA  Motor: boot=%d t=%5.3f m=%d sp=(%d,%d) dir=(%d,%d) enc=(%d,%d)(%d,%d) pv=(%d,%d)", 
			(wifi_info.rssi),
			(float)i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x28)/1000.0,
			(float)i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x30)/1.0,
			(float)i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x32)/1000.0,
			i2cnode_get_u16(MOTORNODE_I2C_ADDR, 0x0C),
			(double)i2cnode_get_u64(MOTORNODE_I2C_ADDR, 0x00)/1000000.0,
			i2cnode_get_u16(MOTORNODE_I2C_ADDR, 0x08),
			i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x20),
			i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x40),
			(int)i2cnode_get_i8(MOTORNODE_I2C_ADDR, 0x26),
			(int)i2cnode_get_i8(MOTORNODE_I2C_ADDR, 0x46),
			(int)i2cnode_get_i64(MOTORNODE_I2C_ADDR, 0x30),
			(int)i2cnode_get_i64(MOTORNODE_I2C_ADDR, 0x50),
			(int)i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x38),
			(int)i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x58),
			(int)i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x22),
			(int)i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x42));
#endif

#if 0
		ESP_LOGI(TAG, "Power Supply Status: bootcnt=%d time=0x%08x pmsw=%d SHDWNCNT=%d PWRUPCNT=%d SHDWNREL=%d PWRUPREL=%d", 
			0,
			i2cnode_get_u32(PWRNODE_I2C_ADDR, 0x04),
			i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x08),
			i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x10),
			i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x12),
			i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x14),
			i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x16));
#endif

#ifdef CONFIG_ENABLE_ROS2
	    if(ros2ready(&ros2node)) {
			
		i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x10, 120); // update TWI_MEM_SHDWNCNT while ROS is connected
			
		{
		    auto msg_ubat = std_msgs::Float32();
		    msg_ubat.data = ubat_mV / 1000.0;
		    if(pub_ubat != NULL) pub_ubat->publish(&msg_ubat);
		}
		{
		    auto msg_i = std_msgs::Float32();
		    msg_i.data = isolar_mA / 1000.0;
		    if(pub_isolar != NULL) pub_isolar->publish(&msg_i);
		    msg_i.data = iout_mA / 1000.0;
		    if(pub_iout != NULL) pub_iout->publish(&msg_i);
		    msg_i.data = icharge_mA / 1000.0;
		    if(pub_icharge != NULL) pub_icharge->publish(&msg_i);
		}
		{
		    auto msg = std_msgs::Int64();
		    msg.data = motor_l_enc;
		    if(pub_motor_l_enc != NULL) pub_motor_l_enc->publish(&msg);
		    msg.data = motor_r_enc;
		    if(pub_motor_r_enc != NULL) pub_motor_r_enc->publish(&msg);
		}
		{
		    auto msg = std_msgs::Int16();
		    msg.data = i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x20);
		    if(pub_motor_l_pid_sv != NULL) pub_motor_l_pid_sv->publish(&msg);
		    msg.data = i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x24);
		    if(pub_motor_l_pid_ov != NULL) pub_motor_l_pid_ov->publish(&msg);
		    msg.data = i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x22);
		    if(pub_motor_l_pid_pv != NULL) pub_motor_l_pid_pv->publish(&msg);
		    msg.data = i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x40);
		    if(pub_motor_r_pid_sv != NULL) pub_motor_r_pid_sv->publish(&msg);
		    msg.data = i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x44);
		    if(pub_motor_r_pid_ov != NULL) pub_motor_r_pid_ov->publish(&msg);
		    msg.data = i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x42);
		    if(pub_motor_r_pid_pv != NULL) pub_motor_r_pid_pv->publish(&msg);
		}
	    }
	    ros2::spin(&ros2node);
#endif
	    i2cnode_set_u8(MOTORNODE_I2C_ADDR, 0x0f, 2); // Motor Driver Watchdog Reset
	} catch(int err) {
	    ESP_LOGE(TAG, "I2C exception err=0x%02x", err);
	}
	vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

/**
 * @brief 
 */
void i2c_handler_init()
{
    //esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "main() i2c init ...");
    static i2c_config_t Config;
    memset(&Config, 0, sizeof(i2c_config_t));
    Config.mode = I2C_MODE_MASTER;
    Config.sda_io_num = (gpio_num_t)I2C_BUS_SDA;
    // Config.sda_io_num = (gpio_num_t)13;
    Config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    Config.scl_io_num = (gpio_num_t)I2C_BUS_SCL;
    // Config.scl_io_num = (gpio_num_t)12;
    Config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    Config.master.clk_speed = 50000;
    i2c_param_config((i2c_port_t)I2C_BUS_PORT, &Config);
    i2c_driver_install((i2c_port_t)I2C_BUS_PORT, Config.mode, 0, 0, 0);
    //		i2c_set_timeout((i2c_port_t)I2C_BUS_PORT, (I2C_APB_CLK_FREQ / Config.master.clk_speed)*1024);
    i2c_set_timeout((i2c_port_t)I2C_BUS_PORT, I2C_APB_CLK_FREQ / 100); /* 10ms timeout */
    // i2c_set_start_timing((i2c_port_t)I2C_BUS_PORT,I2C_APB_CLK_FREQ/10000,I2C_APB_CLK_FREQ/10000);
    // i2c_set_stop_timing((i2c_port_t)I2C_BUS_PORT,I2C_APB_CLK_FREQ/10000,I2C_APB_CLK_FREQ/10000);

    xTaskCreate(&i2c_task, "i2c_task", 4096, NULL, 5, NULL);
}

/**
 * EOF
 */