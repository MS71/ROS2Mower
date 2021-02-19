#ifndef _I2CHANDLER_H_
#define _I2CHANDLER_H_

#include "hwconfig.h"

void i2c_handler_init();
void i2c_set_cmd_vel(double x, double y, double z);
bool i2c_lock();
void i2c_release();

esp_err_t i2cnode_check(uint8_t i2caddr); 
esp_err_t i2cnode_read(uint8_t i2caddr, uint8_t regaddr, uint8_t *buf, uint32_t buflen);
esp_err_t i2cnode_read(uint8_t i2caddr, uint8_t *buf, uint32_t buflen);
esp_err_t i2cnode_write(uint8_t i2caddr, uint8_t regaddr, uint8_t *buf, uint32_t buflen); 
void i2cnode_set_u16(uint8_t i2caddr, uint8_t regaddr, uint16_t v) /* throw(int) */;
void i2cnode_init_motor();

uint16_t i2c_ubat_mV();
int16_t i2c_icharge_mA();
int16_t i2c_iout_mA();
int16_t i2c_isolar_mA();

void i2c_int(int level);
extern "C" {
void i2c_setpin_boot(int level);
}

#ifdef CONFIG_ENABLE_ROS2
#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/imu.h>
#define I2CROS2SENSORDATA_USE_NAV_MSG_ODOMETRY
#ifdef I2CROS2SENSORDATA_USE_NAV_MSG_ODOMETRY    
#include <nav_msgs/msg/odometry.h>
#endif
#define I2CROS2SENSORDATA_USE_GEOMETRY_MSG_POSE_2D
#ifdef I2CROS2SENSORDATA_USE_GEOMETRY_MSG_POSE_2D    
#include <geometry_msgs/msg/pose2_d.h>
#endif
#define noI2CROS2SENSORDATA_USE_GEOMETRY_MSG_TF
#ifdef I2CROS2SENSORDATA_USE_GEOMETRY_MSG_TF
#include <tf2_msgs/msg/tf_message.h>
#endif
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/transform_stamped.h>
typedef struct
{
    // ubat
    bool                                    msg_ubat_valid;
    std_msgs__msg__Float32                  msg_ubat;
    
    // imu
    bool                                    msg_imu_valid;
    sensor_msgs__msg__Imu                   msg_imu;

    // odom_tf
#ifdef I2CROS2SENSORDATA_USE_NAV_MSG_ODOMETRY    
    nav_msgs__msg__Odometry                 msg_odom_tf;
    bool                                    msg_odom_tf_valid;
#endif    
#ifdef I2CROS2SENSORDATA_USE_GEOMETRY_MSG_POSE_2D
    geometry_msgs__msg__Pose2D              msg_pose_2d;
    bool                                    msg_pose_2d_valid;
#endif
#ifdef I2CROS2SENSORDATA_USE_GEOMETRY_MSG_TF
    tf2_msgs__msg__TFMessage                msg_tf;
    bool                                    msg_tf_valid;
#endif
    // range msg
#define I2CROS2SENSORDATA_NUM_RANGE 12
    uint8_t                                 msg_range_trigger;
    uint32_t                                msg_range_valid[I2CROS2SENSORDATA_NUM_RANGE];
    sensor_msgs__msg__Range                 msg_range[I2CROS2SENSORDATA_NUM_RANGE];    
    
} I2CROS2SensorData;

I2CROS2SensorData* i2c_lock_data();
void i2c_release_data();
#endif

#endif /* _I2CHANDLER_H_ */
