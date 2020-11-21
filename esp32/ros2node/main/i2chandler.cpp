//#define LOG_LOCAL_LEVEL ESP_LOG_INFO

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

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_pm.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "ota_server.h"

#include "i2chandler.h"

#include "ssd1306.h"
#include "ssd1306_default_if.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"

#include "console.h"

#ifdef CONFIG_ENABLE_ROS2
#include "ros2node.h"
#endif

#ifdef CONFIG_ENABLE_I2C_BNO055
#include "BNO055ESP32.h"
#endif

#ifdef CONFIG_ENABLE_I2C_VL53L0X
#include "VL53L0X.h"
#define MAX_NUM_VL53L0X I2CROS2SENSORDATA_NUM_RANGE
#define CONST_VL53L0X_ANGLE_STEP 30
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t* pdata, uint32_t count);
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t* pdata, uint32_t count);
#endif

#ifdef CONFIG_ENABLE_I2C_VL53L1X
#include "VL53L1X.h"
#define MAX_NUM_VL53L1X 1
#define CONST_VL53L1X_ANGLE_STEP 30
#endif

static const char* TAG = "I2C";

#define MOTOR_P (0.30 * 128)
#define MOTOR_I (0.01 * 128)
#define MOTOR_D (0.01 * 128)

static struct
{
    bool       ready;
    
    struct
    {
        double linear_x;
        double linear_y;
        double angular_z;
        bool update;
    } cmd_vel;

    int64_t last_cmd_vel_time;

    SemaphoreHandle_t sem;

    uint16_t ubat_mV;
    int16_t isolar_mA;
    int16_t iout_mA;
    int16_t icharge_mA;

#ifdef CONFIG_ENABLE_ROS2
    I2CROS2SensorData ros2_data;
#endif

#ifdef CONFIG_ENABLE_I2C_VL53L0X
    VL53L0X* vl53l0x[MAX_NUM_VL53L0X];
    // uint16_t vl53l0x_data[MAX_NUM_VL53L0X];
#endif

#ifdef CONFIG_ENABLE_I2C_VL53L1X
    VL53L1X* vl53l1x[MAX_NUM_VL53L1X];
#endif

} i2c_md = {};

#ifdef CONFIG_ENABLE_I2C_BNO055
BNO055* bno055 = NULL;
bno055_calibration_t bno055_calib;
#endif

extern ip4_addr_t s_ip_addr;
struct SSD1306_Device I2CDisplay;

double ubat = 0.0;

/**
 * @brief
 * @return
 */
bool i2c_lock()
{
    if(i2c_md.sem == NULL)
        return false;

    if(xSemaphoreTake(i2c_md.sem, pdMS_TO_TICKS(100)))
    {
        return true;
    }
    ESP_LOGE(TAG, "i2c_lock => false");
    return false;
}

/**
 * @brief
 */
void i2c_release()
{
    xSemaphoreGive(i2c_md.sem);
}

#ifdef CONFIG_ENABLE_ROS2
/**
 * @brief
 * @param data
 * @return
 */
I2CROS2SensorData* i2c_lock_data()
{
    if( i2c_md.ready == false )
    {
        return NULL;
    }
    if(i2c_lock() == true)
    {
        return &i2c_md.ros2_data;
    }
    return NULL;
}

/**
 * @brief
 */
void i2c_release_data()
{
    i2c_release();
}
#endif

/**
 * @brief
 * @param i2caddr
 * @return
 */
esp_err_t i2cnode_check(uint8_t i2caddr)
{
    esp_err_t err = ESP_FAIL;
    if(i2c_lock() == true)
    {
        i2c_cmd_handle_t CommandHandle = NULL;
        if((CommandHandle = i2c_cmd_link_create()) != NULL)
        {
            i2c_master_start(CommandHandle);
            i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(CommandHandle);
            err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(1));
            i2c_cmd_link_delete(CommandHandle);
        }
        i2c_release();
        return err;
    }
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief
 * @param i2caddr
 * @param regaddr
 * @param buf
 * @param buflen
 * @return
 */
esp_err_t i2cnode_read(uint8_t i2caddr, uint8_t regaddr, uint8_t* buf, uint32_t buflen)
{
    esp_err_t err = ESP_OK;
    if(i2c_lock())
    {
#if 0
        if( i2caddr == 0x10 )
        {
            i2c_set_period((i2c_port_t)I2C_BUS_PORT,
                I2C_APB_CLK_FREQ/(2*I2C_BUS_CLOCK_SLOW),
                I2C_APB_CLK_FREQ/(2*I2C_BUS_CLOCK_SLOW));
        }
        else
        {
              i2c_set_period((i2c_port_t)I2C_BUS_PORT,
                I2C_APB_CLK_FREQ/(2*I2C_BUS_CLOCK_SLOW),
                I2C_APB_CLK_FREQ/(2*I2C_BUS_CLOCK_SLOW));
        }
 #endif 
        i2c_cmd_handle_t CommandHandle = NULL;
        if((CommandHandle = i2c_cmd_link_create()) != NULL)
        {
            i2c_master_start(CommandHandle);
            i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(CommandHandle, regaddr, true);
            i2c_master_start(CommandHandle);
            i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_READ, true);
            i2c_master_read(CommandHandle, buf, buflen, I2C_MASTER_LAST_NACK);
            i2c_master_stop(CommandHandle);
            err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            if(err != ESP_OK)
            {
                i2c_setpin_boot(1);
                ESP_LOGE(TAG, "i2cnode_read i2caddr=0x%02x regaddr=0x%02x err=0x%02x", i2caddr, regaddr, err);
                i2c_setpin_boot(0);
            }
            i2c_cmd_link_delete(CommandHandle);
        }
        i2c_release();
        return err;
    }
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief
 * @param i2caddr
 * @param regaddr
 * @param buf
 * @param buflen
 * @return
 */
esp_err_t i2cnode_write(uint8_t i2caddr, uint8_t regaddr, uint8_t* buf, uint32_t buflen)
{
    esp_err_t err = ESP_OK;
    if(i2c_lock())
    {
#if 0
        if( i2caddr == 0x10 )
        {
            i2c_set_period((i2c_port_t)I2C_BUS_PORT,
                I2C_APB_CLK_FREQ/(2*I2C_BUS_CLOCK_SLOW),
                I2C_APB_CLK_FREQ/(2*I2C_BUS_CLOCK_SLOW));
        }
        else
        {
              i2c_set_period((i2c_port_t)I2C_BUS_PORT,
                I2C_APB_CLK_FREQ/(2*I2C_BUS_CLOCK_SLOW),
                I2C_APB_CLK_FREQ/(2*I2C_BUS_CLOCK_SLOW));
        }
#endif
        i2c_cmd_handle_t CommandHandle = NULL;
        if((CommandHandle = i2c_cmd_link_create()) != NULL)
        {
            i2c_master_start(CommandHandle);
            i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(CommandHandle, regaddr, true);
            i2c_master_write(CommandHandle, buf, buflen, (i2c_ack_type_t)0x02);
            i2c_master_stop(CommandHandle);
            esp_err_t err =
                i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            if(err != ESP_OK)
            {
                ESP_LOGE(TAG, "i2cnode_write i2caddr=0x%02x regaddr=0x%02x buf[%d]=%02x%02x%02x%02x err=0x%02x tout=%d",
                    i2caddr, regaddr, buflen, buf[0], buf[1], buf[2], buf[3], err, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            }
            i2c_cmd_link_delete(CommandHandle);
        }
        i2c_release();
        return err;
    }
    return ESP_ERR_TIMEOUT;
}

#ifdef CONFIG_ENABLE_I2C_VL53L0X
/**
 * @brief
 * @param esp_err
 * @return
 */
static VL53L0X_Error esp_to_vl53l0x_error(esp_err_t esp_err)
{
    switch(esp_err)
    {
    case ESP_OK:
        return VL53L0X_ERROR_NONE;
    case ESP_ERR_INVALID_ARG:
        return VL53L0X_ERROR_INVALID_PARAMS;
    case ESP_FAIL:
    case ESP_ERR_INVALID_STATE:
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    case ESP_ERR_TIMEOUT:
        return VL53L0X_ERROR_TIME_OUT;
    default:
        return VL53L0X_ERROR_UNDEFINED;
    }
}

/**
 * Writes the supplied byte buffer to the device
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   pdata     Pointer to uint8_t buffer containing the data to be written
 * @param   count     Number of bytes in the supplied byte buffer
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t* pdata, uint32_t count)
{
    VL53L0X_Error err = esp_to_vl53l0x_error(i2cnode_write(Dev->i2c_address, index, pdata, count));
    if(err != VL53L0X_ERROR_NONE)
        ESP_LOGW("X", "VL53L0X_WriteMulti %d", err);
    return err;
}

/**
 * Reads the requested number of bytes from the device
 * @param   Dev       Device Handle
 * @param   index     The register index
 * @param   pdata     Pointer to the uint8_t buffer to store read data
 * @param   count     Number of uint8_t's to read
 * @return  VL53L0X_ERROR_NONE        Success
 * @return  "Other error code"    See ::VL53L0X_Error
 */
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t* pdata, uint32_t count)
{
    VL53L0X_Error err = esp_to_vl53l0x_error(i2cnode_read(Dev->i2c_address, index, pdata, count));
    if(err != VL53L0X_ERROR_NONE)
        ESP_LOGW("X", "VL53L0X_ReadMulti %d", err);
    return err;
}
#endif /* CONFIG_ENABLE_I2C_VL53L0X */

/**
 * @brief
 * @param i2caddr
 * @param regaddr
 * @param buf
 * @param buflen
 * @return
 */
esp_err_t i2cnode_read(uint8_t i2caddr, uint8_t* buf, uint32_t buflen)
{
    esp_err_t err = ESP_OK;
    if(i2c_lock())
    {
        i2c_cmd_handle_t CommandHandle = NULL;
        if((CommandHandle = i2c_cmd_link_create()) != NULL)
        {
            i2c_master_start(CommandHandle);
            i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_READ, true);
            i2c_master_read(CommandHandle, buf, buflen, I2C_MASTER_LAST_NACK);
            i2c_master_stop(CommandHandle);
            err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            if(err != ESP_OK)
            {
                ESP_LOGE(TAG, "i2cnode_read i2caddr=0x%02x buflen=%d err=0x%02x", i2caddr, buflen, err);
            }
            i2c_cmd_link_delete(CommandHandle);
        }
        i2c_release();
    }
    return err;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @return
 */
uint64_t i2cnode_get_u64(uint8_t i2caddr, uint8_t regaddr) /* throw(int) */
{
    esp_err_t err;
    uint64_t v = 0;
    uint8_t tmpbuf[8] = {};
    if((err = i2cnode_read(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 1 << 24;
    }
    v |= ((uint64_t)tmpbuf[7] << 56);
    v |= ((uint64_t)tmpbuf[6] << 48);
    v |= ((uint64_t)tmpbuf[5] << 40);
    v |= ((uint64_t)tmpbuf[4] << 32);
    v |= ((uint64_t)tmpbuf[3] << 24);
    v |= ((uint64_t)tmpbuf[2] << 16);
    v |= ((uint64_t)tmpbuf[1] << 8);
    v |= ((uint64_t)tmpbuf[0] << 0);
    return v;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @return
 */
uint32_t i2cnode_get_u32(uint8_t i2caddr, uint8_t regaddr) /* throw(int) */
{
    esp_err_t err;
    uint32_t v = 0;
    uint8_t tmpbuf[8] = {};
    if((err = i2cnode_read(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 2 << 24;
    }
    v |= ((uint32_t)tmpbuf[3] << 24);
    v |= ((uint32_t)tmpbuf[2] << 16);
    v |= ((uint32_t)tmpbuf[1] << 8);
    v |= ((uint32_t)tmpbuf[0] << 0);
    return v;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @return
 */
uint16_t i2cnode_get_u16(uint8_t i2caddr, uint8_t regaddr) /* throw(int) */
{
    esp_err_t err;
    uint16_t v = 0;
    uint8_t tmpbuf[2] = {};
    if((err = i2cnode_read(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 3 << 24;
    }
    v |= ((uint16_t)tmpbuf[1] << 8);
    v |= ((uint16_t)tmpbuf[0] << 0);
    return v;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @return
 */
uint8_t i2cnode_get_u8(uint8_t i2caddr, uint8_t regaddr) /* throw(int) */
{
    esp_err_t err;
    uint16_t v = 0;
    uint8_t tmpbuf[1] = {};
    if((err = i2cnode_read(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 4 << 24;
    }
    v |= (tmpbuf[0] << 0);
    return v;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @return
 */
int64_t i2cnode_get_i64(uint8_t i2caddr, uint8_t regaddr) /* throw(int) */
{
    esp_err_t err;
    int64_t v = 0;
    uint8_t tmpbuf[8] = {};
    if((err = i2cnode_read(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 5 << 24;
    }
    v |= ((int64_t)tmpbuf[7] << 56);
    v |= ((int64_t)tmpbuf[6] << 48);
    v |= ((int64_t)tmpbuf[5] << 40);
    v |= ((int64_t)tmpbuf[4] << 32);
    v |= ((int64_t)tmpbuf[3] << 24);
    v |= ((int64_t)tmpbuf[2] << 16);
    v |= ((int64_t)tmpbuf[1] << 8);
    v |= ((int64_t)tmpbuf[0] << 0);
    return v;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @return
 */
int16_t i2cnode_get_i16(uint8_t i2caddr, uint8_t regaddr) /* throw(int) */
{
    esp_err_t err;
    int16_t v = 0;
    uint8_t tmpbuf[2] = {};
    if((err = i2cnode_read(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 6 << 24;
    }
    v |= (tmpbuf[1] << 8);
    v |= (tmpbuf[0] << 0);
    return v;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @return
 */
int8_t i2cnode_get_i8(uint8_t i2caddr, uint8_t regaddr) /* throw(int) */
{
    esp_err_t err;
    int8_t v = 0;
    uint8_t tmpbuf[2] = {};
    if((err = i2cnode_read(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 7 << 24;
    }
    v |= (tmpbuf[0] << 0);
    return v;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @param v
 */
void i2cnode_set_u16(uint8_t i2caddr, uint8_t regaddr, uint16_t v) /* throw(int) */
{
    esp_err_t err;
    uint8_t tmpbuf[2] = {};
    tmpbuf[0] = (v >> 0) & 0xff;
    tmpbuf[1] = (v >> 8) & 0xff;
    if((err = i2cnode_write(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 8 << 24;
    }
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @param v
 */
void i2cnode_set_u8(uint8_t i2caddr, uint8_t regaddr, uint8_t v) /* throw(int) */
{
    esp_err_t err;
    uint8_t tmpbuf[1] = {};
    tmpbuf[0] = (v >> 0) & 0xff;
    if((err = i2cnode_write(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 9 << 24;
    }
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @param v
 */
void i2cnode_set_i16(uint8_t i2caddr, uint8_t regaddr, int16_t v) /* throw(int) */
{
    esp_err_t err;
    uint8_t tmpbuf[2] = {};
    tmpbuf[0] = (v >> 0) & 0xff;
    tmpbuf[1] = (v >> 8) & 0xff;
    if((err = i2cnode_write(i2caddr, regaddr, tmpbuf, sizeof(tmpbuf))) != ESP_OK)
    {
        throw err | (i2caddr << 16) | 10 << 24;
    }
}

#ifdef CONFIG_ENABLE_I2C_OLED
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
    uint8_t ModeByte = 0;
    ModeByte = (IsCommand == true) ? SSD1306_I2C_COMMAND_MODE : SSD1306_I2C_DATA_MODE;
    NullCheck(Data, return false);

    if((i2cnode_write(Address, ModeByte, (uint8_t*)Data, DataLength)) != ESP_OK)
    {
        return false;
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
#endif // CONFIG_ENABLE_I2C_OLED

/**
 * @brief
 */
void i2c_handle_cmd_vel()
{
    if(i2c_md.cmd_vel.update == true)
    {
        i2c_md.cmd_vel.update = false;

        i2c_md.last_cmd_vel_time = esp_timer_get_time();

        double wheel0_speed = 0;
        double wheel1_speed = 0;

        double wheelcircumference = WHEEL_DIAMETER * M_PI;
        double wheeldistcircumference = WHEEL_DISTANCE * M_PI;

        // *** Compute the current wheel speeds ***
        // First compute the Robot's linear and angular speeds
        double xspeed = i2c_md.cmd_vel.linear_x;
        double yspeed = i2c_md.cmd_vel.linear_y;
        double linear_speed = sqrt(xspeed * xspeed + yspeed * yspeed);
        double angular_speed = i2c_md.cmd_vel.angular_z * wheeldistcircumference / (2 * M_PI);

        if(xspeed >= 0)
        {
            // robot is moving forward
            wheel0_speed = (linear_speed + angular_speed) / wheelcircumference;
            wheel1_speed = (linear_speed - angular_speed) / wheelcircumference;
        }
        else
        {
            // robot is backing up
            wheel0_speed = (-linear_speed + angular_speed) / wheelcircumference;
            wheel1_speed = (-linear_speed - angular_speed) / wheelcircumference;
        }

#if 0
        /* limit to max RPM
         */
        if( MOTOR_RPS(wheel0_speed) > MOTOR_MAX_RPS) {
            wheel1_speed *= MOTOR_MAX_RPS / MOTOR_RPS(wheel0_speed);
            wheel0_speed *= MOTOR_MAX_RPS / MOTOR_RPS(wheel0_speed);
        }
        if( MOTOR_RPS(wheel1_speed) > MOTOR_MAX_RPS) {
            wheel0_speed *= MOTOR_MAX_RPS / MOTOR_RPS(wheel1_speed);
            wheel1_speed *= MOTOR_MAX_RPS / MOTOR_RPS(wheel1_speed);
        }
#endif

#ifdef CONFIG_ENABLE_I2C_MOTOR
#ifdef CONFIG_ROS2NODE_HW_ROS2MOWER
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x20, MOTOR_RPS(wheel0_speed));
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x40, MOTOR_RPS(wheel1_speed));

        ESP_LOGW(TAG, "i2c_handle_cmd_vel() %frps %frps %d %d", wheel0_speed, wheel1_speed, MOTOR_RPS(wheel0_speed),
            MOTOR_RPS(wheel1_speed));
#endif // CONFIG_ROS2NODE_HW_ROS2MOWER
#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
        uint8_t tmpbuf[4] = {};
        tmpbuf[0] = (MOTOR_RPS(wheel0_speed) >> 8) & 0xff;
        tmpbuf[1] = (MOTOR_RPS(wheel0_speed) >> 0) & 0xff;
        tmpbuf[2] = (MOTOR_RPS(wheel1_speed) >> 8) & 0xff;
        tmpbuf[3] = (MOTOR_RPS(wheel1_speed) >> 0) & 0xff;
        i2cnode_write(ZUMO_I2C_ADDR, CMD_MOTORS_SET_SPEED, tmpbuf, sizeof(tmpbuf));
        ESP_LOGE(TAG, "i2c_handle_cmd_vel() %f rps %f rps %d %d", wheel0_speed, wheel1_speed, MOTOR_RPS(wheel0_speed),
            MOTOR_RPS(wheel1_speed));
#endif // CONFIG_ROS2NODE_HW_ROS2ZUMO
#endif
    }
}

/**
 * @brief
 * @param x
 * @param y
 * @param z
 */
void i2c_set_cmd_vel(double x, double y, double z)
{
    i2c_md.cmd_vel.linear_x = x;
    i2c_md.cmd_vel.linear_y = y;
    i2c_md.cmd_vel.angular_z = z;
    i2c_md.cmd_vel.update = true;
}

/**
 * @brief
 * @return
 */
bool i2c_cmd_vel_active()
{
    if((i2c_md.last_cmd_vel_time != 0) && ((i2c_md.last_cmd_vel_time + 1000000 * 60) > esp_timer_get_time()))
    {
        return true;
    }

    return (i2c_md.cmd_vel.linear_x != 0.0) || (i2c_md.cmd_vel.linear_y != 0.0) || (i2c_md.cmd_vel.angular_z != 0.0);
}

/**
 * @brief
 */
void i2cnode_init_motor()
{
#ifdef CONFIG_ENABLE_I2C_MOTOR
#ifdef CONFIG_ROS2NODE_HW_ROS2MOWER
    try
    {
        i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x10, 60);    // update TWI_MEM_SHDWNCNT
        i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x18, 13000); // stay on with ubat>12.5V

        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x20, (int16_t)MOTOR_P);
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x22, (int16_t)MOTOR_I);
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x24, (int16_t)MOTOR_D);
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x08, 2);
        // i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x20, MOTOR_RPM(MOTOR_START_RPM_L));
        // i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x40, MOTOR_RPM(MOTOR_START_RPM_R));
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x20, MOTOR_RPM(MOTOR_START_RPM_L));
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x40, MOTOR_RPM(MOTOR_START_RPM_R));
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x0E, 0); /* int pulse */
    }
    catch(int err)
    {
        i2c_setpin_boot(1);
        ESP_LOGE(TAG, "I2C exception err=0x%02x", err);
        i2c_setpin_boot(0);
    }
#endif
#endif
}

uint16_t i2c_ubat_mV()
{
    return i2c_md.ubat_mV;
}

int16_t i2c_isolar_mA()
{
    return i2c_md.isolar_mA;
}

int16_t i2c_iout_mA()
{
    return i2c_md.iout_mA;
}

int16_t i2c_icharge_mA()
{
    return i2c_md.icharge_mA;
}

/**
 * @brief
 * @param enc_l
 * @param enc_r
 */
#ifdef CONFIG_ENABLE_ROS2
void i2c_handle_encoder(int16_t enc_l, int16_t enc_r)
{
    static int64_t time_ = 0;
    int64_t time = esp_timer_get_time(); /* time in us */

    //if(enc_l != 0 || enc_r != 0)
    {
        double dt = 0.000001 * (time - time_); /* s */
        time_ = time;

        /*
         * https://answers.ros.org/question/207392/generating-odom-message-from-encoder-ticks-for-robot_pose_ekf/
         * http://www.seattlerobotics.org/encoder/200610/Article3/IMU%20Odometry,%20by%20David%20Anderson.htm
         */
        static double x = 0;
        static double y = 0;
        static double th = 0;

        /*
         * Pololu Wheel Encoder
         */
        double DistancePerCount = (M_PI * WHEEL_DIAMETER) / (MOTOR_GEAR_N); /* [m] */

        // extract the wheel velocities from the tick signals count
        double v_left = (enc_l * DistancePerCount) / dt;  /* m/s */
        double v_right = (enc_r * DistancePerCount) / dt; /* m/s */

        double vx = ((v_right + v_left) / 2); /* m/s */
        // double vy = 0;                        /* m/s */
        double vth = ((v_right - v_left) /* m/s */ / WHEEL_DISTANCE /* m */);

        double delta_x = (vx * cos(th)) * dt;
        double delta_y = (vx * sin(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        // motor_speed_l = v_left;
        // motor_speed_r = v_right;
        // cmd_vel_motor_update();

        ESP_LOGD(TAG, "vHandleEncoderSteps(%d,%d) d=%f dT=%f v=(%f,%f) x=%f y=%f th=%f ...", enc_l, enc_r,
            DistancePerCount, dt, v_left, v_right, x, y, th);

#ifdef I2CROS2SENSORDATA_USE_NAV_MSG_ODOMETRY    
        i2c_md.ros2_data.msg_odom_tf.pose.pose.position.x = x;
        i2c_md.ros2_data.msg_odom_tf.pose.pose.position.y = y;
        i2c_md.ros2_data.msg_odom_tf.pose.pose.position.z = 0;
        i2c_md.ros2_data.msg_odom_tf.twist.twist.angular.x = 0;
        i2c_md.ros2_data.msg_odom_tf.twist.twist.angular.y = 0;
        i2c_md.ros2_data.msg_odom_tf.twist.twist.angular.z = th;
        
        struct timespec tv = { 0 };
        clock_gettime(CLOCK_MONOTONIC, &tv);
        i2c_md.ros2_data.msg_odom_tf.header.stamp.nanosec = tv.tv_nsec;
        i2c_md.ros2_data.msg_odom_tf.header.stamp.sec = tv.tv_sec;

        i2c_md.ros2_data.msg_odom_tf_valid = true;
        // RCSOFTCHECK(rcl_publish(&i2c_md.ros2_data.pub_odom_tf, &i2c_md.ros2_data.msg_odom_tf, NULL));
#endif
#ifdef I2CROS2SENSORDATA_USE_GEOMETRY_MSG_POSE_2D
    i2c_md.ros2_data.msg_pose_2d.x = x;
    i2c_md.ros2_data.msg_pose_2d.y = y;
    i2c_md.ros2_data.msg_pose_2d.theta = th;
    i2c_md.ros2_data.msg_pose_2d_valid = true;
#endif

#if 0
        i2c_md.ros2_data.msg_pose.x = 0.0;
        i2c_md.ros2_data.msg_pose.y = 0.0;
        i2c_md.ros2_data.msg_pose.z += delta_th;

		//RCSOFTCHECK(rcl_publish(&i2c_md.ros2_data.pub_pose, &i2c_md.ros2_data.msg_pose, NULL));

        i2c_md.ros2_data.msg_odom.x += delta_x;
        i2c_md.ros2_data.msg_odom.y += delta_y;
        i2c_md.ros2_data.msg_odom.z = 0.0;

		//RCSOFTCHECK(rcl_publish(&i2c_md.ros2_data.pub_odom, &i2c_md.ros2_data.msg_odom, NULL));
#endif
        ESP_LOGD(TAG, "vHandleEncoderSteps() ... done");
    }
}
#endif

#ifdef CONFIG_ENABLE_I2C_BNO055
void i2c_init_bno055()
{
    bno055 = new BNO055((i2c_port_t)I2C_BUS_PORT, I2C_BNO055_ADDR);
    if(bno055 != NULL)
    {
        try
        {
            ESP_LOGI(TAG, "i2c_init_bno055() init ...");
            bno055->begin(); // BNO055 is in CONFIG_MODE until it is changed
            bno055->setOprModeConfig();
            bno055->enableExternalCrystal();
            // bno.setSensorOffsets(storedOffsets);
            // bno055->setAxisRemap(BNO055_REMAP_CONFIG_P0, BNO055_REMAP_SIGN_P0); // see datasheet, section 3.4
            // bno055->setAxisRemap(BNO055_REMAP_CONFIG_P1, BNO055_REMAP_SIGN_P1); // see datasheet, section 3.4
            bno055->setAxisRemap(BNO055_REMAP_CONFIG_P2, BNO055_REMAP_SIGN_P2); // see datasheet, section 3.4
            // xxbno055->setAxisRemap(BNO055_REMAP_CONFIG_P3, BNO055_REMAP_SIGN_P3); // see datasheet, section 3.4

            bno055->setUnits(BNO055_UNIT_ACCEL_MS2, BNO055_UNIT_ANGULAR_RATE_RPS, BNO055_UNIT_EULER_DEGREES,
                BNO055_UNIT_TEMP_C, BNO055_DATA_FORMAT_ANDROID);

#if 0
			bno055->setAccelConfig(BNO055_CONF_ACCEL_RANGE_4G,
														 BNO055_CONF_ACCEL_BANDWIDTH_7_81HZ,
 													 	 BNO055_CONF_ACCEL_MODE_NORMAL);
#endif
            /* you can specify a PoWeRMode using:
                                    - setPwrModeNormal(); (Default on startup)
                                    - setPwrModeLowPower();
                                    - setPwrModeSuspend(); (while suspended bno055 must remain in CONFIG_MODE)
                                    */

            bno055->enableAccelSlowMotionInterrupt();
            bno055->enableAccelNoMotionInterrupt();
            bno055->enableAccelAnyMotionInterrupt();
            bno055->enableAccelHighGInterrupt();
            bno055->enableGyroAnyMotionInterrupt();
            bno055->disableGyroHRInterrupt();
            bno055->clearInterruptPin();

            bno055_offsets_t o = {};
            {
                nvs_handle my_handle;
                esp_err_t err = nvs_open("bno055", NVS_READWRITE, &my_handle);
                if(err == ESP_OK)
                {
                    size_t l = sizeof(o);
                    err = nvs_get_blob(my_handle, "bno055_offsets", &o, &l);
                    if(err == ESP_OK && l == sizeof(o))
                    {
                        ESP_LOGW(TAG, "i2c_init_bno055() offsets read from nvs");
                        bno055->setSensorOffsets(o);
                    }
                    nvs_close(my_handle);
                }
            }
            o = bno055->getSensorOffsets();
            bno055_calibration_t calib = bno055->getCalibration();
            ESP_LOGW(TAG, "i2c_init_bno055() SET calib(%d,%d,%d,%d) %d,%d,%d %d,%d,%d %d,%d,%d %d,%d", bno055_calib.sys,
                bno055_calib.gyro, bno055_calib.mag, bno055_calib.accel, o.accelOffsetX, o.accelOffsetY, o.accelOffsetZ,
                o.magOffsetX, o.magOffsetY, o.magOffsetZ, o.gyroOffsetX, o.gyroOffsetY, o.gyroOffsetZ, o.accelRadius,
                o.magRadius);
            (void)calib;

            bno055->setOprModeNdof();
            bno055->clearInterruptPin();

            // bno055->setOprModeNdof();
            ESP_LOGW(TAG, "i2c_init_bno055() BNO055 init done");
        }
        catch(BNO055BaseException& ex)
        {
            ESP_LOGI(TAG, "i2c_init_bno055() BNO055 exception %s", ex.what());
            bno055 = NULL;
        }
    }
}

/**
 * @brief
 */
void i2c_handle_bno055()
{
    try
    {
        if(bno055 != NULL)
        {
            try
            {
                bno055_system_error_t bno055_error = bno055->getSystemError();
                bno055_system_status_t bno055_status = bno055->getSystemStatus();
                bno055_interrupts_status_t irq_status = bno055->getInterruptsStatus();

                bno055->clearInterruptPin();

                if(bno055_status == BNO055_SYSTEM_STATUS_FUSION_ALGO_RUNNING &&
                    bno055_error == BNO055_SYSTEM_ERROR_NO_ERROR)
                {

                    // bno055_calibration_t calib = bno055->getCalibration();

#if 0
                    bno055_offsets_t bno055_offsets = bno055->getSensorOffsets();
                    {
                        nvs_handle my_handle;
                        bno055_offsets_t o = {};
                        esp_err_t err = nvs_open("bno055", NVS_READWRITE, &my_handle);
                        if(err == ESP_OK) {
                            size_t l = sizeof(o);
                            err = nvs_get_blob(my_handle, "bno055_offsets", &o, &l);
                            if(err == ESP_OK && l == sizeof(o)) {
                                if(memcpy(&o, &bno055_offsets, sizeof(bno055_offsets_t)) != 0) {
                                    ESP_LOGW(TAG,
                                        "i2c_handle_bno055() UPT calib(%d,%d,%d,%d) [%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]",
                                        bno055_calib.sys, bno055_calib.gyro, bno055_calib.mag, bno055_calib.accel,
                                        bno055_offsets.accelOffsetX, bno055_offsets.accelOffsetY,
                                        bno055_offsets.accelOffsetZ, bno055_offsets.magOffsetX,
                                        bno055_offsets.magOffsetY, bno055_offsets.magOffsetZ,
                                        bno055_offsets.gyroOffsetX, bno055_offsets.gyroOffsetY,
                                        bno055_offsets.gyroOffsetZ, bno055_offsets.accelRadius,
                                        bno055_offsets.magRadius);
                                    if(err == ESP_OK) {
                                        nvs_set_blob(
                                            my_handle, "bno055_offsets", &bno055_offsets, sizeof(bno055_offsets));
                                    }
                                }
                            }
                            nvs_close(my_handle);
                        }
                    }
#endif

                    bno055_quaternion_t quaternion = bno055->getQuaternion();
                    bno055_vector_t vector_angvel = bno055->getVectorGyroscope();
                    bno055_vector_t vector_linaccl = bno055->getVectorLinearAccel();
                    int8_t temperature = bno055->getTemp();

                    ESP_LOGD(TAG,
                        "i2c_handle_bno055() irq status %d %d %d %d %d error=0x%02x status=0x%02x temp=%d x=%f y=%f "
                        "z=%f w=%f",
                        irq_status.accelNoSlowMotion, irq_status.accelAnyMotion, irq_status.accelHighG,
                        irq_status.gyroHR, irq_status.gyroAnyMotion, bno055_error, bno055_status, temperature,
                        quaternion.x, quaternion.y, quaternion.z, quaternion.w);

#ifdef CONFIG_ENABLE_ROS2
                    i2c_md.ros2_data.msg_imu.orientation.x = quaternion.x;
                    i2c_md.ros2_data.msg_imu.orientation.y = quaternion.y;
                    i2c_md.ros2_data.msg_imu.orientation.z = quaternion.z;
                    i2c_md.ros2_data.msg_imu.orientation.w = quaternion.w;

                    i2c_md.ros2_data.msg_imu.angular_velocity.x = vector_angvel.x /* * M_PI / 180.0*/;
                    i2c_md.ros2_data.msg_imu.angular_velocity.y = -vector_angvel.y /* * M_PI / 180.0*/;
                    i2c_md.ros2_data.msg_imu.angular_velocity.z = vector_angvel.z /* * M_PI / 180.0*/;

                    i2c_md.ros2_data.msg_imu.linear_acceleration.x = vector_linaccl.y /* / 100.0*/;
                    i2c_md.ros2_data.msg_imu.linear_acceleration.y = -vector_linaccl.x /* / 100.0*/;
                    i2c_md.ros2_data.msg_imu.linear_acceleration.z = vector_linaccl.z /* / 100.0*/;

                    struct timespec tv = {};
                    clock_gettime(CLOCK_MONOTONIC, &tv);
                    i2c_md.ros2_data.msg_imu.header.stamp.nanosec = tv.tv_nsec;
                    i2c_md.ros2_data.msg_imu.header.stamp.sec = tv.tv_sec;

                    i2c_md.ros2_data.msg_imu_valid = true;

#endif
                }
                else
                {
                    ESP_LOGE(TAG, "i2c_handle_bno055() irq status %d %d %d %d %d error=0x%02x status=0x%02x",
                        irq_status.accelNoSlowMotion, irq_status.accelAnyMotion, irq_status.accelHighG,
                        irq_status.gyroHR, irq_status.gyroAnyMotion, bno055_error, bno055_status);
                    if(bno055_error == BNO055_SYSTEM_ERROR_FUSION_ALGO_CONF_ERROR)
                    {
                        nvs_handle my_handle;
                        ESP_LOGE(TAG, "i2c_handle_bno055() clear sensor calibration");
                        esp_err_t err = nvs_open("bno055", NVS_READWRITE, &my_handle);
                        if(err == ESP_OK)
                        {
                            nvs_set_blob(my_handle, "bno055_offsets", NULL, 0);
                            nvs_close(my_handle);
                        }
                    }
                }
            }
            catch(BNO055BaseException& ex)
            {
                ESP_LOGE(TAG, "i2c_handle_bno055() exception %s", ex.what());
            }
        }
    }
    catch(int err)
    {
        ESP_LOGE(TAG, "i2c_handle_bno055() I2C exception err=0x%02x", err);
    }
}
#endif /* CONFIG_ENABLE_I2C_BNO055 */

#if defined(CONFIG_ENABLE_I2C_VL53L0X) || defined(CONFIG_ENABLE_I2C_VL53L1X)
void i2c_lidar_init()
{
#if 0
    while(1) {
        for(int i = 0; i < MAX_NUM_VL53L0X; i++) {
            uint16_t reg = i2cnode_get_u16(STM32_I2C_ADDR, 0x70 /*I2C_REG_TB_U16_VL53L1X_RSTREG*/);
            reg &= ~(1 << i);
            i2cnode_set_u16(STM32_I2C_ADDR, 0x70 /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, reg);
            vTaskDelay(10 * (1 + i) / portTICK_PERIOD_MS);
            reg |= (1 << i);
            // ESP_LOGW(TAG, "I2CThread() reset vl53l0x %d reg=%04x", i, reg);
            i2cnode_set_u16(STM32_I2C_ADDR, 0x70 /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, reg);
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
#endif
#ifdef CONFIG_ENABLE_I2C_VL53L0X
    for(int i = 0; i < MAX_NUM_VL53L0X; i++)
    {
        uint16_t reg = i2cnode_get_u16(STM32_I2C_ADDR, 0x70 /*I2C_REG_TB_U16_VL53L1X_RSTREG*/);
        reg &= ~(1 << i);
        i2cnode_set_u16(STM32_I2C_ADDR, 0x70 /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, reg);
        vTaskDelay(5 / portTICK_PERIOD_MS);
        reg |= (1 << i);
        // ESP_LOGW(TAG, "I2CThread() reset vl53l0x %d reg=%04x", i, reg);
        i2cnode_set_u16(STM32_I2C_ADDR, 0x70 /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, reg);
        vTaskDelay((10) / portTICK_PERIOD_MS);
        if(i2cnode_check(VL53L0X_I2C_ADDRESS_DEFAULT) == ESP_OK)
        {
            ESP_LOGW(TAG, "i2c_lidar_init() vl53l0x %d ...", i);
            // if(i2c_lock()) {
            i2c_md.vl53l0x[i] = new VL53L0X((i2c_port_t)I2C_BUS_PORT);
            if(i2c_md.vl53l0x[i] != NULL)
            {
                if(i2c_md.vl53l0x[i]->init(I2C_VL53LXY_ADDR + i))
                {
                    ESP_LOGW(TAG, "i2c_lidar_init() vl53l0x %d initialized", i);
                }
                else
                {
                    delete i2c_md.vl53l0x[i];
                    i2c_md.vl53l0x[i] = NULL;
                    ESP_LOGW(TAG, "i2c_lidar_init() vl53l0x %d error", i);
                }
            }
            // i2c_release();
            //}
        }
    }
#endif /* CONFIG_ENABLE_I2C_VL53L0X */

#ifdef CONFIG_ENABLE_I2C_VL53L1X
    for(int i = 0; i < MAX_NUM_VL53L1X; i++)
    {
        uint16_t reg = i2cnode_get_u16(STM32_I2C_ADDR, 0x70 /*I2C_REG_TB_U16_VL53L1X_RSTREG*/);
        reg &= ~(1 << i);
        i2cnode_set_u16(STM32_I2C_ADDR, 0x70 /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, reg);
        vTaskDelay(2 / portTICK_PERIOD_MS);
        reg |= (1 << i);
        // ESP_LOGW(TAG, "I2CThread() reset vl53l1x %d reg=%04x", i, reg);
        i2cnode_set_u16(STM32_I2C_ADDR, 0x70 /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, reg);
        vTaskDelay((10) / portTICK_PERIOD_MS);
        if(i2cnode_check(VL53L0X_I2C_ADDRESS_DEFAULT) == ESP_OK)
        {
            ESP_LOGW(TAG, "i2c_lidar_init() vl53l0x %d ...", i);
            if(i2c_lock())
            {
                i2c_md.vl53l1x[i] = new VL53L1X(I2C_BUS_PORT);
                if(i2c_md.vl53l1x[i] != NULL)
                {
                    if(i2c_md.vl53l1x[i]->init())
                    {
                        // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
                        // You can change these settings to adjust the performance of the sensor, but
                        // the minimum timing budget is 20 ms for short distance mode and 33 ms for
                        // medium and long distance modes. See the VL53L1X datasheet for more
                        // information on range and timing limits.
                        // Start continuous readings at a rate of one measurement every 50 ms (the
                        // inter-measurement period). This period should be at least as long as the
                        // timing budget.
                        i2c_md.vl53l1x[i]->setDistanceMode(VL53L1X::Medium);
                        i2c_md.vl53l1x[i]->setMeasurementTimingBudget(30000);
                        i2c_md.vl53l1x[i]->startContinuous(30);
                        ESP_LOGW(TAG, "i2c_lidar_init() vl53l1x %d initialized", i);
                    }
                    else
                    {
                        delete i2c_md.vl53l1x[i];
                        i2c_md.vl53l1x[i] = NULL;
                        ESP_LOGW(TAG, "i2c_lidar_init() vl53l1x %d error", i);
                    }
                }
                i2c_release();
            }
        }
    }
#endif /* CONFIG_ENABLE_I2C_VL53L1X */
}

void i2c_setpin_boot(int level);

void i2c_lidar_handle()
{
    try
    {
#ifdef CONFIG_ENABLE_I2C_VL53L0X
        for(int i = 0; i < MAX_NUM_VL53L0X; i++)
        {
            if(i2c_md.vl53l0x[i] != NULL)
            {
                i2c_md.ros2_data.msg_range[i].range = 0;
                uint16_t vl53l0x_data = 0;
                if(i2c_md.vl53l0x[i]->readData(&vl53l0x_data) == true)
                {
                    struct timespec tv = {};
                    clock_gettime(CLOCK_MONOTONIC, &tv);
                    i2c_md.ros2_data.msg_range[i].header.stamp.nanosec = tv.tv_nsec;
                    i2c_md.ros2_data.msg_range[i].header.stamp.sec = tv.tv_sec;
                    i2c_md.ros2_data.msg_range[i].range = 0.001 * vl53l0x_data;
                    i2c_md.ros2_data.msg_range[i].min_range = 0.01;
                    i2c_md.ros2_data.msg_range[i].max_range = 2.00;
                    i2c_md.ros2_data.msg_range[i].field_of_view = 30.0 * 2 * M_PI / 360;
                    i2c_md.ros2_data.msg_range[i].radiation_type = sensor_msgs__msg__Range__INFRARED;
                    i2c_md.ros2_data.msg_range_valid[i] = true;                                        
                    // ESP_LOGI(TAG, "i2c_lidar_handle %d range_mm=%f", i, i2c_md.ros2_data.msg_range[i].range);
                    i2c_md.vl53l0x[i]->start();
                }
            }
        }
#endif /* CONFIG_ENABLE_I2C_VL53L0X */

#ifdef CONFIG_ENABLE_I2C_VL53L1X
        for(int i = 0; i < MAX_NUM_VL53L1X; i++)
        {
            if(i2c_md.vl53l1x[i] != NULL)
            {
                if(i2c_lock())
                {
                    if(i2c_md.vl53l1x[i]->dataReady())
                    {
                        i2c_md.vl53l1x[i]->read();
                        if(i2c_md.vl53l1x[i]->ranging_data.range_status == VL53L1X::RangeValid)
                        {
                            ESP_LOGD(TAG, "i2c_lidar_handle %d range_mm=%d (%f,%f)", i,
                                i2c_md.vl53l1x[i]->ranging_data.range_mm,
                                i2c_md.vl53l1x[i]->ranging_data.peak_signal_count_rate_MCPS,
                                i2c_md.vl53l1x[i]->ranging_data.ambient_count_rate_MCPS);
                            u16RangeMilliMeter[i] = i2c_md.vl53l1x[i]->ranging_data.range_mm;
                        }
                    }
                }
            }
        }
#endif
    }
    catch(int err)
    {
        ESP_LOGE(TAG, "i2c_lidar_handle() I2C exception err=0x%02x", err);
    }
}
#endif

#define FRAME_ID_CAPACITY 50

/**
 * @brief
 * @param param
 */
static void i2c_task(void* param)
{
    esp_pm_lock_handle_t pmlock;
    esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "i2clock", &pmlock);
    esp_pm_lock_acquire(pmlock);

#ifdef CONFIG_ENABLE_ROS2
    std_msgs__msg__Float32__init(&i2c_md.ros2_data.msg_ubat);
    i2c_md.ros2_data.msg_ubat_valid = false;

    // init odom tf message ...
#ifdef I2CROS2SENSORDATA_USE_NAV_MSG_ODOMETRY    
    nav_msgs__msg__Odometry__init(&i2c_md.ros2_data.msg_odom_tf);
    i2c_md.ros2_data.msg_odom_tf_valid = false;
    
    i2c_md.ros2_data.msg_odom_tf.header.frame_id.data = (char*)malloc(FRAME_ID_CAPACITY * sizeof(char));
    //sprintf(i2c_md.ros2_data.msg_odom_tf.header.frame_id.data, "/odom_link");
    sprintf(i2c_md.ros2_data.msg_odom_tf.header.frame_id.data, "map");
    i2c_md.ros2_data.msg_odom_tf.header.frame_id.size = strlen(i2c_md.ros2_data.msg_odom_tf.header.frame_id.data);
    i2c_md.ros2_data.msg_odom_tf.header.frame_id.capacity = FRAME_ID_CAPACITY;

    i2c_md.ros2_data.msg_odom_tf.child_frame_id.data = (char*)malloc(FRAME_ID_CAPACITY * sizeof(char));
    //sprintf(i2c_md.ros2_data.msg_odom_tf.child_frame_id.data, "/base_link");
    sprintf(i2c_md.ros2_data.msg_odom_tf.child_frame_id.data, "base_footprint");
    i2c_md.ros2_data.msg_odom_tf.child_frame_id.size = strlen(i2c_md.ros2_data.msg_odom_tf.child_frame_id.data);
    i2c_md.ros2_data.msg_odom_tf.child_frame_id.capacity = FRAME_ID_CAPACITY;
#endif
#ifdef I2CROS2SENSORDATA_USE_GEOMETRY_MSG_POSE_2D
    geometry_msgs__msg__Pose2D__init(&i2c_md.ros2_data.msg_pose_2d);
    i2c_md.ros2_data.msg_pose_2d_valid = false;
    i2c_md.ros2_data.msg_pose_2d.x = 0;
    i2c_md.ros2_data.msg_pose_2d.y = 0;
    i2c_md.ros2_data.msg_pose_2d.theta = 0;
#endif


    // init imu message ...
    sensor_msgs__msg__Imu__init(&i2c_md.ros2_data.msg_imu);
    i2c_md.ros2_data.msg_imu_valid = false;
    
    i2c_md.ros2_data.msg_imu.header.frame_id.data = (char*)malloc(FRAME_ID_CAPACITY * sizeof(char));
    sprintf(i2c_md.ros2_data.msg_imu.header.frame_id.data, "imu_link");
    i2c_md.ros2_data.msg_imu.header.frame_id.size = strlen(i2c_md.ros2_data.msg_imu.header.frame_id.data);
    i2c_md.ros2_data.msg_imu.header.frame_id.capacity = FRAME_ID_CAPACITY;
        
    // init range messages ...
    for(int i = 0; i < I2CROS2SENSORDATA_NUM_RANGE; i++)
    {
        sensor_msgs__msg__Range__init(&i2c_md.ros2_data.msg_range[i]);
        i2c_md.ros2_data.msg_range_valid[i] = false;
        
        i2c_md.ros2_data.msg_range[i].header.frame_id.data = (char*)malloc(FRAME_ID_CAPACITY * sizeof(char));
        sprintf(i2c_md.ros2_data.msg_range[i].header.frame_id.data, "range_%d_link",i);
        i2c_md.ros2_data.msg_range[i].header.frame_id.size = strlen(i2c_md.ros2_data.msg_range[i].header.frame_id.data);
        i2c_md.ros2_data.msg_range[i].header.frame_id.capacity = FRAME_ID_CAPACITY;
    }
#endif

#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
    try
    {
        i2cnode_set_u16(STM32_I2C_ADDR, 0x60 /*I2C_REG_TB_U16_TON_TOUT*/, 10); // shutdown in 10 seconds
        i2cnode_set_u16(STM32_I2C_ADDR, 0x68 /*I2C_REG_TB_U16_TOFF_PERIOD*/, 60);
        
        i2cnode_set_u16(STM32_I2C_ADDR, 0x70 /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, 0x0000);
        //i2cnode_set_u16(STM32_I2C_ADDR, 0x64 /*I2C_REG_TB_U16_TON_WDG*/, 10); // set STM32 watchdog timeout to 10 seconds ...
        vTaskDelay(50 / portTICK_PERIOD_MS);
#ifdef CONFIG_ENABLE_I2C_BNO055
        // release BNO055
        {
            uint16_t reg = i2cnode_get_u16(STM32_I2C_ADDR, 0x70 /*I2C_REG_TB_U16_VL53L1X_RSTREG*/);
            reg |= (1 << 15);
            ESP_LOGW(TAG, "i2c_task() rst-reg=0x%04x", reg);
            i2cnode_set_u16(STM32_I2C_ADDR, 0x70 /*I2C_REG_TB_U16_VL53L1X_RSTREG*/, reg);
            vTaskDelay(200 / portTICK_PERIOD_MS);
            {
                int retry = 20;
                while(i2cnode_check(I2C_BNO055_ADDR) != ESP_OK && retry--)
                {
                    ESP_LOGW(TAG, "i2c_task() waiting for bno055 ...");
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                }
            }
            vTaskDelay(50 / portTICK_PERIOD_MS);
            i2c_init_bno055();
        }
#endif
#if defined(CONFIG_ENABLE_I2C_VL53L0X) || defined(CONFIG_ENABLE_I2C_VL53L1X)
        i2c_lidar_init();
#endif
    }
    catch(int err)
    {
        ESP_LOGE(TAG, "i2c_task() I2C exception err=0x%08x", err);
        esp_pm_lock_release(pmlock);
        while(1)
        {
            ESP_LOGE(TAG, "i2c_task() Can't init ROS2Zumo, waiting for restart ...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
#endif // CONFIG_ROS2NODE_HW_ROS2ZUMO

#ifdef CONFIG_ENABLE_I2C_OLED
    SSD1306_Init_I2C(
        &I2CDisplay, 128, 64, OLED_I2C_ADDR >> 1, -1, I2CDefaultWriteCommand, I2CDefaultWriteData, I2CDefaultReset);
    SSD1306_DisplayOn(&I2CDisplay);
    SSD1306_SetContrast(&I2CDisplay, 255);
    SSD1306_Clear(&I2CDisplay, SSD_COLOR_BLACK);
    SSD1306_SetFont(&I2CDisplay, &Font_droid_sans_mono_16x31);
    SSD1306_FontDrawString(&I2CDisplay, (128 - 8 * 16) / 2, (64 - 31) / 2, "Init ...", SSD_COLOR_WHITE);
    SSD1306_Update(&I2CDisplay);
    int64_t next_oled_update = esp_timer_get_time() + 1000000UL;
#endif // CONFIG_ENABLE_I2C_OLED

    // vTaskDelay(3000 / portTICK_PERIOD_MS);

    i2cnode_init_motor();

    esp_pm_lock_release(pmlock);

    i2c_md.ready = true;
    while(1)
    {
        /*
         * WHILE ...
         */
        bool keepon = false;

        //i2c_setpin_boot(1);
        esp_pm_lock_acquire(pmlock);
        //i2c_setpin_boot(0);

#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
#ifdef CONFIG_ENABLE_ROS2
        if( ros2node_connected() != 0 )
        {
            ESP_LOGE(TAG, "i2c_task() keep on");
            //i2cnode_set_u16(STM32_I2C_ADDR, 0x60, 30); // shutdown in 10 seconds
        }
#endif
#endif // CONFIG_ROS2NODE_HW_ROS2ZUMO

#ifdef CONFIG_ENABLE_I2C_BNO055
        i2c_handle_bno055();
#endif // CONFIG_ENABLE_I2C_BNO055

#if defined(CONFIG_ENABLE_I2C_VL53L0X) || defined(CONFIG_ENABLE_I2C_VL53L1X)
        i2c_lidar_handle();
#endif

        try
        {

#ifdef CONFIG_ENABLE_I2C_POWER
#ifdef CONFIG_ROS2NODE_HW_ROS2MOWER
            i2c_md.ubat_mV = i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x28);
            i2c_md.isolar_mA = i2cnode_get_i16(PWRNODE_I2C_ADDR, 0x30);
            i2c_md.iout_mA = i2cnode_get_i16(PWRNODE_I2C_ADDR, 0x32);
            i2c_md.icharge_mA = i2cnode_get_i16(PWRNODE_I2C_ADDR, 0x34);
#endif // CONFIG_ROS2NODE_HW_ROS2MOWER
#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
            i2c_md.ubat_mV = i2cnode_get_u16(STM32_I2C_ADDR, 0x5a);
#endif // CONFIG_ROS2NODE_HW_ROS2ZUMO
            ubat = (double)i2c_md.ubat_mV / 1000.0;
#endif

#ifdef CONFIG_ENABLE_ROS2
            i2c_md.ros2_data.msg_ubat.data = ubat;
            i2c_md.ros2_data.msg_ubat_valid = true;
#endif

#ifdef CONFIG_ENABLE_I2C_MOTOR
#ifdef CONFIG_ROS2NODE_HW_ROS2MOWER
            uint64_t t_mot = i2cnode_get_u64(MOTORNODE_I2C_ADDR, 0x00);
            i2cnode_set_u16(MOTORNODE_I2C_ADDR, 0x0A, 0xffff); // TWI_REG_U16_AUTOBREAK

            int64_t motor_l_enc = i2cnode_get_i64(MOTORNODE_I2C_ADDR, 0x30);
            int64_t motor_r_enc = i2cnode_get_i64(MOTORNODE_I2C_ADDR, 0x50);

            int16_t motor_l_rel_enc = i2cnode_get_i64(MOTORNODE_I2C_ADDR, 0x38);
            int16_t motor_r_rel_enc = i2cnode_get_i64(MOTORNODE_I2C_ADDR, 0x58);
#endif

#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
            int16_t motor_l_rel_enc = 0;
            int16_t motor_r_rel_enc = 0;
            {
                uint8_t tmpbuf[24] = {};
                if(ESP_OK == i2cnode_read(ZUMO_I2C_ADDR, tmpbuf, sizeof(tmpbuf)))
                {
                    int i = 6;
                    motor_l_rel_enc |= (tmpbuf[i++] << 8);
                    motor_l_rel_enc |= (tmpbuf[i++] << 0);
                    motor_r_rel_enc |= (tmpbuf[i++] << 8);
                    motor_r_rel_enc |= (tmpbuf[i++] << 0);
                }
            }
#endif // CONFIG_ROS2NODE_HW_ROS2ZUMO
#endif

#ifdef CONFIG_ENABLE_I2C_OLED
            if(next_oled_update < esp_timer_get_time())
            {
                char tmpstr[64];
                int y = 0;
                int s = 5;
                SSD1306_Clear(&I2CDisplay, SSD_COLOR_BLACK);

#ifdef CONFIG_ENABLE_I2C_POWER
#ifdef CONFIG_ROS2NODE_HW_ROS2MOWER
                SSD1306_SetFont(&I2CDisplay, &Font_droid_sans_mono_7x13);
                uint32_t pm_rtc = i2cnode_get_u32(PWRNODE_I2C_ADDR, 0x04);
                sprintf(tmpstr, "P:%d T:%03d:%02d:%02d:%02d", ((esp_ota_get_running_partition()->address) >> 20) & 0x7,
                    (pm_rtc / (60 * 60 * 24)) % 1000, (pm_rtc / (60 * 60)) % 24, (pm_rtc / 60) % 60, (pm_rtc) % 60);
                SSD1306_FontDrawString(&I2CDisplay, 0, y, tmpstr, SSD_COLOR_WHITE);
                y += (s + 7);
#endif // CONFIG_ROS2NODE_HW_ROS2MOWER
#endif

                SSD1306_SetFont(&I2CDisplay, &Font_droid_sans_mono_7x13);
                sprintf(tmpstr, "IP:%d.%d.%d.%d", (s_ip_addr.addr >> 0) & 0xff, (s_ip_addr.addr >> 8) & 0xff,
                    (s_ip_addr.addr >> 16) & 0xff, (s_ip_addr.addr >> 24) & 0xff);
                SSD1306_FontDrawString(&I2CDisplay, 0, y, tmpstr, SSD_COLOR_WHITE);
                y += (s + 7);

                SSD1306_SetFont(&I2CDisplay, &Font_droid_sans_mono_13x24);
                sprintf(tmpstr, "Ub:%2.2fV", i2c_md.ubat_mV / 1000.0);
                SSD1306_FontDrawString(&I2CDisplay, 0, y, tmpstr, SSD_COLOR_WHITE);
                y += (s + 13);

                SSD1306_SetFont(&I2CDisplay, &Font_droid_sans_fallback_15x17);
                sprintf(tmpstr, "I: %3d %3d %4d", i2c_md.isolar_mA, i2c_md.iout_mA, i2c_md.icharge_mA);
                SSD1306_FontDrawString(&I2CDisplay, 0, y, tmpstr, SSD_COLOR_WHITE);
                y += (s + 13);

                SSD1306_Update(&I2CDisplay);
                next_oled_update = esp_timer_get_time() + 1000000UL;
            }
#endif /* CONFIG_ENABLE_I2C_OLED */

#ifdef CONFIG_ENABLE_ROS2
            i2c_handle_encoder(motor_l_rel_enc, motor_r_rel_enc);
#endif

            // i2c_set_cmd_vel( 0.0, 0.0, -2*M_PI / 10.0 /* rad/sec*/ );
            i2c_handle_cmd_vel();

#ifdef CONFIG_ENABLE_I2C_POWER
#ifdef CONFIG_ROS2NODE_HW_ROS2MOWER
            if(keepon == true || console_connected() || i2c_cmd_vel_active())
            {
                i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x10, 600); // update TWI_MEM_SHDWNCNT while ROS is connected
            }
            else if(i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x10) > 30)
            {
                i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x10, 30); // update TWI_MEM_SHDWNCNT while ROS is connected
            }
            else
            {
                ESP_LOGW(TAG, "I2C shutdown in  %d seconds: keepon=%d con=%d cmd_vel=%d",
                    i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x10), (keepon) ? 1 : 0, console_connected(),
                    i2c_cmd_vel_active());
            }
#endif
#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO
            if(keepon == true || console_connected() || i2c_cmd_vel_active())
            {
                i2cnode_set_u16(STM32_I2C_ADDR, 0x60 /*I2C_REG_TB_U16_TON_TOUT*/, 10); // shutdown in 10 seconds
            }
#endif // CONFIG_ROS2NODE_HW_ROS2ZUMO
#endif
            (void)keepon;

#ifdef CONFIG_ENABLE_I2C_MOTOR
#ifdef CONFIG_ROS2NODE_HW_ROS2MOWER
            i2cnode_set_u8(MOTORNODE_I2C_ADDR, 0x0f, 2); // Motor Driver Watchdog Reset
#endif
#endif
        }
        catch(int err)
        {
            ESP_LOGE(TAG, "I2C exception err=0x%02x", err);
        }

        /*
         * SLEEP ...
         */
        esp_pm_lock_release(pmlock);

        if(i2c_cmd_vel_active())
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        else
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        /*
         * ... WHILE
         */
    }
    vTaskDelete(NULL);
}

void i2c_int(int level)
{
    i2c_md.ready = false;
    
#ifdef I2C_BUS_INT
    if(level == 1)
    {
        gpio_set_direction((gpio_num_t)I2C_BUS_INT, GPIO_MODE_INPUT);
        gpio_set_pull_mode((gpio_num_t)I2C_BUS_INT, GPIO_PULLUP_ONLY);
        gpio_set_level((gpio_num_t)I2C_BUS_INT, 1);
    }
    else
    {
        gpio_set_direction((gpio_num_t)I2C_BUS_INT, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode((gpio_num_t)I2C_BUS_INT, GPIO_PULLUP_ONLY);
        gpio_set_level((gpio_num_t)I2C_BUS_INT, 0);
    }
#endif
}

void i2c_setpin_boot(int level)
{
#ifdef I2C_BUS_INT
    if(level == 1)
    {
        gpio_set_direction((gpio_num_t)0, GPIO_MODE_INPUT);
        gpio_set_pull_mode((gpio_num_t)0, GPIO_PULLUP_ONLY);
        gpio_set_level((gpio_num_t)0, 1);
    }
    else
    {
        gpio_set_direction((gpio_num_t)0, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode((gpio_num_t)0, GPIO_PULLUP_ONLY);
        gpio_set_level((gpio_num_t)0, 0);
    }
#endif
}

/**
 * @brief
 */
void i2c_handler_init()
{
    memset(&i2c_md, 0, sizeof(i2c_md));
    // esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "main() i2c init ...");

    i2c_md.sem = xSemaphoreCreateBinary();
    xSemaphoreGive(i2c_md.sem);

#ifdef I2C_BUS_INT
    gpio_reset_pin((gpio_num_t)I2C_BUS_INT);
#endif
    i2c_int(1);

    static i2c_config_t Config;
    memset(&Config, 0, sizeof(i2c_config_t));
    Config.mode = I2C_MODE_MASTER;
    Config.sda_io_num = (gpio_num_t)I2C_BUS_SDA;
    // Config.sda_io_num = (gpio_num_t)13;
    Config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    Config.scl_io_num = (gpio_num_t)I2C_BUS_SCL;
    // Config.scl_io_num = (gpio_num_t)12;
    Config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    Config.master.clk_speed = I2C_BUS_CLOCK;
    i2c_param_config((i2c_port_t)I2C_BUS_PORT, &Config);
    i2c_driver_install((i2c_port_t)I2C_BUS_PORT, Config.mode, 0, 0, 0);
    i2c_set_timeout((i2c_port_t)I2C_BUS_PORT, I2C_APB_CLK_FREQ / 100); /* 10ms timeout */
    
    //i2c_set_start_timing((i2c_port_t)I2C_BUS_PORT,I2C_APB_CLK_FREQ/1000,I2C_APB_CLK_FREQ/1000);
    //i2c_set_stop_timing((i2c_port_t)I2C_BUS_PORT,I2C_APB_CLK_FREQ/1000,I2C_APB_CLK_FREQ/1000);
        
    xTaskCreate(&i2c_task, "i2c_task", 8192, NULL, 5, NULL);
}

/**
 * EOF
 */
