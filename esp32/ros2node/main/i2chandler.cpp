//#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#undef ENABLE_OLED
#undef ENABLE_BNO055

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
#include "ros2esp.h"
#endif

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_pm.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "ota_server.h"

#include "i2chandler.h"

#include "ssd1306.h"
#include "ssd1306_default_if.h"
#include "ssd1306_draw.h"
#include "ssd1306_font.h"

#include "console.h"

#ifdef ENABLE_BNO055
#include "BNO055ESP32.h"
#endif

static const char* TAG = "I2C";

#define MOTOR_GEAR_N (18*7*23)
#define MOTOR_RPS(_rps_) (int)((_rps_) * MOTOR_GEAR_N)
#define MOTOR_RPM(_rpm_) (MOTOR_RPS(_rpm_) / 60.0)
#define MOTOR_MAX_RPS (45.0 / 60.0)
#define MOTOR_WHEEL_D 0.175
#define MOTOR_START_RPM_L 0.0
#define MOTOR_START_RPM_R 0.0

#define WHEEL_DIAMETER 0.175
#define WHEEL_DISTANCE 0.35

#define MOTOR_P (0.30 * 128)
#define MOTOR_I (0.01 * 128)
#define MOTOR_D (0.01 * 128)

static struct {
    struct {
        double linear_x;
        double linear_y;
        double angular_z;
        bool update;
    } cmd_vel;

    int64_t last_cmd_vel_time;

    SemaphoreHandle_t sem;
    
} i2c_md = {};

#ifdef ENABLE_BNO055
BNO055* bno055 = NULL;
bno055_calibration_t bno055_calib;
bool bno055_configured = false;
#endif

extern ip4_addr_t s_ip_addr;
struct SSD1306_Device I2CDisplay;

double ubat = 0.0;

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

ros2::Publisher<nav_msgs::Odometry>* pub_odom = NULL;
ros2::Publisher<sensor_msgs::Imu>* pub_imu = NULL;
#endif

/**
 * @brief 
 * @return 
 */
bool i2c_lock()
{
    if( xSemaphoreTake(i2c_md.sem, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) )
    {
        return true;
    }
    return false;
}

/**
 * @brief 
 */
void i2c_release()
{
    xSemaphoreGive(i2c_md.sem); 
}

/**
 * @brief 
 * @param i2caddr
 * @return 
 */
esp_err_t i2cnode_check(uint8_t i2caddr)
{
    esp_err_t err = ESP_FAIL;
    if( i2c_lock() )
    {
        i2c_cmd_handle_t CommandHandle = NULL;
        if((CommandHandle = i2c_cmd_link_create()) != NULL) {
            i2c_master_start(CommandHandle);
            i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_stop(CommandHandle);
            err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, 
                CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            i2c_cmd_link_delete(CommandHandle);
        }
        i2c_release();    
    }
    return err;
}

/**
 * @brief 
 * @param i2caddr
 * @param regaddr
 * @param buf
 * @param buflen
 * @return 
 */
esp_err_t i2cnode_read(uint8_t i2caddr, uint8_t regaddr, uint8_t *buf, uint16_t buflen) 
{
    esp_err_t err = ESP_OK;
    if( i2c_lock() )
    {
        i2c_cmd_handle_t CommandHandle = NULL;
        if((CommandHandle = i2c_cmd_link_create()) != NULL) {
            i2c_master_start(CommandHandle);
            i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
            i2c_master_write_byte(CommandHandle, regaddr, true);
            i2c_master_start(CommandHandle);
            i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_READ, true);
            i2c_master_read(CommandHandle, buf, buflen, (i2c_ack_type_t)0x02);
            i2c_master_stop(CommandHandle);
            err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, 
                CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            if(err != ESP_OK) {
                ESP_LOGE(TAG, "i2cnode_get i2caddr=0x%02x regaddr=0x%02x err=0x%02x", 
                    i2caddr, 
                    regaddr,
                    err);
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
 * @param regaddr
 * @param buf
 * @param buflen
 * @return 
 */
esp_err_t i2cnode_write(uint8_t i2caddr, uint8_t regaddr, uint8_t *buf, uint16_t buflen) 
{
    esp_err_t err = ESP_OK;
    i2c_cmd_handle_t CommandHandle = NULL;
    if((CommandHandle = i2c_cmd_link_create()) != NULL) {
        i2c_master_start(CommandHandle);
        i2c_master_write_byte(CommandHandle, (i2caddr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(CommandHandle, regaddr, true);
        i2c_master_write(CommandHandle, buf, buflen, (i2c_ack_type_t)0x02);
        i2c_master_stop(CommandHandle);
        esp_err_t err = i2c_master_cmd_begin((i2c_port_t)I2C_BUS_PORT, CommandHandle, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
        if(err != ESP_OK) {
            ESP_LOGE(TAG, "i2cnode_set i2caddr=0x%02x regaddr=0x%02x err=0x%02x", 
                i2caddr, 
                regaddr,
                err);
        }
        i2c_cmd_link_delete(CommandHandle);
    }
    return err;
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @return
 */
uint64_t i2cnode_get_u64(uint8_t i2caddr, uint8_t regaddr) throw(int)
{
    esp_err_t err;
    int16_t v = 0;
    uint8_t tmpbuf[8] = {};
    if( (err = i2cnode_read(i2caddr,regaddr,tmpbuf,sizeof(tmpbuf))) != ESP_OK )
    {
        throw err;
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
uint32_t i2cnode_get_u32(uint8_t i2caddr, uint8_t regaddr) throw(int)
{
    esp_err_t err;
    int16_t v = 0;
    uint8_t tmpbuf[8] = {};
    if( (err = i2cnode_read(i2caddr,regaddr,tmpbuf,sizeof(tmpbuf))) != ESP_OK )
    {
        throw err;
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
uint16_t i2cnode_get_u16(uint8_t i2caddr, uint8_t regaddr) throw(int)
{
    esp_err_t err;
    uint16_t v = 0;
    uint8_t tmpbuf[2] = {};
    if( (err = i2cnode_read(i2caddr,regaddr,tmpbuf,sizeof(tmpbuf))) != ESP_OK )
    {
        throw err;
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
uint8_t i2cnode_get_u8(uint8_t i2caddr, uint8_t regaddr) throw(int)
{
    esp_err_t err;
    uint16_t v = 0;
    uint8_t tmpbuf[1] = {};
    if( (err = i2cnode_read(i2caddr,regaddr,tmpbuf,sizeof(tmpbuf))) != ESP_OK )
    {
        throw err;
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
int64_t i2cnode_get_i64(uint8_t i2caddr, uint8_t regaddr) throw(int)
{
    esp_err_t err;
    int16_t v = 0;
    uint8_t tmpbuf[8] = {};
    if( (err = i2cnode_read(i2caddr,regaddr,tmpbuf,sizeof(tmpbuf))) != ESP_OK )
    {
        throw err;
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
int16_t i2cnode_get_i16(uint8_t i2caddr, uint8_t regaddr) throw(int)
{
    esp_err_t err;
    int16_t v = 0;
    uint8_t tmpbuf[2] = {};
    if( (err = i2cnode_read(i2caddr,regaddr,tmpbuf,sizeof(tmpbuf))) != ESP_OK )
    {
        throw err;
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
int8_t i2cnode_get_i8(uint8_t i2caddr, uint8_t regaddr) throw(int)
{
    esp_err_t err;
    int8_t v = 0;
    uint8_t tmpbuf[2] = {};
    if( (err = i2cnode_read(i2caddr,regaddr,tmpbuf,sizeof(tmpbuf))) != ESP_OK )
    {
        throw err;
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
void i2cnode_set_u16(uint8_t i2caddr, uint8_t regaddr, uint16_t v) throw(int)
{
    esp_err_t err;
    uint8_t tmpbuf[2] = {};
    tmpbuf[0] = (v>>0)&0xff;
    tmpbuf[1] = (v>>8)&0xff;
    if( (err = i2cnode_write(i2caddr,regaddr,tmpbuf,sizeof(tmpbuf))) != ESP_OK )
    {
        throw err;
    }
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @param v
 */
void i2cnode_set_u8(uint8_t i2caddr, uint8_t regaddr, uint8_t v) throw(int)
{
    esp_err_t err;
    uint8_t tmpbuf[1] = {};
    tmpbuf[0] = (v>>0)&0xff;
    if( (err = i2cnode_write(i2caddr,regaddr,tmpbuf,sizeof(tmpbuf))) != ESP_OK )
    {
        throw err;
    }
}

/**
 * @brief
 * @param i2caddr
 * @param addr
 * @param v
 */
void i2cnode_set_i16(uint8_t i2caddr, uint8_t regaddr, int16_t v) throw(int)
{
    esp_err_t err;
    uint8_t tmpbuf[2] = {};
    tmpbuf[0] = (v>>0)&0xff;
    tmpbuf[1] = (v>>8)&0xff;
    if( (err = i2cnode_write(i2caddr,regaddr,tmpbuf,sizeof(tmpbuf))) != ESP_OK )
    {
        throw err;
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

    if( (i2cnode_write(Address,ModeByte,(uint8_t*)Data,DataLength)) != ESP_OK )
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

#ifdef CONFIG_ENABLE_ROS2
int64_t odo_last_time = 0;
void vHandleEncoderSteps(ros2::Node* ros2node, int16_t enc_l, int16_t enc_r)
{
    if(enc_l != 0 || enc_r != 0) 
    {
        int64_t current_time = esp_timer_get_time();
        double dt = 0.000001 * (current_time - odo_last_time); /* s */
        odo_last_time = current_time;

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
        double v_left = (enc_l * DistancePerCount) / dt; /* m/s */
        double v_right = (enc_r * DistancePerCount) / dt; /* m/s */

        double vx = ((v_right + v_left) / 2); /* m/s */
        double vy = 0; /* m/s */
        double vth = ((v_right - v_left) /* m/s */ / WHEEL_DISTANCE /* m */ );

        double delta_x = (vx * cos(th)) * dt;
        double delta_y = (vx * sin(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        // motor_speed_l = v_left;
        // motor_speed_r = v_right;
        // cmd_vel_motor_update();

        ESP_LOGW(TAG, "vHandleEncoderSteps(%d,%d) d=%f dT=%f v=(%f,%f) x=%f y=%f th=%f", 
        enc_l, enc_r, 
        DistancePerCount, dt, 
        v_left, v_right, 
        x, y, th);

        auto odom_msg = nav_msgs::Odometry();

        geometry_msgs::Quaternion odom_quat; // = tf::createQuaternionFromYaw(th);
        odom_quat.z = th;
        //odom_quat.setRQY(0,0,0);
        
        // Odometry message
        odom_msg.header.stamp = ros2::now();
        //odom_msg.header.seq = odom_msg.header.seq + 1;
        strcpy(odom_msg.header.frame_id,"odom");

        // set the position
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;

        // set the velocity
        strcpy(odom_msg.child_frame_id,"base_link");
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.angular.z = vth;

        double cv_pose = 0.01;
        double cv_twist = 0.01;
        odom_msg.pose.covariance[0] = cv_pose;
        odom_msg.pose.covariance[7] = cv_pose;
        odom_msg.pose.covariance[8] = cv_pose;
        odom_msg.pose.covariance[14] = cv_pose;
        odom_msg.pose.covariance[21] = cv_pose;
        odom_msg.pose.covariance[28] = cv_pose;
        odom_msg.pose.covariance[35] = cv_pose;
        odom_msg.twist.covariance[0] = cv_twist;
        odom_msg.twist.covariance[7] = cv_twist;
        odom_msg.twist.covariance[8] = cv_twist;
        odom_msg.twist.covariance[14] = cv_twist;
        odom_msg.twist.covariance[21] = cv_twist;
        odom_msg.twist.covariance[28] = cv_twist;
        odom_msg.twist.covariance[35] = cv_twist;

        // publish the message
        pub_odom->publish(&odom_msg);
    }
}
#endif

/**
 * @brief
 */
void i2c_handle_cmd_vel()
{
    if(i2c_md.cmd_vel.update == true) {
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

        if(xspeed >= 0) {
            // robot is moving forward
            wheel0_speed = (linear_speed + angular_speed) / wheelcircumference;
            wheel1_speed = (linear_speed - angular_speed) / wheelcircumference;
        } else {
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
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x20, MOTOR_RPS(wheel0_speed));
        i2cnode_set_i16(MOTORNODE_I2C_ADDR, 0x40, MOTOR_RPS(wheel1_speed));

        ESP_LOGW(TAG, "i2c_handle_cmd_vel() %frps %frps %d %d", 
            wheel0_speed, wheel1_speed,
            MOTOR_RPS(wheel0_speed), MOTOR_RPS(wheel1_speed));
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
    if( (i2c_md.last_cmd_vel_time!=0) &&
        ((i2c_md.last_cmd_vel_time + 1000000 * 60) > esp_timer_get_time()))   
    {
        return true;
    }

    return (i2c_md.cmd_vel.linear_x != 0.0) || (i2c_md.cmd_vel.linear_y != 0.0) || (i2c_md.cmd_vel.angular_z != 0.0);
}

#ifdef CONFIG_ENABLE_ROS2
/**
 * @brief
 * @param vel_cmd
 * @param arg
 */
void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd, void* arg)
{
    i2c_set_cmd_vel(vel_cmd.linear.x, vel_cmd.linear.y, vel_cmd.angular.z);
}

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
            pub_odom = ros2node->createPublisher<nav_msgs::Odometry>(ROS2_NODENAME "/odom_raw");
            pub_imu = ros2node->createPublisher<sensor_msgs::Imu>(ROS2_NODENAME "/imu");
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
#endif

void i2cnode_init_motor()
{
#ifdef CONFIG_ENABLE_I2C_MOTOR
    try {
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
    } catch(int err) {
        ESP_LOGE(TAG, "I2C exception err=0x%02x", err);
    }
#endif
}

/**
 * @brief
 * @param param
 */
static void i2c_task(void* param)
{
    esp_pm_lock_handle_t pmlock;
#ifdef CONFIG_ENABLE_ROS2
    static ros2::Node ros2node(ROS2_NODENAME);
#endif

    esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "i2clock", &pmlock);
    esp_pm_lock_acquire(pmlock);

#ifdef CONFIG_ENABLE_I2C_OLED
    uint8_t oled_update = 0;
    SSD1306_Init_I2C(&I2CDisplay, 128, 64, OLED_I2C_ADDR >> 1, -1, 
        I2CDefaultWriteCommand, I2CDefaultWriteData, I2CDefaultReset);
    SSD1306_DisplayOn(&I2CDisplay);
    SSD1306_SetContrast(&I2CDisplay,255);
    SSD1306_Clear(&I2CDisplay, SSD_COLOR_BLACK);
    SSD1306_SetFont(&I2CDisplay, &Font_droid_sans_mono_16x31);
    SSD1306_FontDrawString(&I2CDisplay, (128-8*16)/2, (64-31)/2, "Init ...", SSD_COLOR_WHITE);
    SSD1306_Update(&I2CDisplay);
#endif // CONFIG_ENABLE_I2C_OLED
	
#ifdef ENABLE_BNO055
    bno055 = new BNO055((i2c_port_t)I2C_BUS_PORT, 0x28);
    if(bno055 != NULL) {
        try {
            bno055_configured = false;
            ESP_LOGI(TAG, "I2CThread() BNO055 init ...");
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
            // bno055->setOprModeNdof();
            ESP_LOGW(TAG, "I2CThread() BNO055 init done");
        } catch(BNO055BaseException ex) {
            ESP_LOGI(TAG, "I2CThread() BNO055 exception %s", ex.what());
        }
    }
#endif

    i2cnode_init_motor();

while(1)
    vTaskDelay(1000 / portTICK_PERIOD_MS);

#ifdef CONFIG_ENABLE_I2C_OLED
    SSD1306_Clear(&I2CDisplay, SSD_COLOR_BLACK);
    SSD1306_Update(&I2CDisplay);
#endif // CONFIG_ENABLE_I2C_OLED

    esp_pm_lock_release(pmlock);
    while(1) {
        bool keepon = false;

        //esp_pm_lock_acquire(pmlock);
		
#if 1
		int errcnt_0x0a = 0;
		int okcnt_0x0a = 0;
		int errcnt_0x09 = 0;
		int okcnt_0x09 = 0;
		while(1)
		{
			uint8_t buf[8] = {};
#if 1			
			if(i2cnode_read(0x0a,0x00,buf,sizeof(buf))==ESP_OK)
			{
				//ESP_LOGW(TAG, "I2CThread() I2C Ok errcnt=%d", ++errcnt);
				okcnt_0x0a++;
			}
			else
			{
				ESP_LOGE(TAG, "0x0a I2C Error okcnt=%d errcnt=%d", okcnt_0x0a, ++errcnt_0x0a);
			}
#endif
#if 0			
			if(i2cnode_read(0x09,0x00,buf,sizeof(buf))==ESP_OK)
			{
				//ESP_LOGW(TAG, "I2CThread() I2C Ok errcnt=%d", ++errcnt);
				okcnt_0x09++;
			}
			else
			{
				ESP_LOGE(TAG, "0x09 I2C Error okcnt=%d errcnt=%d", okcnt_0x09, ++errcnt_0x09);
			}
#endif			
		}
#endif		

        try {
    
#ifdef CONFIG_ENABLE_I2C_POWER
            uint16_t ubat_mV = i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x28);
            int16_t isolar_mA = i2cnode_get_i16(PWRNODE_I2C_ADDR, 0x30);
            int16_t iout_mA = i2cnode_get_i16(PWRNODE_I2C_ADDR, 0x32);
            int16_t icharge_mA = i2cnode_get_i16(PWRNODE_I2C_ADDR, 0x34);
#endif
#ifdef CONFIG_ENABLE_I2C_MOTOR
            uint64_t t_mot = i2cnode_get_u64(MOTORNODE_I2C_ADDR, 0x00);
            i2cnode_set_u16(MOTORNODE_I2C_ADDR, 0x0A, 0xffff); // TWI_REG_U16_AUTOBREAK

            int64_t motor_l_enc = i2cnode_get_i64(MOTORNODE_I2C_ADDR, 0x30);
            int64_t motor_r_enc = i2cnode_get_i64(MOTORNODE_I2C_ADDR, 0x50);

            int16_t motor_l_rel_enc = i2cnode_get_i64(MOTORNODE_I2C_ADDR, 0x38);
            int16_t motor_r_rel_enc = i2cnode_get_i64(MOTORNODE_I2C_ADDR, 0x58);

            ubat = (double)ubat_mV / 1000.0;
#endif

#ifdef ENABLE_OLED
            char tmpstr[64];
            if(oled_update-- == 0) {
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

                oled_update = 10;
            }
#endif
            
            wifi_ap_record_t wifi_info = {};
            esp_wifi_sta_get_ap_info(&wifi_info);

#if 0
            if( LOG_LOCAL_LEVEL >= ESP_LOG_DEBUG )
            {
                ESP_LOGD(TAG,
                    "rssi=%d Ubat=%2.1fV Isol=%1.1fmA Iout=%1.3fA  Motor: boot=%d t=%5.3f m=%d sp=(%d,%d) dir=(%d,%d) "
                    "enc=(%d,%d)(%d,%d) pv=(%d,%d)",
                    (wifi_info.rssi), (float)i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x28) / 1000.0,
                    (float)i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x30) / 1.0,
                    (float)i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x32) / 1000.0, 
                    i2cnode_get_u16(MOTORNODE_I2C_ADDR, 0x0C), /* bootcnt */
                    (double)t_mot / 1000000.0,
                    i2cnode_get_u16(MOTORNODE_I2C_ADDR, 0x08), /* mode */
                    i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x20), /* sp */
                    i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x40), /* sp */
                    (int)i2cnode_get_i8(MOTORNODE_I2C_ADDR, 0x26), /* dir */
                    (int)i2cnode_get_i8(MOTORNODE_I2C_ADDR, 0x46), /* dir */
                    (int)motor_l_enc, (int)motor_r_enc, 
                    (int)motor_l_rel_enc,(int)motor_r_rel_enc, 
                    (int)i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x22), /* pv */
                    (int)i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x42)); /* pv */
            }
#endif

#if 0
		ESP_LOGD(TAG, "Power Supply Status: bootcnt=%d time=0x%08x pmsw=%d SHDWNCNT=%d PWRUPCNT=%d SHDWNREL=%d PWRUPREL=%d", 
			0,
			i2cnode_get_u32(PWRNODE_I2C_ADDR, 0x04),
			i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x08),
			i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x10),
			i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x12),
			i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x14),
			i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x16));
#endif

#ifdef CONFIG_ENABLE_ROS2
        vHandleEncoderSteps(&ros2node, motor_l_rel_enc, motor_r_rel_enc);

            if(ros2ready(&ros2node)) {

                //keepon = true;

                {
                    auto msg_ubat = std_msgs::Float32();
                    msg_ubat.data = ubat_mV / 1000.0;
                    if(pub_ubat != NULL)
                        pub_ubat->publish(&msg_ubat);
                }
                {
                    auto msg_i = std_msgs::Float32();
                    msg_i.data = isolar_mA / 1000.0;
                    if(pub_isolar != NULL)
                        pub_isolar->publish(&msg_i);
                    msg_i.data = iout_mA / 1000.0;
                    if(pub_iout != NULL)
                        pub_iout->publish(&msg_i);
                    msg_i.data = icharge_mA / 1000.0;
                    if(pub_icharge != NULL)
                        pub_icharge->publish(&msg_i);
                }
                {
                    auto msg = std_msgs::Int64();
                    msg.data = motor_l_enc;
                    if(pub_motor_l_enc != NULL)
                        pub_motor_l_enc->publish(&msg);
                    msg.data = motor_r_enc;
                    if(pub_motor_r_enc != NULL)
                        pub_motor_r_enc->publish(&msg);
                }
                {
                    auto msg = std_msgs::Int16();
                    msg.data = i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x20);
                    if(pub_motor_l_pid_sv != NULL)
                        pub_motor_l_pid_sv->publish(&msg);
                    msg.data = i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x24);
                    if(pub_motor_l_pid_ov != NULL)
                        pub_motor_l_pid_ov->publish(&msg);
                    msg.data = i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x22);
                    if(pub_motor_l_pid_pv != NULL)
                        pub_motor_l_pid_pv->publish(&msg);
                    msg.data = i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x40);
                    if(pub_motor_r_pid_sv != NULL)
                        pub_motor_r_pid_sv->publish(&msg);
                    msg.data = i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x44);
                    if(pub_motor_r_pid_ov != NULL)
                        pub_motor_r_pid_ov->publish(&msg);
                    msg.data = i2cnode_get_i16(MOTORNODE_I2C_ADDR, 0x42);
                    if(pub_motor_r_pid_pv != NULL)
                        pub_motor_r_pid_pv->publish(&msg);
                }
            }
            ros2::spin(&ros2node);
#endif

#ifdef ENABLE_BNO055
            if(bno055 != NULL) {
                try {
                    if(/*nh.connected() && published==true &&*/ bno055_configured == false) {
#if 1
                        ESP_LOGW(TAG, "BNO055 try get BNO055Callibration param ...");

                        // bno055->setOprModeConfig();
                        int p[3 + 3 + 3 + 2] = { 8, 7, 1, 180, -695, 574, -2, -2, -1, 1000, 372 };
                        // nh.getParam("/BNO055Callibration", p, 11, 100);

                        bno055_offsets_t o;
                        o.accelOffsetX = p[0];
                        o.accelOffsetY = p[1];
                        o.accelOffsetZ = p[2];
                        o.magOffsetX = p[3];
                        o.magOffsetY = p[4];
                        o.magOffsetZ = p[5];
                        o.gyroOffsetX = p[6];
                        o.gyroOffsetY = p[7];
                        o.gyroOffsetZ = p[8];
                        o.accelRadius = p[9];
                        o.magRadius = p[10];
                        bno055->setSensorOffsets(o);
                        o = bno055->getSensorOffsets();
                        bno055_calibration_t calib = bno055->getCalibration();
                        ESP_LOGW(TAG, "BNO055 SET calib(%d,%d,%d,%d) %d,%d,%d %d,%d,%d %d,%d,%d %d,%d",
                            bno055_calib.sys, bno055_calib.gyro, bno055_calib.mag, bno055_calib.accel, o.accelOffsetX,
                            o.accelOffsetY, o.accelOffsetZ, o.magOffsetX, o.magOffsetY, o.magOffsetZ, o.gyroOffsetX,
                            o.gyroOffsetY, o.gyroOffsetZ, o.accelRadius, o.magRadius);
#endif
                        bno055->setOprModeNdof();
                        bno055_configured = true;
                    }

                    if(bno055_configured == true) {
                        bno055_calibration_t calib = bno055->getCalibration();

                        if(memcmp(&bno055_calib, &calib, sizeof(bno055_calibration_t)) != 0) {
                            bno055->setOprModeConfig();
                            bno055_offsets_t bno055_offsets = bno055->getSensorOffsets();
                            bno055_calib = calib;
                            bno055->setOprModeNdof();

                            ESP_LOGW(TAG, "BNO055 UPT calib(%d,%d,%d,%d) [%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]",
                                bno055_calib.sys, bno055_calib.gyro, bno055_calib.mag, bno055_calib.accel,
                                bno055_offsets.accelOffsetX, bno055_offsets.accelOffsetY, bno055_offsets.accelOffsetZ,
                                bno055_offsets.magOffsetX, bno055_offsets.magOffsetY, bno055_offsets.magOffsetZ,
                                bno055_offsets.gyroOffsetX, bno055_offsets.gyroOffsetY, bno055_offsets.gyroOffsetZ,
                                bno055_offsets.accelRadius, bno055_offsets.magRadius);
#if 0
                            if(nh.connected() && published == true) {
                                sprintf(imucalib_buf, "BNO055 calib(%d,%d,%d,%d) [%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]",
                                    bno055_calib.sys, bno055_calib.gyro, bno055_calib.mag, bno055_calib.accel,
                                    bno055_offsets.accelOffsetX, bno055_offsets.accelOffsetY,
                                    bno055_offsets.accelOffsetZ, bno055_offsets.magOffsetX, bno055_offsets.magOffsetY,
                                    bno055_offsets.magOffsetZ, bno055_offsets.gyroOffsetX, bno055_offsets.gyroOffsetY,
                                    bno055_offsets.gyroOffsetZ, bno055_offsets.accelRadius, bno055_offsets.magRadius);
                                imucalib_msg.data = imucalib_buf;
                                pub_imucalib.publish(&imucalib_msg);
                            }
#endif
                        }
                    }

                    if(ros2ready(&ros2node) && bno055_configured == true) {
                        static int64_t imu_last_time = 0;
                        int64_t current_time = esp_timer_get_time();
                        double dt = 0.000001 * (current_time - imu_last_time); /* s */

                        if( dt > 0.01 ) {
                            imu_last_time = current_time;

                            bno055_quaternion_t quaternion = bno055->getQuaternion();
                            bno055_vector_t vector_angvel = bno055->getVectorGyroscope();
                            bno055_vector_t vector_linaccl = bno055->getVectorLinearAccel();
                            int8_t temperature = bno055->getTemp();

                            //ESP_LOGW(TAG, "BNO055 %d",temperature);

                            static sensor_msgs::Imu imu_msg;
#if 1
                            strcpy(imu_msg.header.frame_id,"imu_link");
                            imu_msg.header.stamp = ros2::now();
                            //imu_msg.header.seq = imu_msg.header.seq + 1;
                            imu_msg.orientation.x = quaternion.y;
                            imu_msg.orientation.y = -quaternion.x;
                            imu_msg.orientation.z = quaternion.z;
                            imu_msg.orientation.w = quaternion.w;

                            const double cov_orientation = 0.08;
                            const double cov_velocity = 0.02;
                            const double cov_acceleration = 0.04;

                            const double orientation_covariance[9] = { 
                                cov_orientation, 0.0, 0.0, 
                                0.0, cov_orientation, 0.0, 
                                0.0, 0.0, cov_orientation };
                            memcpy(imu_msg.orientation_covariance, orientation_covariance,
                                sizeof(imu_msg.orientation_covariance));

                            imu_msg.angular_velocity.x = vector_angvel.y /* * M_PI / 180.0*/;
                            imu_msg.angular_velocity.y = -vector_angvel.x /* * M_PI / 180.0*/;
                            imu_msg.angular_velocity.z = vector_angvel.z /* * M_PI / 180.0*/;

                            const double linear_acceleration_covariance[9] = { 
                                cov_acceleration, 0.0, 0.0, 
                                0.0, cov_acceleration, 0.0, 
                                0.0, 0.0, cov_acceleration };
                            memcpy(imu_msg.linear_acceleration_covariance, linear_acceleration_covariance,
                                sizeof(imu_msg.linear_acceleration_covariance));

                            imu_msg.linear_acceleration.x = vector_linaccl.y /* / 100.0*/;
                            imu_msg.linear_acceleration.y = -vector_linaccl.x /* / 100.0*/;
                            imu_msg.linear_acceleration.z = vector_linaccl.z /* / 100.0*/;

                            pub_imu->publish(&imu_msg);
#endif                            
                        }
                    }
                } catch(BNO055BaseException ex) {
                    ESP_LOGE(TAG, "I2CThread() BNO055 exception %s", ex.what());
                }
            }
#endif

            // i2c_set_cmd_vel( 0.0, 0.0, -2*M_PI / 10.0 /* rad/sec*/ );
            i2c_handle_cmd_vel();

#ifdef CONFIG_ENABLE_I2C_POWER
            if(keepon == true || console_connected() || i2c_cmd_vel_active()) 
            {
                i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x10, 600); // update TWI_MEM_SHDWNCNT while ROS is connected
            }
            else if ( i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x10) > 30 )
            {
                i2cnode_set_u16(PWRNODE_I2C_ADDR, 0x10, 30); // update TWI_MEM_SHDWNCNT while ROS is connected
            }
            else
            {
                ESP_LOGW(TAG, "I2C shutdown in  %d seconds: keepon=%d con=%d cmd_vel=%d", 
                    i2cnode_get_u16(PWRNODE_I2C_ADDR, 0x10),
                    (keepon)?1:0,
                    console_connected(),
                    i2c_cmd_vel_active());
            }
#endif

#ifdef CONFIG_ENABLE_I2C_MOTOR
            i2cnode_set_u8(MOTORNODE_I2C_ADDR, 0x0f, 2); // Motor Driver Watchdog Reset
#endif
        } catch(int err) {
            ESP_LOGE(TAG, "I2C exception err=0x%02x", err);
        }
        esp_pm_lock_release(pmlock);

        if( i2c_cmd_vel_active() )
        {
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        else
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

/**
 * @brief
 */
void i2c_handler_init()
{
    // esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "main() i2c init ...");

    i2c_md.sem = xSemaphoreCreateBinary( );
    xSemaphoreGive(i2c_md.sem); 

    static i2c_config_t Config;
    memset(&Config, 0, sizeof(i2c_config_t));
    Config.mode = I2C_MODE_MASTER;
    Config.sda_io_num = (gpio_num_t)I2C_BUS_SDA;
    // Config.sda_io_num = (gpio_num_t)13;
    Config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    Config.scl_io_num = (gpio_num_t)I2C_BUS_SCL;
    // Config.scl_io_num = (gpio_num_t)12;
    Config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    Config.master.clk_speed = 400000;
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