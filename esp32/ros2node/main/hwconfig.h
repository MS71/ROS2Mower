#ifndef _HWCONFIG_H_
#define _HWCONFIG_H_

/***********************************************************
 */
#ifdef CONFIG_ROS2NODE_HW_ROS2MOWER
#define BUZZER_PIN 12

#define GPS_UART1_TXD 32
#define GPS_UART1_RXD 33

#define SPIHOST_PIN_NUM_MISO (gpio_num_t)23
#define SPIHOST_PIN_NUM_MOSI (gpio_num_t)19
#define SPIHOST_PIN_NUM_CLK  (gpio_num_t)18
#define SPIHOST_PIN_NUM_CS   (gpio_num_t)5

#define SDSPI_PIN_NUM_MISO (gpio_num_t)2
#define SDSPI_PIN_NUM_MOSI (gpio_num_t)15
#define SDSPI_PIN_NUM_CLK (gpio_num_t)14
#define SDSPI_PIN_NUM_CS (gpio_num_t)13

#define I2C_BUS_PORT 0
#define I2C_BUS_SDA 21
#define I2C_BUS_SCL 22
//#define I2C_BUS_SDA 26
//#define I2C_BUS_SCL 27

#ifdef CONFIG_ENABLE_I2C_MOTOR
#define MOTORNODE_I2C_ADDR  0x0a
#endif

#define MOTOR_GEAR_N (18*7*23)
#define MOTOR_RPS(_rps_) (int)((_rps_) * MOTOR_GEAR_N)
#define MOTOR_RPM(_rpm_) (MOTOR_RPS(_rpm_) / 60.0)
#define MOTOR_MAX_RPS (45.0 / 60.0)
#define MOTOR_WHEEL_D 0.175
#define MOTOR_START_RPM_L 0.0
#define MOTOR_START_RPM_R 0.0

#define WHEEL_DIAMETER 0.175
#define WHEEL_DISTANCE 0.35

#ifdef CONFIG_ENABLE_I2C_POWER
#define PWRNODE_I2C_ADDR    0x09
#endif

#define ROS2_NODENAME "ros2mower"
#endif /* ROS2NODE_HW_ROS2MOWER */
/***********************************************************
 */
#ifdef CONFIG_ROS2NODE_HW_ROS2ZUMO

#define I2C_BUS_PORT 0
#define I2C_BUS_SDA 13
#define I2C_BUS_SCL 12
#define I2C_BUS_INT 4

/* The encoders provide a resolution of 12 counts per revolution of the motor shaft when counting
 * both edges of both channels. To compute the counts per revolution of the drive sprockets,
 * multiply the gearboxes’ gear ratio by 12. For example, if 75:1 motors
 * (which have gear ratios more accurately specified as 75.81:1) are used, the encoders provide 75.81 × 12 ≈ 909.7 CPR.
 */

/*
	double gear = 100.0/1.0;
	double steps_per_revolution = 12.0;
	double wheel_diameter = 0.038; // 38mm
	double lengthBetweenTwoWheels = 0.09; // 9cm
*/
#define MOTOR_GEAR_N (12*100)
#define MOTOR_RPS(_rps_) (int)((_rps_) * MOTOR_GEAR_N)
#define MOTOR_RPM(_rpm_) (MOTOR_RPS(_rpm_) / 60.0)
#define MOTOR_MAX_RPS (45.0 / 60.0)
//#define MOTOR_WHEEL_D 0.038
#define MOTOR_START_RPM_L 0.0
#define MOTOR_START_RPM_R 0.0

#define WHEEL_DIAMETER 0.038
#define WHEEL_DISTANCE 0.09

#ifdef CONFIG_ENABLE_I2C_POWER
#define STM32_I2C_ADDR    0x10
#endif

#define ZUMO_I2C_ADDR     0x08

#define CMD_BEEP                0x01
#define CMD_BATLEVEL            0x02
#define CMD_MOTORS_SET_SPEED    0x03
#define CMD_ENCODERS            0x04
#define CMD_LIDAR_SET_PWM       0x05
#define CMD_GET_STATUS          0x06
#define CMD_SET_LED             0x07

#define ROS2_NODENAME "ros2zumo"

#define PWR_KEEP_ALIVE_DELAY    15

#endif /* ROS2NODE_HW_ROS2ZUMO */
/***********************************************************
 */
#ifdef CONFIG_ROS2NODE_HW_S2

#define I2C_BUS_PORT 0
#define I2C_BUS_SDA 13
#define I2C_BUS_SCL 12
#define I2C_BUS_INT 4

#define ROS2_NODENAME "ros2s2"

#endif /* CONFIG_ROS2NODE_HW_S2 */
/***********************************************************
 */
 
#define I2C_VL53LXY_ADDR 0x50

#define I2C_BNO055_ADDR 0x28

#define I2C_TIMEOUT_MS 10
#define I2C_BUS_CLOCK       400000

#ifdef CONFIG_ENABLE_I2C_OLED
#define OLED_I2C_ADDR  0x78
#endif

#endif

/* EOF
 */
