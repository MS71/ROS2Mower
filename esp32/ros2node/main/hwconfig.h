#ifndef _HWCONFIG_H_
#define _HWCONFIG_H_

#ifdef CONFIG_ENABLE_I2C

#define I2C_BUS_PORT 0
#define I2C_BUS_SDA 26
#define I2C_BUS_SCL 27
#define I2C_TIMEOUT_MS 10

#ifdef CONFIG_ENABLE_I2C_POWER
#define PWRNODE_I2C_ADDR    0x09
#endif

#ifdef CONFIG_ENABLE_I2C_MOTOR
#define MOTORNODE_I2C_ADDR  0x0a
#endif

#ifdef CONFIG_ENABLE_I2C_OLED
#define OLED_I2C_ADDR  0x78
#endif

#endif // CONFIG_ENABLE_I2C

#endif
