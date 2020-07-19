#ifndef _I2CHANDLER_H_
#define _I2CHANDLER_H_

#include "hwconfig.h"

void i2c_handler_init();
void i2c_set_cmd_vel(double x, double y, double z);
bool i2c_lock();
void i2c_release();

esp_err_t i2cnode_check(uint8_t i2caddr); 
esp_err_t i2cnode_read(uint8_t i2caddr, uint8_t regaddr, uint8_t *buf, uint16_t buflen);
esp_err_t i2cnode_write(uint8_t i2caddr, uint8_t regaddr, uint8_t *buf, uint16_t buflen); 
void i2cnode_set_u16(uint8_t i2caddr, uint8_t regaddr, uint16_t v) throw(int);
void i2cnode_init_motor();

#endif /* _I2CHANDLER_H_ */