/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "BNO055ESP32.h"

extern "C" {
	void app_main(void);
}

static const int I2CPortNumber = 0;
static const int SDAPin = 26;
static const int SCLPin = 27;

BNO055* bno055 = NULL;

void app_main()
{
    printf("Hello BNO055\n");

    static i2c_config_t Config;
  	memset( &Config, 0, sizeof( i2c_config_t ) );
  	Config.mode = I2C_MODE_MASTER;
  	Config.sda_io_num = (gpio_num_t)SDAPin;
  	Config.sda_pullup_en = GPIO_PULLUP_ENABLE;
  	Config.scl_io_num = (gpio_num_t)SCLPin;
  	Config.scl_pullup_en = GPIO_PULLUP_ENABLE;
  	Config.master.clk_speed = 500000;
  	i2c_param_config( (i2c_port_t)I2CPortNumber, &Config );
  	i2c_driver_install( (i2c_port_t)I2CPortNumber, Config.mode, 0, 0, 0 );
  	i2c_set_timeout((i2c_port_t)I2CPortNumber, (I2C_APB_CLK_FREQ / Config.master.clk_speed)*1024);

    vTaskDelay(750 / portTICK_PERIOD_MS);

    /*
     * init BNO055 ...
     */
    bno055 = new BNO055((i2c_port_t)I2CPortNumber,0x28);
    bno055->begin();  // BNO055 is in CONFIG_MODE until it is changed
    bno055->enableExternalCrystal();
    //bno.setSensorOffsets(storedOffsets);
    //bno055->setAxisRemap(BNO055_REMAP_CONFIG_P5, BNO055_REMAP_SIGN_P0); // see datasheet, section 3.4
    /* you can specify a PoWeRMode using:
       - setPwrModeNormal(); (Default on startup)
       - setPwrModeLowPower();
       - setPwrModeSuspend(); (while suspended bno055 must remain in CONFIG_MODE)
     */
    bno055->setOprModeNdof();

    while(1)
    {
      int8_t t = bno055->getTemp();
      bno055_vector_t v = bno055->getVectorEuler();
      bno055_calibration_t c = bno055->getCalibration();
      printf("Temp: %d°C Calib: %d %d %d %d Euler: %f %f %f\n",
      t,
      c.sys,c.gyro,c.mag,c.accel,
      v.x,v.y,v.z);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
