# BNO055ESP32 [![Build Status](https://travis-ci.org/ShellAddicted/BNO055ESP32.svg?branch=master)](https://travis-ci.org/ShellAddicted/BNO055ESP32)
This idf-component provides a C++ Interface for [Bosch-Sensortec's BNO055](https://www.bosch-sensortec.com/bst/products/all_products/bno055) compatible with [Espressif's ESP32 SoC](https://www.espressif.com/en/products/hardware/esp32/overview) (running [esp-idf](https://github.com/espressif/esp-idf)).

# Compatibility
Tested on ESP32D0WDQ6 (DevKitC) with [Adafruit's BNO055 Breakout Board](https://www.adafruit.com/product/2472)

#### Supported Interfaces
- <b>I²C</b> 

# Getting Started
***NOTE: this code is not (yet) Production Ready.***   
You can use this library as a component for your project: 
```
cd <YOUR_PROJECT_ROOT>
mkdir components/
cd components/
git clone https://github.com/MS71/esp32-bno055.git
```
Remember to enable ```Compiler Options -> Enable C++ Exceptions``` using ```make menuconfig```


