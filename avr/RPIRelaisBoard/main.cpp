#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

extern "C" {
#include "i2c.h"
}

#define BIT_REL1 1
#define PORT_REL1 B
#define BIT_REL2 0
#define PORT_REL2 B

#define CONCATx(a, b) a##b
#define REG(a, b) CONCATx(a, b)

uint8_t i2c_reg_addr = 0;

/*
 * i2c_TwiRxHandler
 */
void i2c_TwiRxHandler(uint16_t idx, uint8_t data)
{
  if( idx == 0 )
  {
    i2c_reg_addr = data;
  }
  else
  {
    switch( i2c_reg_addr++ )
    {
      case 0:
        if( data == 0 )
        {
          REG(PORT, PORT_REL1) &= ~(1 << BIT_REL1);
        }
        else
        {
          REG(PORT, PORT_REL1) |= (1 << BIT_REL1);
        }
        break;
        
      case 1: 
        if( data == 0 )
        {
          REG(PORT, PORT_REL2) &= ~(1 << BIT_REL2);
        }
        else
        {
          REG(PORT, PORT_REL2) |= (1 << BIT_REL2);
        }
        break;

      default: 
        break;
    }
  }
}

/*
 * i2c_TwiTxHandler
 */
uint8_t i2c_TwiTxHandler(uint16_t idx)
{
    return 0;
}


int main()
{
  i2c_init();

  REG(DDR, PORT_REL1) |= (1 << BIT_REL1);
  REG(DDR, PORT_REL2) |= (1 << BIT_REL2);
  REG(PORT, PORT_REL1) &= ~(1 << BIT_REL1);
  REG(PORT, PORT_REL2) &= ~(1 << BIT_REL2);
  
  sei();
  while(1)
  {
  }
}

/*
 * EOF
 */
