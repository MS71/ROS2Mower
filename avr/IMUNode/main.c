#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdarg.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdarg.h>
#include <stdio.h>
#include <avr/eeprom.h>

#include "i2c.h"

#define PIN_INT         	PB1
#define PIN_BNO055_RESET	PB3
#define PIN_BNO055_INT	    PB4

#define INTOUT(_v_) \
    { \
	  DDRB |= (1<<PIN_INT); \
      PORTB = (PORTB & ~(1<<PIN_INT)) | ((((_v_)>>0)&1)<<PIN_INT); \
	}

#define BNO055_RESET(_v_) \
    { \
	  DDRB |= (1<<PIN_INT); \
      PORTB = (PORTB & ~(1<<PIN_INT)) | ((((_v_)>>0)&1)<<PIN_INT); \
	}

uint16_t boot_counter = 0;
uint16_t eeFooByteArray1[1] EEMEM;


/*
 * i2c_TwiRxHandler
 */
void i2c_TwiRxHandler( uint16_t idx, uint8_t data )
{
    if( idx == 0 )
    {
        BNO055_RESET(((data>>0)&1));
    }
}

/*
 * i2c_TwiTxHandler
 */
uint8_t i2c_TwiTxHandler( uint16_t idx )
{
    uint8_t reg = 0;
    if( idx == 0 )
    {
        reg |= ((PINB>>PIN_BNO055_RESET)&0x1)<<0;
        reg |= ((PINB>>PIN_BNO055_INT)&0x1)<<1;
    }
    return reg;
}

/*
 * main
 */
int main(void)
{
    boot_counter = eeprom_read_word(&eeFooByteArray1[0]) + 1;
    eeprom_write_word(&eeFooByteArray1[0], boot_counter);

    DDRB &= ~(1 << PIN_INT);
    PORTB |= (1 << PIN_INT);

    DDRB &= ~(1 << PIN_BNO055_INT);
    PORTB |= (1 << PIN_BNO055_INT);

    BNO055_RESET(0);
    
    i2c_init();

    for(;;) 
    {
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();
    }
    return 0; // the program executed successfully
}
/*
 * EOF
 */
