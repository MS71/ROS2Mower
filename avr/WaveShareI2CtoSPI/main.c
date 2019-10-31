//#define F_CPU 8000000  // CPU frequency for proper time calculation in delay function
 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "i2c.h"

#define PIN_BUSY 	2
#define PIN_RST 	1
#define PIN_DC	 	0
#define PIN_CS	 	7

#define PIN_CLK 	0
#define PIN_DIN 	1

#define SET_RST(_l_) \
	DDRA |= (1<<PIN_RST); \
	PORTA = (PORTA & ~(1<<PIN_RST)) | ((_l_)<<PIN_RST)

#define SET_DC(_l_) \
	DDRA |= (1<<PIN_DC); \
	PORTA = (PORTA & ~(1<<PIN_DC)) | ((_l_)<<PIN_DC)

#define SET_CS(_l_) \
	DDRA |= (1<<PIN_CS); \
	PORTA = (PORTA & ~(1<<PIN_CS)) | ((_l_)<<PIN_CS)

#define SET_CLK(_l_) \
	DDRB |= (1<<PIN_CLK); \
	PORTB = (PORTB & ~(1<<PIN_CLK)) | ((_l_)<<PIN_CLK)

#define SET_DIN(_l_) \
	DDRB |= (1<<PIN_DIN); \
	PORTB = (PORTB & ~(1<<PIN_DIN)) | ((_l_)<<PIN_DIN)

/*
 * i2c_TwiRxHandler
 */
void i2c_TwiRxHandler( uint16_t idx, uint8_t data )
{
  static uint8_t data_0 = 0;
  if( idx == 0 )
  {
    data_0 = data;
    SET_RST((data_0>>1)&1);
    SET_DC((data_0>>2)&1);
    SET_CS((data_0>>3)&1);    
  }
  else
  {
	if( ((data_0>>7)&1) == 1 )
	{
      // Data
      SET_DC(1);
      SET_CS(0);
	}
	else if( ((data_0>>6)&1) == 1 )
	{
	  // Command
      SET_DC(0);
      SET_CS(0);
	}
	SET_CLK(0);
	SET_DIN(((data&(1<<7))==0)?0:1);
	SET_CLK(1);
	SET_CLK(0);
	SET_DIN(((data&(1<<6))==0)?0:1);
	SET_CLK(1);
	SET_CLK(0);
	SET_DIN(((data&(1<<5))==0)?0:1);
	SET_CLK(1);
	SET_CLK(0);
	SET_DIN(((data&(1<<4))==0)?0:1);
	SET_CLK(1);
	SET_CLK(0);
	SET_DIN(((data&(1<<3))==0)?0:1);
	SET_CLK(1);
	SET_CLK(0);
	SET_DIN(((data&(1<<2))==0)?0:1);
	SET_CLK(1);
	SET_CLK(0);
	SET_DIN(((data&(1<<1))==0)?0:1);
	SET_CLK(1);
	SET_CLK(0);
	SET_DIN(((data&(1<<0))==0)?0:1);
	SET_CLK(1);		
	if( ((data_0>>7)&1) == 1 )
	{
      SET_CS(1);
	}
	else if( ((data_0>>6)&1) == 1 )
	{
      SET_CS(1);
	}
  }
}

/*
 * i2c_TwiTxHandler
 */
uint8_t i2c_TwiTxHandler( uint16_t idx )
{
	uint8_t d = 0;
    DDRA &= ~(1<<PIN_BUSY);
    PORTA |= (1<<PIN_BUSY);
	d |= (((PINA >> PIN_BUSY)&1)<<0);
	d |= (((PINA >> PIN_RST)&1)<<1);
	d |= (((PINA >> PIN_DC)&1)<<2);
	d |= (((PINA >> PIN_CS)&1)<<3);
    return d;
}

/*
 * main
 */
int main(void)
{
    DDRA &= ~(1<<PIN_BUSY);
    SET_RST(0);
    SET_DC(0);
    SET_CS(1);
    SET_CLK(0);
    SET_DIN(0);
    SET_RST(1);

    i2c_init();

    sei();

    for(;;)
    {
    }

    return 0;  // the program executed successfully
}

/*
 * EOF
 */
