#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h> 	  

#include "i2c.h"

#define TW_ADDRESS				0x02
#define TWI_STATUS				TWSSRA&0xCF
#define TW_ADDRESS_ACK			TWSCRB=(1<<TWCMD1)|(1<<TWCMD0)
#define TW_STOP_ACK				TWSCRB=(1<<TWCMD1)
#define TW_DATA_ACK				TWSCRB=0x03
#define TW_DATA_NACK			TWSCRB=0x07
#define TW_ADDRESS_MATCH		0x41
#define TW_STOP					0x40
#define TW_BUS_COLLISION		0x48
#define TW_DATA_RX				0x81
#define TW_DATA_TX				0x83

//#define FL_TW_BUS_COLLISION 0
//#define FL_TW_BUS_ERROR		1
//#define FL_TWI_DONE			2
//#define FL_BUF_ADDR			3

//volatile uint8_t data;
//char upper_buffer[buffer_size];
//char lower_buffer[buffer_size];
//volatile uint8_t buffer_address;

static volatile uint16_t u8TWIByteIdx = 0;

ISR(TWI_SLAVE_vect)
{
    switch(TWI_STATUS) {
		
    case TW_ADDRESS_MATCH:
	TW_ADDRESS_ACK;
	u8TWIByteIdx = 0;
	//SetFlag(FL_BUF_ADDR);
	break;
	
    case TW_STOP:
	TW_STOP_ACK;
	u8TWIByteIdx = 0xff;
	//SetFlag(FL_TWI_DONE);
	break;
	
    case TW_DATA_TX:
	{
		TWSD = i2c_TwiTxHandler( u8TWIByteIdx++);
		TW_DATA_ACK;
		break;		
	}

    case TW_DATA_RX:
	{
		uint8_t data = TWSD;
		i2c_TwiRxHandler( u8TWIByteIdx++, data);
		TW_DATA_ACK;
		break;		
	}
#if 0			
			if (FlagIsSet(FL_BUF_ADDR)) {
				//buffer_address=data;
				ClearFlag(FL_BUF_ADDR);
				TW_DATA_ACK;
			}
			else {
				if (buffer_address<=0x0F) {
					upper_buffer[buffer_address++]=data;
					TW_DATA_ACK;
				}
				else if (buffer_address<=0x1F) {
					lower_buffer[(buffer_address++)-0x10]=data;
					TW_DATA_ACK;
				}
			}
#endif
	break;
    }
}

void i2c_init()
{
	TWSCRA=(1<<TWDIE)|(1<<TWSIE)|(1<<TWASIE)|(1<<TWEN)|(1<<TWSHE);
	TWSA=TWI_ADDR;
	u8TWIByteIdx = 0xff;
}

uint8_t i2c_idle()
{
	if( u8TWIByteIdx == 0xff )
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
