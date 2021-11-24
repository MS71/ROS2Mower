#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h> 	  

#include "i2c.h"

static volatile uint16_t u8TWIByteIdx = 0;

#define I2C_DATA_INTERRUPT      0x80
#define I2C_BUS_COLLISION       0x08
#define I2C_ADDRESS_STOP_MATCH  0x40
#define BUFFER_SIZE 10

ISR( TWI_SLAVE_vect )
{
	uint8_t status = TWSSRA & 0xC0;

	if (status & I2C_DATA_INTERRUPT) // Received data from master
	{
		if (TWSSRA & (1 << TWDIR)) // Send data to the master
		{
			TWSD = i2c_TwiTxHandler( u8TWIByteIdx++ );
			//if(tx_buf_index >= sizeof(tx_buf))
			//{
			//	tx_buf_index=0;
			//}

			//TWSD = tx_buf[tx_buf_index];
			//tx_buf_index++;

			TWSCRB = (uint8_t) ((1<<TWCMD1)|(1<<TWCMD0));
		}
		else // Receive data from the master
		{
			TWSCRB |= (uint8_t) ((1<<TWCMD1)|(1<<TWCMD0));
			
			uint8_t data = TWSD;
			i2c_TwiRxHandler( u8TWIByteIdx++, data);
			//if(rx_buf_index >= sizeof(rx_buf))
			//{
			//	rx_buf_index=0;
			//}

			//rx_buf[rx_buf_index] = TWSD;
			//rx_buf_index++;
		}
	}
	else if (status & I2C_ADDRESS_STOP_MATCH)
	{
		if (TWSSRA & I2C_BUS_COLLISION)
		{
			TWSCRB = (uint8_t) (1<<TWCMD1);
		}
		else
		{
			if (TWSSRA & (1<<TWAS))
			{
				// ACK
				TWSCRB = (uint8_t) ((1<<TWCMD1)|(1<<TWCMD0));
				u8TWIByteIdx = 0;
			}
			else
			{
				// Stop Condition
				TWSCRB = (uint8_t) (1<<TWCMD1);
				//tx_buf_index = 0;
				//rx_buf_index = 0;
				u8TWIByteIdx = 0xff;
			}
		}
	}
}

void i2c_init()
{
	TWSA=TWI_ADDR<<1;
	TWSAM = 0;
	
	TWSD = 0xFF;

	TWSCRA = (1 << TWEN)   // Two-Wire Interface Enable
		| (1 << TWSHE)  // TWI SDA Hold Time Enable
		| (1 << TWASIE) // TWI Address/Stop Interrupt Enable
		| (1 << TWSIE)  // TWI Stop Interrupt Enable
		| (1 << TWDIE); // TWI Data Interrupt Enable
		
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
