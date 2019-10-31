//#define F_CPU 8000000  // CPU frequency for proper time calculation in delay function
 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "pid.h"
#include "i2c.h"

#define ENABLE_ENCODER_PASSTHROUGH

#define PIN_DRV8801_BRAKE	PA3
#define PIN_DRV8801_SLEEP	PB1
#define PIN_DRV8801_DIR 	PB2
#define PIN_DRV8801_PWM		PA7
#ifdef ENABLE_ENCODER_PASSTHROUGH
#define PIN_ENCODER_OUT		PB0
#define TOGGLE_ENCODER_OUT() \
	DDRB |= (1<<PIN_ENCODER_OUT); \
	PORTB = (PORTB & ~(1<<PIN_ENCODER_OUT)) | ((~(PORTB))&(1<<PIN_ENCODER_OUT))
#else
#define PIN_DRV8801_FAULT	PB0
#define TOGGLE_ENCODER_OUT()
#endif
#define PIN_DRV8801_CS		ADC0

#define DRV8801_SLEEP(_l_) \
	DDRB |= (1<<PIN_DRV8801_SLEEP); \
	PORTB = (PORTB & ~(1<<PIN_DRV8801_SLEEP)) | ((_l_)<<PIN_DRV8801_SLEEP)

#define DRV8801_BRAKE(_l_) \
	DDRA |= (1<<PIN_DRV8801_BRAKE); \
	PORTA = (PORTA & ~(1<<PIN_DRV8801_BRAKE)) | ((_l_)<<PIN_DRV8801_BRAKE)

#define DRV8801_DIR(_l_) \
	DDRB |= (1<<PIN_DRV8801_DIR); \
	PORTB = (PORTB & ~(1<<PIN_DRV8801_DIR)) | ((_l_)<<PIN_DRV8801_DIR)

#define DRV8801_PWM(_l_) \
	DDRA |= (1<<PIN_DRV8801_PWM); \
	PORTA = (PORTA & ~(1<<PIN_DRV8801_PWM)) | ((_l_)<<PIN_DRV8801_PWM)

#define K_P     0.3
#define K_I     0.05
#define K_D     0.05

pidData_t pid;
int16_t   pid_setPoint = 0;
int16_t   pid_processValue = 0;
int16_t   pid_Value = 0;

volatile int32_t i32_enccnt = 0;
volatile int16_t enccnt_100ms = 0;
volatile uint8_t ac_reenable_cnt = 0;

volatile uint32_t u32_time_us = 0;

void TIM0_100ms();

/*
 * timer0 overflow
 * 8MHz/256 => 32us => 31.250KHz
 */
volatile uint16_t tim0_divcnt = 0;
ISR (TIM0_OVF_vect)
{
  u32_time_us += (1000000UL*256*256)/F_CPU;

  if( ac_reenable_cnt > 0 )
  {
    ac_reenable_cnt--;
    if( ac_reenable_cnt == 0 )
    {
      ACSR |= (1<<ACIE);
    }
  }

  if( tim0_divcnt > 0 )
  {
    tim0_divcnt--;
  }
  else
  {
    tim0_divcnt = (F_CPU/(256L*256*10))-1;
    TIM0_100ms();
  }
}

/********************************************************************************
 Wheel Encoder
 max. 5kHz => 200us
********************************************************************************/
ISR(ANA_COMP_vect) 
{
#define AIN0 PA1
#define AIN1 PA2
  if( (ACSR&(1<<ACO)) == 0 )
  {
    // -
    PORTA = ( PORTA & (~((1<<AIN0)|(1<<AIN1)))) | ((0<<AIN0)|(1<<AIN1));
    DDRA |= ((1<<AIN0)|(1<<AIN1));
    if( (PORTB & (1<<PIN_DRV8801_DIR)) == 0 )
    {
      i32_enccnt--;
      enccnt_100ms--;
    }
    else
    {
      i32_enccnt++;
      enccnt_100ms++;
    }
    //ac_reenable_cnt = 4;
    //ACSR &= ~(1<<ACIE);

    DDRA = ( DDRA & (~((1<<AIN0)|(1<<AIN1)))) | ((1<<AIN0)|(0<<AIN1));
  }
  else
  {
    // +
    PORTA = ( PORTA & (~((1<<AIN0)|(1<<AIN1)))) | ((1<<AIN0)|(0<<AIN1));
    DDRA |= ((1<<AIN0)|(1<<AIN1));
    if( (PORTB & (1<<PIN_DRV8801_DIR)) == 0 )
    {
      i32_enccnt--;
      enccnt_100ms--;
    }
    else
    {
      i32_enccnt++;
      enccnt_100ms++;
    }

    //ac_reenable_cnt = 5;
    //ACSR &= ~(1<<ACIE);

    DDRA = ( DDRA & (~((1<<AIN0)|(1<<AIN1)))) | ((0<<AIN0)|(1<<AIN1));
  }

  TOGGLE_ENCODER_OUT();
  ACSR &= ~(1<<ACI);
}

/*
 * ~100ms ISR
 */
void TIM0_100ms()
{
  pid_processValue = enccnt_100ms;
  enccnt_100ms = 0;
  pid_Value = 2*pid_Controller(pid_setPoint, pid_processValue, &pid);

  if( pid_setPoint > 0 )
  {
    if( pid_Value > 0 )
    {
      OCR0B = pid_Value;
    }
    else
    {
      OCR0B = 0;
    }
    DRV8801_BRAKE(1);
    DRV8801_DIR(1);
    DRV8801_SLEEP(1);
  }
  else if( pid_setPoint < 0 )
  {
    if( pid_Value < 0 )
    {
      OCR0B = -pid_Value;
    }
    else
    {
      OCR0B = 0;
    }
    DRV8801_BRAKE(1);
    DRV8801_DIR(0);
    DRV8801_SLEEP(1);
  }
  else
  {
    DRV8801_BRAKE(0);
    DRV8801_SLEEP(0);
    OCR0B = 0;
  }

}

volatile uint8_t  u8TWIReg = 0;
volatile uint8_t  u8TWIRegIdx = 0;
volatile uint32_t u32TWITmp = 0;

/*
 * i2c_TwiRxHandler
 */
void i2c_TwiRxHandler( uint16_t idx, uint8_t data )
{
  if( idx == 0 )
  {
    u8TWIReg = data;	// store register addr
  }
  else
  {
    u32TWITmp = ((u32TWITmp>>8) | (((uint32_t)data)<<24));
    switch(u8TWIReg+idx)
    {
      case 8+2: pid_setPoint = (int16_t)((u32TWITmp>>16)&0xffff); break;
      default: break;
    }
  }
}

/*
 * i2c_TwiTxHandler
 */
uint8_t i2c_TwiTxHandler( uint16_t idx )
{
  switch(u8TWIReg+idx)
  {
    case 0:  u8TWIRegIdx = 0; u32TWITmp = u32_time_us; break;
    case 4:  u8TWIRegIdx = 0; u32TWITmp = (uint32_t)i32_enccnt; break;
    case 8:  u8TWIRegIdx = 0; u32TWITmp = (uint32_t)pid_setPoint; break;
    case 10: u8TWIRegIdx = 0; u32TWITmp = (uint32_t)pid_processValue; break;
    case 12: u8TWIRegIdx = 0; u32TWITmp = (uint32_t)pid_Value; break;
    default: break;
  }
  if((u8TWIReg+idx) > 14) u32TWITmp = 0;

  return ((u32TWITmp>>((u8TWIRegIdx++)*8))&0xff);
}

/*
 * main
 */
int main(void)
{
    DRV8801_SLEEP(1);
    DRV8801_BRAKE(0);
    DRV8801_DIR(1);
    DRV8801_PWM(0);

    ACSR  |=  (1<<ACI);    // clear Analog Comparator interrupt
    ACSR  |= 
        (0<<ACD)   |         // Comparator ON
        (0<<ACBG)  |         // Disconnect 1.23V reference from AIN0 (use AIN0 and AIN1 pins)
        (1<<ACIE)  |         // Comparator Interrupt enabled
        (0<<ACIC)  |         // input capture disabled
        (0<<ACIS1) |         // Comparator interrupt on output toggle
        (0<<ACIS0);          // 

    // fast PWM mode
    TCCR0A = (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS02) | (0 << CS01) | (0 << CS00);   // clock source = CLK/8, start PWM

    // Overflow Interrupt erlauben
    TIMSK0 |= (1<<TOIE0);

    pid_setPoint = 0;

    pid_Init((int16_t)(K_P*SCALING_FACTOR),
	     (int16_t)(K_I*SCALING_FACTOR),
	     (int16_t)(K_D*SCALING_FACTOR), 
             &pid);

    i2c_init();

    sei();

    for(;;)
    {
#if 0
      int16_t v = 100;
      pid_setPoint = v;
      _delay_ms(30000);

      pid_setPoint = 0;
      _delay_ms(1000);

      pid_setPoint = -v;
      _delay_ms(30000);

      pid_setPoint = 0;
      _delay_ms(1000);
#endif
    }

    return 0;  // the program executed successfully
}

/*
 * EOF
 */
