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

#include "pid.h"

#include "i2c.h"


#define MOTOR_GEAR_N (18*7*23UL)
#define MOTOR_RPS(_rps_) (int32_t)(((int32_t)_rps_) * MOTOR_GEAR_N)
#define MOTOR_RPM(_rpm_) (int32_t)(MOTOR_RPS(_rpm_) / 60.0)

#define PIN_INT  	PA5

#ifdef VARIANT_A_T84_L298_SE
#define BIT_ENCA	1
#define PORT_ENCA	A
#define BIT_ENCB	0
#define PORT_ENCB	A

#define BIT_MA_A	2
#define PORT_MA_A   A
#define BIT_MA_B	0
#define PORT_MA_B	B
#define BIT_MB_A	1
#define PORT_MB_A	B
#define BIT_MB_B	3
#define PORT_MB_B	A
#define BIT_PWMA	2
#define PORT_PWMA	B
#define BIT_PWMB	7
#define PORT_PWMB	A
#endif

#ifdef VARIANT_D_T841_L298_SE
#define BIT_ENCA	2
#define PORT_ENCA	B
#define BIT_ENCB	7
#define PORT_ENCB	A
#define BIT_MA_A	0
#define PORT_MA_A   A
#define BIT_MA_B	0
#define PORT_MA_B	B
#define BIT_MB_A	1
#define PORT_MB_A	B
#define BIT_MB_B	3
#define PORT_MB_B	A
#define BIT_PWMA	1
#define PORT_PWMA	A
#define BIT_PWMB	2
#define PORT_PWMB	A
#endif

#ifdef VARIANT_D_T841_VNH2SP30_SE
#define BIT_ENCA	2
#define PORT_ENCA	B
#define BIT_MA_A	0
#define PORT_MA_A   A
#define BIT_MA_B	0
#define PORT_MA_B	B
#define BIT_PWMA	1
#define PORT_PWMA	A
#endif

#ifdef VARIANT_C_T84_VNH2SP30_SE
#define BIT_ENCA	1
#define PORT_ENCA	A

#define BIT_MA_A	2
#define PORT_MA_A   A
#define BIT_MA_B	0
#define PORT_MA_B	B
#define BIT_MA_EN	1
#define PORT_MA_EN  B
#define BIT_PWMA	2
#define PORT_PWMA	B
#endif


#define CONCATx(a,b) a##b
#define REG(a,b) CONCATx(a,b)

//#define DEBUG_INT(_ch_,_v_) if( main_loop_int_pulse == _ch_ ) INTOUT(_v_);
#define DEBUG_INT(_ch_,_v_) 

#define INTOUT(_v_) \
    { \
	  DDRA |= (1<<PIN_INT); \
      PORTA = (PORTA & ~(1<<PIN_INT)) | ((((_v_)>>0)&1)<<PIN_INT); \
	}

#if defined(PORT_MA_A) && defined(BIT_MA_B)
#define MODE_A(_m_) \
    { \
	  REG(DDR,PORT_MA_A) |= (1<<BIT_MA_A); \
	  REG(DDR,PORT_MA_B) |= (1<<BIT_MA_B); \
	  if((_m_>>0)&1) REG(PORT,PORT_MA_A) |= (1<<BIT_MA_A); else REG(PORT,PORT_MA_A) &= ~(1<<BIT_MA_A); \
	  if((_m_>>1)&1) REG(PORT,PORT_MA_B) |= (1<<BIT_MA_B); else REG(PORT,PORT_MA_B) &= ~(1<<BIT_MA_B); \
	}
#endif

#if defined(PORT_MB_A) && defined(BIT_MB_B)
#define MODE_B(_m_) \
    { \
	  REG(DDR,PORT_MB_A) |= (1<<BIT_MB_A); \
	  REG(DDR,PORT_MB_B) |= (1<<BIT_MB_B); \
	  if((_m_>>0)&1) REG(PORT,PORT_MB_A) |= (1<<BIT_MB_A); else REG(PORT,PORT_MB_A) &= ~(1<<BIT_MB_A); \
	  if((_m_>>1)&1) REG(PORT,PORT_MB_B) |= (1<<BIT_MB_B); else REG(PORT,PORT_MB_B) &= ~(1<<BIT_MB_B); \
	}
#endif

#define TOGGLE_PIN(_port_,_pin_) \
	_port_ = (_port_&(~(1<<(_pin_)))) | ((~(_port_))&(1<<(_pin_)))

int16_t k_p = (int16_t)((0.20)*SCALING_FACTOR);
int16_t k_i = (int16_t)((0.01)*SCALING_FACTOR);
int16_t k_d = (int16_t)((0.01)*SCALING_FACTOR);

uint8_t main_loop_int_pulse = 0;
uint8_t i2c_wdt_reset = 0;
uint16_t boot_counter = 0;
uint16_t eeFooByteArray1[1] EEMEM;

uint8_t motor_enc_tp = 2;
uint8_t motor_stop_tp = 16;

typedef struct
{
	pidData_t pid;
	int16_t   pid_setPoint;
	int16_t   pid_processValue;
	int16_t   pid_Value;
	uint8_t   pwm;

	int16_t   i16_encoder;
  	uint32_t  u32_encperiod;
    uint16_t  u16_t1_cnt;
    uint16_t  u16_t1_tcnt1;

	int64_t   i64_encoder;
	int8_t    i8_encoder_step;
	uint8_t   u8_encoder_flag;
} Motor;

Motor motor_A = {0};
Motor motor_B = {0};

uint16_t motor_break_cntdown = 0;

enum {
		MODE_OFF=0,
		MODE_BREAK=1,
		MODE_PID=2,
		MODE_PWM=3,
		MODE_TEST_01=4,
		MODE_TEST_01_PWM=5,
} motor_mode = MODE_OFF;

#if 0
#define MOTOR_TEST_01_STEPS 8
struct
{
	int16_t  speed_A;
	int16_t  speed_B;
	uint16_t delay_ms;
} motor_test_01[MOTOR_TEST_01_STEPS] = {};
#endif

volatile uint64_t u32_time_us = 0;

volatile uint8_t u8Tick = 0;
volatile uint8_t u8HandlePIDFlag = 0;

void handlePID();
void TIM0_65536us();

static uint64_t get_time_us()
{
	return (uint64_t)u32_time_us*8192;
}

#ifdef DEBUGUART
/********************************************************************************
 uart_putchar
********************************************************************************/
volatile uint8_t uart_char = 0;
volatile uint8_t uart_char_state = 0;
static int uart_putchar(char ch, FILE *stream)
{
	if (ch == '\n') uart_putchar('\r', stream);
	while(uart_char_state!=0xff);
	uart_char = ch;
	uart_char_state = 0;
	return 1;
}

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void uart_handle()
{
	if( uart_char_state != 0xff )
	{
		uint8_t bitidx = (uart_char_state++); // 300.48 BAUD

		if( bitidx == 0 )
		{
			INTOUT(0);
		}
		else if( bitidx < 9 )
		{
			INTOUT( ((uart_char>>(bitidx-1))&1)==0?0:1 );
		}
		else if( bitidx == 9 )
		{
			INTOUT(1);
		}
		else
		{
			/* done */
			uart_char_state = 0xff;
			//uart_char_state = 0;
			//uart_char++;
			return;
		}
	}
}
#endif

volatile uint8_t tim0_divcnt = 0;
/*
 * timer0/1 overflow
 */
#if defined( __AVR_ATtiny84__ )
ISR (TIM1_OVF_vect)
{
    DEBUG_INT(3,1);
    DEBUG_INT(3,0);
    DEBUG_INT(4,1);

    if( motor_A.u16_t1_cnt < 0xffff ) motor_A.u16_t1_cnt++;
    if( (motor_A.u16_t1_cnt&63)==0 ) /* > 512ms */
    {
      DEBUG_INT(14,1);
      DEBUG_INT(14,0);
      motor_A.u32_encperiod = (((uint32_t)motor_A.u32_encperiod*(motor_stop_tp-1)) + 1000000UL)/motor_stop_tp;
    }

    if( motor_B.u16_t1_cnt < 0xffff ) motor_B.u16_t1_cnt++;
    if( (motor_B.u16_t1_cnt&63)==0 ) /* > 512ms */
    {
      DEBUG_INT(15,1);
      DEBUG_INT(15,0);
      motor_B.u32_encperiod = (((uint32_t)motor_B.u32_encperiod*(motor_stop_tp-1)) + 1000000UL)/motor_stop_tp;
    }
    
    //sei();

    u32_time_us++;
    tim0_divcnt++;
        
#ifdef DEBUGUART
	uart_handle();
#endif

  /*
   * ~8ms
   */
  u8HandlePIDFlag = 1;

  if( (tim0_divcnt&7) == 0 )
  { /* 64ms */
    DEBUG_INT(13,1);
	u8Tick = 1;
    DEBUG_INT(13,0);
  }
  DEBUG_INT(4,0);
}
#endif

/********************************************************************************
 Wheel Encoder
 max. 5kHz => 200us
********************************************************************************/
ISR(PCINT0_vect)
{
    uint16_t tcnt1 = TCNT1;
    uint8_t _t1ov = (TOV1!=0)?1:0;

	DEBUG_INT(5,1);

#ifdef BIT_ENCA
	uint8_t flaga = 0;
	{
		static uint8_t _enca = 0;
		uint8_t enca = (REG(PIN,PORT_ENCA) & (1<<BIT_ENCA));
		if( enca != _enca )
		{
			_enca = enca;
			if( enca == 0 )
			{
				flaga = 1;
                DEBUG_INT(6,1);
			}
		}
	}
#endif

#ifdef BIT_ENCB
	uint8_t flagb = 0;
	{
		static uint8_t _encb = 0;
		uint8_t encb = (REG(PIN,PORT_ENCB) & (1<<BIT_ENCB));
		if( encb != _encb )
		{
			_encb = encb;
			if( encb == 0 )
			{
				flagb = 1;
                DEBUG_INT(6,1);
			}
		}
	}
#endif

	//sei();	// allow other IRQs e.g. i2c

#ifdef BIT_ENCA
	if( flaga != 0 )
	{
        uint32_t p = (((uint32_t)(motor_A.u16_t1_cnt+_t1ov)<<16) + (uint32_t)tcnt1 - (uint32_t)motor_A.u16_t1_tcnt1)/8;
        motor_A.u32_encperiod = (((uint32_t)motor_A.u32_encperiod*(motor_enc_tp-1)) + p)/motor_enc_tp;
        motor_A.u16_t1_cnt = 0;
        motor_A.u16_t1_tcnt1 = tcnt1;
        motor_A.i16_encoder += motor_A.i8_encoder_step;
        motor_A.i64_encoder += motor_A.i8_encoder_step;
        motor_A.u8_encoder_flag = 1;
        DEBUG_INT(8,1);
        DEBUG_INT(8,0);
	}
#endif

#ifdef BIT_ENCB
	if( flagb != 0 )
	{
        uint32_t p = (((uint32_t)(motor_B.u16_t1_cnt+_t1ov)<<16) + (uint32_t)tcnt1 - (uint32_t)motor_B.u16_t1_tcnt1)/8;
        motor_B.u32_encperiod = (((uint32_t)motor_B.u32_encperiod*(motor_enc_tp-1)) + p)/motor_enc_tp;
        motor_B.u16_t1_cnt = 0;
        motor_B.u16_t1_tcnt1 = tcnt1;
        motor_B.i16_encoder += motor_B.i8_encoder_step;
        motor_B.i64_encoder += motor_B.i8_encoder_step;
        motor_B.u8_encoder_flag = 1;
        DEBUG_INT(9,1);
        DEBUG_INT(9,0);
	}
#endif

	DEBUG_INT(5,0);
	DEBUG_INT(6,0);
}

#ifdef MODE_A
static void setModeA(uint8_t mode,uint8_t pwm)
{
	  MODE_A(mode);
	  OCR0A = pwm;
}
#endif

#ifdef MODE_B
static void setModeB(uint8_t mode,uint8_t pwm)
{
	  MODE_B(mode);
	  OCR0B = pwm;
}
#endif

void calcPID(Motor *m,void(*set_mode)(uint8_t mode,uint8_t pwm))
{    
  cli();
  int16_t sp = m->pid_setPoint;
  if( m->u32_encperiod > 0 )
  {
    DEBUG_INT(16,1);
    DEBUG_INT(16,0);
    m->pid_processValue = (1000000UL) / m->u32_encperiod;
    sei();
  }
  else
  {
    DEBUG_INT(17,1);
    DEBUG_INT(17,0);
    sei();
    return;
  }

  if( m->pid_setPoint > 0 )
  {
    m->i8_encoder_step = 1;

    DEBUG_INT(18,1);
    DEBUG_INT(18,0);

    m->pid_Value = pid_Controller(sp, m->pid_processValue, &m->pid);
    m->pwm = ((((int32_t)m->pid_Value)*255)/SCALING_FACTOR);
    
    if( m->pid_Value > 0 )
    {
      set_mode(2,m->pwm);
    }
    else
    {
      m->pwm = 0;
      set_mode(2,0);
    }
  }
  else if( m->pid_setPoint < 0 )
  {
     m->i8_encoder_step = -1;

    DEBUG_INT(19,1);
    DEBUG_INT(19,0);

     m->pid_Value = pid_Controller(-m->pid_setPoint, m->pid_processValue, &m->pid);
     m->pwm = ((((int32_t)m->pid_Value)*255)/SCALING_FACTOR);
     if( m->pid_Value > 0 )
     {
       set_mode(1,m->pwm);
     }
     else
     {
       m->pwm = 0;
       set_mode(1,0);
     }
  }
  else
  {
    DEBUG_INT(20,1);
    DEBUG_INT(20,0);

    m->pwm = 0;
    set_mode(0,0);
  }
}

void handlePID()
{
	if( (motor_mode==MODE_PID) ||
		(motor_mode==MODE_TEST_01) )
	{
#ifdef MODE_A
		calcPID(&motor_A,setModeA);
#endif        
#ifdef MODE_B
		calcPID(&motor_B,setModeB);
#endif
	}
	else if(motor_mode==MODE_OFF)
	{
#ifdef MODE_A
		setModeA(0,0);
#endif        
#ifdef MODE_B
		setModeB(0,0);
#endif
	}
	else if(motor_mode==MODE_BREAK)
	{
#ifdef MODE_A
		setModeA(0,0xff);
#endif        
#ifdef MODE_B
		setModeB(0,0xff);
#endif        
	}
}

#if 0
/*
 * Motor Test 01
 */
void handle_motor_test_01()
{
	if( motor_mode==MODE_TEST_01 || motor_mode==MODE_TEST_01_PWM )
	{
		static uint64_t _u64_time_us = 0;
		static uint8_t step = 0;

		if( step >= MOTOR_TEST_01_STEPS )
		{
			step = 0;
		}

		if( motor_test_01[step].delay_ms != 0 )
		{
			if( get_time_us() >= _u64_time_us )
			{
				if( motor_mode==MODE_TEST_01 )
				{
					motor_A.pid_setPoint = motor_test_01[step].speed_A;
					motor_B.pid_setPoint = motor_test_01[step].speed_B;
				}
				else if( motor_mode==MODE_TEST_01_PWM )
				{
					if( motor_test_01[step].speed_A > 0 )
					{
						setModeA(1,motor_test_01[step].speed_A);
					}
					else if( motor_test_01[step].speed_A < 0 )
					{
						setModeA(2,-motor_test_01[step].speed_A);
					}
					else
					{
						setModeA(0,0);
					}
					if( motor_test_01[step].speed_B > 0 )
					{
						setModeB(1,motor_test_01[step].speed_B);
					}
					else if( motor_test_01[step].speed_B < 0 )
					{
						setModeB(2,-motor_test_01[step].speed_B);
					}
					else
					{
						setModeB(0,0);
					}
				}
				/* next ...
				 */
				_u64_time_us = get_time_us() +
					(1000UL*motor_test_01[step].delay_ms);
				step++;
			}
		}
		else
		{
			step = 0;
		}
	}
}
#endif

volatile uint8_t u8TWIReg = 0;
volatile uint8_t u8TWITmp[16] = {};
volatile uint8_t u8TWITmpIdx = 0;

#define TWI_REG_U64_CLOCK_US	 ((0<<8)|8)
#define TWI_REG_U16_MODE  	     ((8<<8)|2)
#define TWI_REG_U16_AUTOBREAK    ((10<<8)|2)
#define TWI_REG_U16_BOOTCOUNTER  ((12<<8)|2)
#define TWI_REG_U8_MAINLOOPPULSE ((14<<8)|1)
#define TWI_REG_U8_I2CWDTRESET   ((15<<8)|1)

// PID Parameter
#define TWI_REG_S16_PID_K_PID	((0x10<<8)|6)

// Motor A
#define TWI_REG_S16_MA_PID_SP	(((0x20+0)<<8)|2)
#define TWI_REG_S16_MA_PID_PV	(((0x20+2)<<8)|2)
#define TWI_REG_S16_MA_PID_OV	(((0x20+4)<<8)|2)
#define TWI_REG_U8_MA_DIR		(((0x20+6)<<8)|1)
#define TWI_REG_U8_MA_PWM		(((0x20+7)<<8)|1)

#define TWI_REG_S64_MA_ENC		(((0x30+0)<<8)|8)
#define TWI_REG_S16_MA_ENC		(((0x30+8)<<8)|2)

// Motor B
#define TWI_REG_S16_MB_PID_SP	(((0x40+0)<<8)|2)
#define TWI_REG_S16_MB_PID_PV	(((0x40+2)<<8)|2)
#define TWI_REG_S16_MB_PID_OV	(((0x40+4)<<8)|2)
#define TWI_REG_U8_MB_DIR		(((0x40+6)<<8)|1)
#define TWI_REG_U8_MB_PWM		(((0x40+7)<<8)|1)

#define TWI_REG_S64_MB_ENC		(((0x50+0)<<8)|8)
#define TWI_REG_S16_MB_ENC		(((0x50+8)<<8)|2)

#define TWI_REG_BYTES(_val_) \
	((_val_)&0xff)

#define TWI_REG_RX(_val_) \
	((((_val_)>>8)&0xff)+(((_val_)>>0)&0xff))

#define TWI_REG_TX(_val_) \
	((((_val_)>>8)&0xff)+0)

/*
 * i2c_TwiRxHandler
 */
void i2c_TwiRxHandler( uint16_t idx, uint8_t data )
{
  DEBUG_INT(12,1);
  if( idx == 0 )
  {
    u8TWIReg = data;	// store register addr
  }
  else
  {
	 uint8_t i = 0;
	 uint8_t v = 0;
	 uint8_t n = 0;
	 u8TWITmp[(u8TWITmpIdx++)&15] = data;

	 switch(u8TWIReg+idx)
     {
#if 1
		 case TWI_REG_RX(TWI_REG_U8_MAINLOOPPULSE):
			main_loop_int_pulse = 0;
			n = TWI_REG_BYTES(TWI_REG_U8_MAINLOOPPULSE);
			for(i=0;i<n;i++)
			{
				v = u8TWITmp[(u8TWITmpIdx-(i+1))&15];
				u8TWITmp[(u8TWITmpIdx-(i+1))&15] = 0;
				main_loop_int_pulse |= v<<((n-1-i)*8);
			}
		 break;
		 case TWI_REG_RX(TWI_REG_U8_I2CWDTRESET):
			if( i2c_wdt_reset == 0 )
			{
			  i2c_wdt_reset = 1;
			  switch(data)
		  	  {
                            case 1: wdt_enable(WDTO_1S); break;
                            case 2: wdt_enable(WDTO_2S); break;
                            case 4: wdt_enable(WDTO_4S); break;
                            case 8: wdt_enable(WDTO_8S); break;
			    default: wdt_enable(WDTO_1S); break;
			  }
			}
			DEBUG_INT(11,1);
			wdt_reset();
			DEBUG_INT(11,0);
		 break;
		 case TWI_REG_RX(TWI_REG_U16_BOOTCOUNTER):
			boot_counter = 0;
			n = TWI_REG_BYTES(TWI_REG_U16_BOOTCOUNTER);
			for(i=0;i<n;i++)
			{
				v = u8TWITmp[(u8TWITmpIdx-(i+1))&15];
				u8TWITmp[(u8TWITmpIdx-(i+1))&15] = 0;
				boot_counter |= v<<((n-1-i)*8);
			}
			eeprom_write_word(&eeFooByteArray1[0], boot_counter);
		 break;
		 case TWI_REG_RX(TWI_REG_U16_MODE):
			motor_mode = 0;
			n = TWI_REG_BYTES(TWI_REG_U16_MODE);
			for(i=0;i<n;i++)
			{
				v = u8TWITmp[(u8TWITmpIdx-(i+1))&15];
				u8TWITmp[(u8TWITmpIdx-(i+1))&15] = 0;
				motor_mode |= v<<((n-1-i)*8);
			}
		 break;
		 case TWI_REG_RX(TWI_REG_U16_AUTOBREAK):
			motor_break_cntdown = 0;
			n = TWI_REG_BYTES(TWI_REG_U16_AUTOBREAK);
			for(i=0;i<n;i++)
			{
				v = u8TWITmp[(u8TWITmpIdx-(i+1))&15];
				u8TWITmp[(u8TWITmpIdx-(i+1))&15] = 0;
				motor_break_cntdown |= v<<((n-1-i)*8);
			}
		 break;
		 case TWI_REG_RX(TWI_REG_S16_PID_K_PID):
			k_p = 0;
			k_i = 0;
			k_d = 0;
			n = 2;
			for(i=0;i<n;i++)
			{
				v = u8TWITmp[(u8TWITmpIdx-(i+0+1))&15];
				u8TWITmp[(u8TWITmpIdx-(i+0+1))&15] = 0;
				k_p |= v<<((n-1-i)*8);
			}
			n = 2;
			for(i=0;i<n;i++)
			{
				v = u8TWITmp[(u8TWITmpIdx-(i+2+1))&15];
				u8TWITmp[(u8TWITmpIdx-(i+2+1))&15] = 0;
				k_i |= v<<((n-1-i)*8);
			}
			n = 2;
			for(i=0;i<n;i++)
			{
				v = u8TWITmp[(u8TWITmpIdx-(i+4+1))&15];
				u8TWITmp[(u8TWITmpIdx-(i+4+1))&15] = 0;
				k_d |= v<<((n-1-i)*8);
			}
			pid_Init(k_p,k_i,k_d, &(motor_A.pid));
			pid_Init(k_p,k_i,k_d, &(motor_B.pid));

			motor_A.i64_encoder = 0;
			motor_B.i64_encoder = 0;

		 break;
		 case TWI_REG_RX(TWI_REG_S16_MA_PID_SP):
			motor_A.pid_setPoint = 0;
			n = TWI_REG_BYTES(TWI_REG_S16_MA_PID_SP);
			for(i=0;i<n;i++)
			{
				v = u8TWITmp[(u8TWITmpIdx-(i+1))&15];
				u8TWITmp[(u8TWITmpIdx-(i+1))&15] = 0;
				motor_A.pid_setPoint |= v<<((n-1-i)*8);
			}
		 break;
		 case TWI_REG_RX(TWI_REG_S16_MB_PID_SP):
			motor_B.pid_setPoint = 0;
			n = TWI_REG_BYTES(TWI_REG_S16_MB_PID_SP);
			for(i=0;i<n;i++)
			{
				v = u8TWITmp[(u8TWITmpIdx-(i+1))&15];
				u8TWITmp[(u8TWITmpIdx-(i+1))&15] = 0;
				motor_B.pid_setPoint |= v<<((n-1-i)*8);
			}
		 break;
#endif
      default:
	     break;
     }
  }
  DEBUG_INT(12,0);
}

/*
 * i2c_TwiTxHandler
 */
uint8_t i2c_TwiTxHandler( uint16_t idx )
{
  uint8_t i = 0;
  uint8_t v = 0;
  DEBUG_INT(12,1);
  switch(u8TWIReg+idx)
  {
	  case TWI_REG_TX(TWI_REG_U64_CLOCK_US):
      {
          uint64_t t = get_time_us();
	      for(i=0;i<TWI_REG_BYTES(TWI_REG_U64_CLOCK_US);i++)
		  {
	        u8TWITmp[(u8TWITmpIdx+i)&15] = (t>>(i*8))&0xff;
		  }
      }
 	  break;
	  case TWI_REG_TX(TWI_REG_U16_BOOTCOUNTER):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_U16_BOOTCOUNTER);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (boot_counter>>(i*8))&0xff;
		}
 	  break;
	  case TWI_REG_TX(TWI_REG_U16_MODE):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_U16_MODE);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (motor_mode>>(i*8))&0xff;
		}
 	  break;
	  case TWI_REG_TX(TWI_REG_U16_AUTOBREAK):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_U16_AUTOBREAK);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (motor_break_cntdown>>(i*8))&0xff;
		}
 	  break;
	  case TWI_REG_TX(TWI_REG_S16_PID_K_PID):
	    for(i=0;i<2;i++)
		{
	      u8TWITmp[(u8TWITmpIdx+0+i)&15] = (k_p>>(i*8))&0xff;
		}
	    for(i=0;i<2;i++)
		{
	      u8TWITmp[(u8TWITmpIdx+2+i)&15] = (k_i>>(i*8))&0xff;
		}
	    for(i=0;i<2;i++)
		{
	      u8TWITmp[(u8TWITmpIdx+4+i)&15] = (k_d>>(i*8))&0xff;
		}
 	  break;
// Motor A
	  case TWI_REG_TX(TWI_REG_S16_MA_PID_SP):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_S16_MA_PID_SP);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (motor_A.pid_setPoint>>(i*8))&0xff;
		}
 	  break;
	  case TWI_REG_TX(TWI_REG_S16_MA_PID_PV):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_S16_MA_PID_PV);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (motor_A.pid_processValue>>(i*8))&0xff;
		}
 	  break;
	  case TWI_REG_TX(TWI_REG_S16_MA_PID_OV):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_S16_MA_PID_OV);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (motor_A.pid_Value>>(i*8))&0xff;
		}
 	  break;
	  case TWI_REG_TX(TWI_REG_U8_MA_DIR):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_U8_MA_DIR);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (motor_A.i8_encoder_step>>(i*8))&0xff;
		}
 	  break;
	  case TWI_REG_TX(TWI_REG_U8_MA_PWM):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_U8_MA_PWM);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (motor_A.pwm>>(i*8))&0xff;
		}
 	  break;
	  case TWI_REG_TX(TWI_REG_S64_MA_ENC):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_S64_MA_ENC);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (motor_A.i64_encoder>>(i*8))&0xff;
		}
 	  break;
	  case TWI_REG_TX(TWI_REG_S16_MA_ENC):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_S16_MA_ENC);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (motor_A.i16_encoder>>(i*8))&0xff;
		}
		motor_A.i16_encoder = 0;
 	  break;
// Motor B
	  case TWI_REG_TX(TWI_REG_S16_MB_PID_SP):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_S16_MB_PID_SP);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (motor_B.pid_setPoint>>(i*8))&0xff;
		}
 	  break;
	  case TWI_REG_TX(TWI_REG_S16_MB_PID_PV):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_S16_MB_PID_PV);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (motor_B.pid_processValue>>(i*8))&0xff;
		}
 	  break;
	  case TWI_REG_TX(TWI_REG_S16_MB_PID_OV):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_S16_MB_PID_OV);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (motor_B.pid_Value>>(i*8))&0xff;
		}
 	  break;
	  case TWI_REG_TX(TWI_REG_U8_MB_DIR):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_U8_MB_DIR);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (motor_B.i8_encoder_step>>(i*8))&0xff;
		}
 	  break;
	  case TWI_REG_TX(TWI_REG_U8_MB_PWM):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_U8_MB_PWM);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (motor_B.pwm>>(i*8))&0xff;
		}
 	  break;
	  case TWI_REG_TX(TWI_REG_S64_MB_ENC):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_S64_MB_ENC);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (motor_B.i64_encoder>>(i*8))&0xff;
		}
 	  break;
	  case TWI_REG_TX(TWI_REG_S16_MB_ENC):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_S16_MB_ENC);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (motor_B.i16_encoder>>(i*8))&0xff;
		}
		motor_B.i16_encoder = 0;
 	  break;
      default:
	     break;
  }

  v = u8TWITmp[u8TWITmpIdx&15];
  u8TWITmp[u8TWITmpIdx&15] = 0;
  u8TWITmpIdx++;
  DEBUG_INT(12,0);
  return v;
}

/*
 * main
 */
int main(void)
{
    cli();
#if 0
    while(1)
    {
#ifdef BIT_PWMA
        REG(DDR,PORT_PWMA) |= (1<<BIT_PWMA);
        REG(PORT,PORT_PWMA) |= (1<<BIT_PWMA);
        _delay_ms(1);
        REG(PORT,PORT_PWMA) &= ~(1<<BIT_PWMA);  
        REG(DDR,PORT_PWMA) &= ~(1<<BIT_PWMA);
#endif      
#ifdef BIT_ENCA
        REG(DDR,PORT_ENCA) |= (1<<BIT_ENCA);
        REG(PORT,PORT_ENCA) |= (1<<BIT_ENCA);
        _delay_ms(2);
        REG(PORT,PORT_ENCA) &= ~(1<<BIT_ENCA);  
        REG(DDR,PORT_ENCA) &= ~(1<<BIT_ENCA);
#endif      
#ifdef BIT_MA_A
        REG(DDR,PORT_MA_A) |= (1<<BIT_MA_A);
        REG(PORT,PORT_MA_A) |= (1<<BIT_MA_A);
        _delay_ms(3);
        REG(PORT,PORT_MA_A) &= ~(1<<BIT_MA_A);  
        REG(DDR,PORT_MA_A) &= ~(1<<BIT_MA_A);
#endif      
#ifdef BIT_MA_B
        REG(DDR,PORT_MA_B) |= (1<<BIT_MA_B);
        REG(PORT,PORT_MA_B) |= (1<<BIT_MA_B);
        _delay_ms(4);
        REG(PORT,PORT_MA_B) &= ~(1<<BIT_MA_B);  
        REG(DDR,PORT_MA_B) &= ~(1<<BIT_MA_B);
#endif
      
#ifdef BIT_PWMB
        REG(DDR,PORT_PWMB) |= (1<<BIT_PWMB);
        REG(PORT,PORT_PWMB) |= (1<<BIT_PWMB);
        _delay_ms(5);
        REG(PORT,PORT_PWMB) &= ~(1<<BIT_PWMB);  
        REG(DDR,PORT_PWMB) &= ~(1<<BIT_PWMB);
#endif      
#ifdef BIT_ENCB
        REG(DDR,PORT_ENCB) |= (1<<BIT_ENCB);
        REG(PORT,PORT_ENCB) |= (1<<BIT_ENCB);
        _delay_ms(6);
        REG(PORT,PORT_ENCB) &= ~(1<<BIT_ENCB);  
        REG(DDR,PORT_ENCB) &= ~(1<<BIT_ENCB);
#endif      
#ifdef BIT_MB_A
        REG(DDR,PORT_MB_A) |= (1<<BIT_MB_A);
        REG(PORT,PORT_MB_A) |= (1<<BIT_MB_A);
        _delay_ms(7);
        REG(PORT,PORT_MB_A) &= ~(1<<BIT_MB_A);  
        REG(DDR,PORT_MB_A) &= ~(1<<BIT_MB_A);
#endif      
#ifdef BIT_MB_B
        REG(DDR,PORT_MB_B) |= (1<<BIT_MB_B);
        REG(PORT,PORT_MB_B) |= (1<<BIT_MB_B);
        _delay_ms(8);
        REG(PORT,PORT_MB_B) &= ~(1<<BIT_MB_B);  
        REG(DDR,PORT_MB_B) &= ~(1<<BIT_MB_B);
#endif      
        _delay_ms(100);
    }
#endif

	boot_counter = eeprom_read_word(&eeFooByteArray1[0]) + 1; 
	eeprom_write_word(&eeFooByteArray1[0], boot_counter);

    DDRA &= ~(1<<PIN_INT);
    PORTA |= (1<<PIN_INT);

#ifdef DEBUGUART
	stdout = &mystdout;
#endif
#ifdef BIT_PWMA
    REG(DDR,PORT_PWMA) |= (1<<BIT_PWMA);
    MODE_A(0);
    OCR0A = 0;
#endif
#ifdef BIT_PWMB
    REG(DDR,PORT_PWMB) |= (1<<BIT_PWMB);
    MODE_B(0);
    OCR0B = 0;
#endif
#ifdef BIT_ENCA
    REG(DDR,PORT_ENCA) &= ~(1<<BIT_ENCA);
    REG(PORT,PORT_ENCA) |= (1<<BIT_ENCA); // enable pullup
    PCMSK0 |= (1<<BIT_ENCA);
#endif
#ifdef BIT_ENCB
    REG(DDR,PORT_ENCB) &= ~(1<<BIT_ENCB);
    REG(PORT,PORT_ENCB) |= (1<<BIT_ENCB); // enable pullup
    PCMSK0 |= (1<<BIT_ENCB);
#endif
  GIMSK  = (1<<PCIE0);
  GIFR |= (1<<PCIF0);

#ifdef BIT_MA_EN
  REG(DDR,PORT_MA_EN) &= ~(1<<BIT_MA_EN);
  REG(PORT,PORT_MA_EN) |= (1<<BIT_MA_EN);
#endif
    
#if 0
  MODE_A(1);
  MODE_B(1);    
  PORTA |= (1<<PIN_PWMA);
  PORTB |= (1<<PIN_PWMB);
  while(1);
#endif

  // fast PWM mode
  TCCR0A = (1 << COM0A1) | (0 << COM0A0) | (1 << COM0B1) | (0 << COM0B0) | (0 << WGM01) | (1 << WGM00);
  //TCCR0B = (0 << WGM02) | (1 << CS02) | (0 << CS01) | (0 << CS00);   // clock source = CLK/1, start PWM
  TCCR0B = (0 << WGM02) | (0 << CS02) | (0 << CS01) | (1 << CS00);   // clock source = CLK/1, start PWM

  TCCR1A = 0;
  TCCR1B = (0 << CS12) | (0 << CS11) | (1 << CS10);   // clock source = CLK/8, start PWM
  TIMSK1 |= (1<<TOIE1);

  pid_Init(k_p,k_i,k_d, &(motor_A.pid));
  pid_Init(k_p,k_i,k_d, &(motor_B.pid));

#ifndef DEBUGUART
  i2c_init();
#endif

#if 0
  INTOUT(1);
  for(;;)
    {
        INTOUT(0);
        _delay_ms(1);
        INTOUT(1);
        _delay_ms(1);
    }
#endif

  sei();

#if 0
	motor_mode = MODE_TEST_01;
#endif

#if 0
	motor_test_01[0].speed_A = 300;
	motor_test_01[0].speed_B = -300;
	motor_test_01[0].delay_ms = 10000;
	motor_test_01[1].speed_A = 0;
	motor_test_01[1].speed_B = 0;
	motor_test_01[1].delay_ms = 1000;
	motor_test_01[2].speed_A = -300;
	motor_test_01[2].speed_B = 300;
	motor_test_01[2].delay_ms = 10000;
	motor_test_01[3].speed_A = 0;
	motor_test_01[3].speed_B = 0;
	motor_test_01[3].delay_ms = 1000;
	motor_test_01[4].speed_A = 1000;
	motor_test_01[4].speed_B = -1000; 
	motor_test_01[4].speed_B = -1000; 
	motor_test_01[4].delay_ms = 10000;
	motor_test_01[5].speed_A = 0;
	motor_test_01[5].speed_B = 0;
	motor_test_01[5].delay_ms = 1000;
	motor_test_01[6].speed_A = -1000;
	motor_test_01[6].speed_B = 1000;
	motor_test_01[6].delay_ms = 10000;
	motor_test_01[7].speed_A = 0;
	motor_test_01[7].speed_B = 0;
	motor_test_01[7].delay_ms = 1000;
#endif

#if 0
	motor_mode = MODE_PWM;
	setModeA(1,50);
#endif

#if 0
	motor_mode = MODE_PWM;
	setModeA(1,160);
	setModeB(2,160);
#endif

#if 0
	motor_mode = MODE_PID;
	motor_A.pid_setPoint = MOTOR_RPM(3);
	motor_B.pid_setPoint = MOTOR_RPM(-3);
#endif

	main_loop_int_pulse = 0;

#if 0
    while(1)
    {
        DDRA |= (1<<PIN_ENCB);
        PORTA |= (1<<PIN_ENCB);
        PORTA &= ~(1<<PIN_ENCB);
    }
#endif

	wdt_enable(WDTO_2S);   // Watchdog auf 1 s stellen

    for(;;)
    {            
			if( u8HandlePIDFlag == 1 )
			{
				u8HandlePIDFlag = 0;
				DEBUG_INT(7,1);
                if( motor_break_cntdown > 0 )
                {
                    motor_break_cntdown--;
                    if( motor_break_cntdown == 0 )
                    {
                        motor_mode = MODE_BREAK;
                    }
                }
				handlePID();
				DEBUG_INT(7,0);
			}

#if 0
			if( u8Tick == 1 )
			{
				u8Tick = 0;
				handle_motor_test_01();
			}
#endif

#ifdef DEBUGUART
			{
				static uint64_t _t = 0;
				if( u64_time_us > _t )
				{
					_t = u64_time_us + 1000000UL;
					printf("%ld",(long)motor_B.u32_encperiod);
				}
			}
#endif

            DEBUG_INT(1,1);
            DEBUG_INT(1,0);
			if( i2c_idle() != 0 )
			{
                DEBUG_INT(2,1);
                DEBUG_INT(2,0);
				if(i2c_wdt_reset == 0) wdt_reset();

			}
			
			//sleep ...
			if( i2c_idle() != 0 )
			{
				DEBUG_INT(10,1);
				set_sleep_mode(SLEEP_MODE_IDLE);
				sleep_mode();
				DEBUG_INT(10,0);
			}
			else
			{
				DEBUG_INT(14,1);
				DEBUG_INT(14,0);				
			}
    }

    return 0;  // the program executed successfully
}
/*
 * EOF
 */
