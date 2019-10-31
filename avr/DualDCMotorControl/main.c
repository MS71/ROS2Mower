#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "pid.h"
#include "i2c.h"

#define PIN_ENCA	PA1
#define PIN_ENCB	PA0
#define PIN_INA1	PA2
#define PIN_INA2	PA3
#define PIN_INB1	PB1
#define PIN_INB2	PB0
#define PIN_PWMA	PA7
#define PIN_PWMB	PB2

#define MODE_B(_m_) \
	DDRA |= (1<<PIN_INA1) | (1<<PIN_INA2); \
	PORTA = (PORTA & ~(1<<PIN_INA1)) | ((((_m_)>>0)&1)<<PIN_INA1); \
	PORTA = (PORTA & ~(1<<PIN_INA2)) | ((((_m_)>>1)&1)<<PIN_INA2);

#define MODE_A(_m_) \
	DDRB |= (1<<PIN_INB1) | (1<<PIN_INB2); \
	PORTB = (PORTB & ~(1<<PIN_INB1)) | ((((_m_)>>0)&1)<<PIN_INB1); \
	PORTB = (PORTB & ~(1<<PIN_INB2)) | ((((_m_)>>1)&1)<<PIN_INB2);

#define TOGGLE_PIN(_port_,_pin_) \
	_port_ = (_port_&(~(1<<(_pin_)))) | ((~(_port_))&(1<<(_pin_)))

int16_t k_p = (int16_t)((1.5)*SCALING_FACTOR);
int16_t k_i = (int16_t)((0.01)*SCALING_FACTOR);
int16_t k_d = (int16_t)((0.01)*SCALING_FACTOR);

typedef struct
{
	pidData_t pid;
	int16_t   pid_setPoint;
	int16_t   pid_processValue;
	int16_t   pid_Value;
	uint8_t   pwm;

	int16_t   i16_encoder;
	
	int64_t   i64_encoder;
	int8_t    i8_encoder_step;
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

#define MOTOR_TEST_01_STEPS 4
struct
{
	int16_t  speed_A;
	int16_t  speed_B;
	uint16_t delay_ms;
} motor_test_01[MOTOR_TEST_01_STEPS] = {};

volatile uint64_t u64_time_us = 0;
volatile uint8_t u8Tick = 0;

void TIM0_65536us();

/*
 * timer0 overflow
 */
volatile uint8_t tim0_divcnt = 0;
ISR (TIM0_OVF_vect)
{
  // Freg = F_CPU/(8*256) = 256us ~ 3906Hz
  u64_time_us += (1000000UL*8*256)/F_CPU;
  tim0_divcnt++;

  if( (tim0_divcnt&3) == 0 )
  {
  	if( motor_break_cntdown > 0 )
	{
		motor_break_cntdown--;
		if( motor_break_cntdown == 0 )
		{
			motor_mode = MODE_BREAK;
		}
	}
  }

  if( (tim0_divcnt) == 0 )
  {
	TIM0_65536us();
  }
  
}

/********************************************************************************
 Wheel Encoder
 max. 5kHz => 200us
********************************************************************************/
ISR(PCINT0_vect) 
{
	{
		static uint8_t _enca = 0;
		uint8_t enca = (PINA & (1<<PIN_ENCA));
		if( enca != _enca )
		{
			motor_A.i16_encoder++;
			motor_A.i64_encoder += motor_A.i8_encoder_step;
			_enca = enca;
		}
	}
	{
		static uint8_t _encb = 0;
		uint8_t encb = (PINA & (1<<PIN_ENCB));
		if( encb != _encb )
		{
			motor_B.i16_encoder++;
			motor_B.i64_encoder += motor_B.i8_encoder_step;
			_encb = encb;
		}
	}
}

static void setModeA(uint8_t mode,uint8_t pwm)
{
	  MODE_A(mode);
	  OCR0A = pwm;
}

static void setModeB(uint8_t mode,uint8_t pwm)
{
	  MODE_B(mode);
	  OCR0B = pwm;
}

void calcPID(Motor *m,void(*set_mode)(uint8_t mode,uint8_t pwm))
{
  m->pid_processValue = m->i16_encoder;
  m->i16_encoder = 0;
  if( m->pid_setPoint > 0 )
  {
	m->i8_encoder_step = 1;
	  
    m->pid_Value = pid_Controller(m->pid_setPoint, m->pid_processValue, &m->pid);
    m->pwm = ((((int32_t)m->pid_Value)*255)/SCALING_FACTOR);
    if( m->pwm > 0 )
    {
 	  set_mode(1,m->pwm);  
    }
    else
    {
 	  set_mode(3,0);
    }
  }
  else if( m->pid_setPoint < 0 )
  {
	m->i8_encoder_step = -1;

    m->pid_Value = pid_Controller(-m->pid_setPoint, m->pid_processValue, &m->pid);
    m->pwm = ((((int32_t)m->pid_Value)*255)/SCALING_FACTOR);
    if( m->pwm > 0 )
    {
 	  set_mode(2,m->pwm);
    }
    else
    {
 	  set_mode(3,0);
    }
  }
  else
  {
 	  set_mode(0,0);
  }
}

/*
 * ~64ms ISR
 */
void TIM0_65536us()
{
	u8Tick = 1;
	
	if( (motor_mode==MODE_PID) || 
		(motor_mode==MODE_TEST_01) )
	{
		calcPID(&motor_A,setModeA);
		calcPID(&motor_B,setModeB);
	}
	else if(motor_mode==MODE_OFF)
	{
		setModeA(0,0);
		setModeB(0,0);
	}
	else if(motor_mode==MODE_BREAK)
	{
		setModeA(0,0xff);
		setModeB(0,0xff);
	}
}

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
			if( u64_time_us >= _u64_time_us )
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
				_u64_time_us = u64_time_us + 
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

volatile uint8_t u8TWIReg = 0;
volatile uint8_t u8TWITmp[16] = {};
volatile uint8_t u8TWITmpIdx = 0;

#define TWI_REG_U64_CLOCK_US	((0<<8)|8)
#define TWI_REG_U16_MODE  	    ((8<<8)|2)
#define TWI_REG_U16_AUTOBREAK   ((10<<8)|2)

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
      default: 
	     break;
     }
  }
}

/*
 * i2c_TwiTxHandler
 */
uint8_t i2c_TwiTxHandler( uint16_t idx )
{
  uint8_t i = 0;
  uint8_t v = 0;
  switch(u8TWIReg+idx)
  {
	  case TWI_REG_TX(TWI_REG_U64_CLOCK_US):
	    for(i=0;i<TWI_REG_BYTES(TWI_REG_U64_CLOCK_US);i++)
		{
	      u8TWITmp[(u8TWITmpIdx+i)&15] = (u64_time_us>>(i*8))&0xff;
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
  return v;
}

/*
 * main
 */
int main(void)
{
	DDRA |= (1<<PIN_PWMA);
	DDRB |= (1<<PIN_PWMB);

	DDRA &= ~(1<<PIN_ENCA);
	DDRA &= ~(1<<PIN_ENCB);

	PORTA |= (1<<PIN_ENCA);
	PORTA |= (1<<PIN_ENCB);

    PCMSK0 |= (1<<PCINT0);
	PCMSK0 |= (1<<PCINT1);
	GIMSK  = (1<<PCIE0); 

	MODE_A(0);
	MODE_B(0);

    OCR0A = 0;
    OCR0B = 0;

    // fast PWM mode
    TCCR0A = (1 << COM0A1) | (0 << COM0A0) | 
			 (1 << COM0B1) | (0 << COM0B0) | 
			 (1 << WGM01) | (1 << WGM00);
    TCCR0B = (0 << CS02) | (1 << CS01) | (0 << CS00);   // clock source = CLK/8, start PWM

    // Overflow Interrupt erlauben
    TIMSK0 |= (1<<TOIE0);

    pid_Init(k_p,k_i,k_d, &(motor_A.pid));
    pid_Init(k_p,k_i,k_d, &(motor_B.pid));

    i2c_init();

    sei();

#if 0
	motor_mode = MODE_TEST_01_PWM;
	motor_test_01[0].speed_A = 150;
	motor_test_01[0].speed_B = 150;
	motor_test_01[0].delay_ms = 1000;
	motor_test_01[1].speed_A = -150;
	motor_test_01[1].speed_B = -150;
	motor_test_01[1].delay_ms = 1000;
#endif

    for(;;)
    {
		if( u8Tick == 1 )
		{
			u8Tick = 0;
			handle_motor_test_01();
		}
		
		// sleep ...
		set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();
    }

    return 0;  // the program executed successfully
}
/*
 * EOF
 */
