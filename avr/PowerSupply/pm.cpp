#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "config.h"

#include "i2c.h"

extern uint8_t u8TWIMem[];
uint8_t pm_shdwn = 0;
uint8_t pm_pwrup = 0;

uint32_t pm_rtc = 0;

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) 
{
#if 1
    uint16_t v;      

    v = (u8TWIMem[TWI_MEM_SHDWNCNT+0] | (u8TWIMem[TWI_MEM_SHDWNCNT+1]<<8));
    if( v > 0 )
    {
      v--;
      if( v == 0 )
      {
        pm_shdwn = 1;
      }
      u8TWIMem[TWI_MEM_SHDWNCNT+0] = (v>>0)&0xff;
      u8TWIMem[TWI_MEM_SHDWNCNT+1] = (v>>8)&0xff;
    }

    v = (u8TWIMem[TWI_MEM_PWRUPCNT+0] | (u8TWIMem[TWI_MEM_PWRUPCNT+1]<<8));
    if( v > 0 )
    {
      v--;
      if( v == 0 )
      {
        pm_pwrup = 1;
      }
      u8TWIMem[TWI_MEM_PWRUPCNT+0] = (v>>0)&0xff;
      u8TWIMem[TWI_MEM_PWRUPCNT+1] = (v>>8)&0xff;
    }

    pm_rtc++;
#endif    
#if 1
    v = (u8TWIMem[TWI_MEM_RTC+0] | (u8TWIMem[TWI_MEM_RTC+1]<<8) | (u8TWIMem[TWI_MEM_RTC+2]<<16) | (u8TWIMem[TWI_MEM_RTC+3]<<24));
    v++;
    u8TWIMem[TWI_MEM_RTC+0] = (pm_rtc>>0)&0xff;
    u8TWIMem[TWI_MEM_RTC+1] = (pm_rtc>>8)&0xff;
    u8TWIMem[TWI_MEM_RTC+2] = (pm_rtc>>16)&0xff;
    u8TWIMem[TWI_MEM_RTC+3] = (pm_rtc>>24)&0xff;
#endif
}

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) 
{
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCSR = bb;
  WDTCSR |= _BV(WDIE);
}

// set system into the sleep state 
// system wakes up when wtchdog is timed out
void system_sleep() 
{
  cbi(ADCSRA,ADEN);  // switch Analog to Digitalconverter OFF

  if(digitalRead(PIN_ON)!=0)
  {
    set_sleep_mode(SLEEP_MODE_IDLE); // sleep mode is set here
  }
  else
  {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  }

  sleep_enable();
  sleep_mode();                        // System sleeps here

  sleep_disable();                     // System continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
}


#define INTERNAL2V56_NO_CAP (6)

void pm_init()
{
  u8TWIMem[TWI_MEM_SHDWNCNT+0] = (DEF_SHUTDOWNDELAY>>0)&0xff;
  u8TWIMem[TWI_MEM_SHDWNCNT+1] = (DEF_SHUTDOWNDELAY>>8)&0xff;
  u8TWIMem[TWI_MEM_PWRUPCNT+0] = (DEF_POWERUPDELAY>>0)&0xff;
  u8TWIMem[TWI_MEM_PWRUPCNT+1] = (DEF_POWERUPDELAY>>8)&0xff;
  
  pinMode(PIN_ON,OUTPUT);
  digitalWrite(PIN_ON,LOW);

  pinMode(PIN_IN_PMSW,INPUT);
  pinMode(PIN_IN_U1,INPUT);
  pinMode(PIN_IN_U2,INPUT);
  pinMode(PIN_IN_U3,INPUT);
  pinMode(PIN_IN_U4,INPUT);
   
  digitalWrite(PIN_FON,HIGH);  // let led blink
  pinMode(PIN_FON,INPUT);

  setup_watchdog(6); // approximately 1 seconds sleep 	
}

uint8_t pmsw()
{
  int v = analogRead(PIN_IN_PMSW);
  int p = 0;

  if( v < PMSW_A )
  {
    p = 1;
  }
  else if( v < PMSW_B )
  {
    p = 3;
  }
  else
  {
    p = 2; 
  }

  v = (v&0xfff) | p<<12;
  
  cli();
  u8TWIMem[TWI_MEM_PMSW+0] = (v>>0)&0xff;
  u8TWIMem[TWI_MEM_PMSW+1] = (v>>8)&0xff;
  sei();
  
  return p;
}

void pm_loop()
{    
    analogReference(INTERNAL2V56_NO_CAP);
#if 1    
    /*
     * read the ADCs
     */
     {
      uint32_t adc = analogRead(PIN_IN_U1);
      adc = (adc*2560)/1023;
      adc *= UMULTIPLYER;
      cli();
      u8TWIMem[TWI_MEM_U1+0] = (adc>>0)&0xff;
      u8TWIMem[TWI_MEM_U1+1] = (adc>>8)&0xff;
      sei();
     }
     {
      uint32_t adc = analogRead(PIN_IN_U2);
      adc = (adc*2560)/1023;
      adc *= UMULTIPLYER;
      cli();
      u8TWIMem[TWI_MEM_U2+0] = (adc>>0)&0xff;
      u8TWIMem[TWI_MEM_U2+1] = (adc>>8)&0xff;
      sei();
     }
     {
      uint32_t adc = analogRead(PIN_IN_U3);
      adc = (adc*2560)/1023;
      adc *= UMULTIPLYER;
      cli();
      u8TWIMem[TWI_MEM_U3+0] = (adc>>0)&0xff;
      u8TWIMem[TWI_MEM_U3+1] = (adc>>8)&0xff;
      sei();
     }
     {
      uint32_t adc = analogRead(ADC_CH_A0_A1_20x);
      adc = (adc*2560)/1023;
      adc *= UMULTIPLYER;
      cli();
      u8TWIMem[TWI_MEM_I1+0] = (adc>>0)&0xff;
      u8TWIMem[TWI_MEM_I1+1] = (adc>>8)&0xff;
      sei();
     }
     {
      uint32_t adc = analogRead(ADC_CH_A0_A3_20x);
      adc = (adc*2560)/1023;
      adc *= UMULTIPLYER;
      cli();
      u8TWIMem[TWI_MEM_I2+0] = (adc>>0)&0xff;
      u8TWIMem[TWI_MEM_I2+1] = (adc>>8)&0xff;
      sei();
     }
     {
      uint32_t adc = analogRead(ADC_CH_A1_A2_20x);
      adc = (adc*2560)/1023;
      adc *= UMULTIPLYER;
      cli();
      u8TWIMem[TWI_MEM_I3+0] = (adc>>0)&0xff;
      u8TWIMem[TWI_MEM_I3+1] = (adc>>8)&0xff;
      sei();
     }
     /*
      * increment loop counter
      */
     {
      uint32_t v = 0;      
      v = (v<<8) | u8TWIMem[TWI_MEM_LOOPCNT+3];
      v = (v<<8) | u8TWIMem[TWI_MEM_LOOPCNT+2];
      v = (v<<8) | u8TWIMem[TWI_MEM_LOOPCNT+1];
      v = (v<<8) | u8TWIMem[TWI_MEM_LOOPCNT+0];
      v++;
      cli();
      u8TWIMem[TWI_MEM_LOOPCNT+0] = (v>>0)&0xff;
      u8TWIMem[TWI_MEM_LOOPCNT+1] = (v>>8)&0xff;
      u8TWIMem[TWI_MEM_LOOPCNT+2] = (v>>16)&0xff;
      u8TWIMem[TWI_MEM_LOOPCNT+3] = (v>>24)&0xff;
      sei();
     }
    cli();
    u8TWIMem[TWI_MEM_RTC+0] = (pm_rtc>>0)&0xff;
    u8TWIMem[TWI_MEM_RTC+1] = (pm_rtc>>8)&0xff;
    u8TWIMem[TWI_MEM_RTC+1] = (pm_rtc>>16)&0xff;
    u8TWIMem[TWI_MEM_RTC+1] = (pm_rtc>>24)&0xff;
    sei();
#endif

  uint8_t p = pmsw();

#if 0 
  // allways on
  digitalWrite(PIN_ON,HIGH);  
#else
  if( p == 1 )
  {
    // OFF and sleep
    digitalWrite(PIN_ON,LOW);
    pm_pwrup = 1;
    u8TWIMem[TWI_MEM_SHDWNCNT+0] = (DEF_SHUTDOWNDELAY>>0)&0xff;
    u8TWIMem[TWI_MEM_SHDWNCNT+1] = (DEF_SHUTDOWNDELAY>>8)&0xff;
  }
  else if( p == 3 )
  {
    // allways on
    digitalWrite(PIN_ON,HIGH);  
    pm_pwrup = 1;
    u8TWIMem[TWI_MEM_SHDWNCNT+0] = (DEF_SHUTDOWNDELAY>>0)&0xff;
    u8TWIMem[TWI_MEM_SHDWNCNT+1] = (DEF_SHUTDOWNDELAY>>8)&0xff;
  }
  else
  {
    // auto
    if(digitalRead(PIN_ON)!=0)
    {
      // on
      if( pm_shdwn == 1 )
      {
        pm_shdwn = 0;
        pm_pwrup = 0;
        digitalWrite(PIN_ON,LOW);      
      }
    }
    else
    {
      // off
      if( pm_pwrup == 1 )
      {
        pm_pwrup = 0;
        pm_shdwn = 0;

        u8TWIMem[TWI_MEM_SHDWNCNT+0] = (DEF_SHUTDOWNDELAY>>0)&0xff;
        u8TWIMem[TWI_MEM_SHDWNCNT+1] = (DEF_SHUTDOWNDELAY>>8)&0xff;
        
        digitalWrite(PIN_ON,HIGH);        
      }
    }
  }
#endif  

#if 1
  if( digitalRead(PIN_ON) == LOW )
  {
    system_sleep();
  }
  else if( i2c_active() == 0 )
  {
    system_sleep();
  }
#endif
}
