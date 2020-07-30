//#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "config.h"

#include "i2c.h"

extern uint8_t u8TWIMem[];
uint8_t pm_shdwn = 0;
uint8_t pm_pwrup = 0;

uint32_t pm_rtc = 0;

int32_t ubat_mv = UBAT_ON;
float ibat_ma = 0;
float isol_ma = 0;
float ich_ma = 0;
float iload_ma = 0;

// Watchdog Interrupt Service / is executed when watchdog timed out

ISR(WDT_vect) 
{
    uint16_t v;      

    if(digitalRead(PIN_ON)!=0)
    {
      // on
      v = (u8TWIMem[TWI_MEM_SHDWNCNT+0] | (u8TWIMem[TWI_MEM_SHDWNCNT+1]<<8));
      if( v > 0 )
      {
        if( ubat_mv < ((u8TWIMem[TWI_MEM_STAYONUBat+1]<<8) | u8TWIMem[TWI_MEM_STAYONUBat+0]) )
        {
          v--;
        }

        if( v == 0 )
        {
          pm_shdwn = 1;
        }
        u8TWIMem[TWI_MEM_SHDWNCNT+0] = (v>>0)&0xff; 
        u8TWIMem[TWI_MEM_SHDWNCNT+1] = (v>>8)&0xff;
      }
    }

    {
      v = (u8TWIMem[TWI_MEM_WDT+0] | (u8TWIMem[TWI_MEM_WDT+1]<<8));
      if( v > 0 )
      {
        if( v == 0 )
        {
          /* reset */
          MCUSR |= (1<<WDRF);
        }
        u8TWIMem[TWI_MEM_WDT+0] = (v>>0)&0xff; 
        u8TWIMem[TWI_MEM_WDT+1] = (v>>8)&0xff;
      }
    }

    if(digitalRead(PIN_ON)==0)
    {
      // off
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
    }    

    pm_rtc++;

    v = (u8TWIMem[TWI_MEM_RTC+0] | (u8TWIMem[TWI_MEM_RTC+1]<<8) | (u8TWIMem[TWI_MEM_RTC+2]<<16) | (u8TWIMem[TWI_MEM_RTC+3]<<24));
    v++;
    u8TWIMem[TWI_MEM_RTC+0] = (pm_rtc>>0)&0xff;
    u8TWIMem[TWI_MEM_RTC+1] = (pm_rtc>>8)&0xff;
    u8TWIMem[TWI_MEM_RTC+2] = (pm_rtc>>16)&0xff;
    u8TWIMem[TWI_MEM_RTC+3] = (pm_rtc>>24)&0xff;
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
  u8TWIMem[TWI_MEM_PWRUPCNT+0] = (DEF_POWERUPDELAY_INIT>>0)&0xff;
  u8TWIMem[TWI_MEM_PWRUPCNT+1] = (DEF_POWERUPDELAY_INIT>>8)&0xff;

  u8TWIMem[TWI_MEM_SHDWNREL+0] = (DEF_SHUTDOWNDELAY>>0)&0xff;
  u8TWIMem[TWI_MEM_SHDWNREL+1] = (DEF_SHUTDOWNDELAY>>8)&0xff;
  u8TWIMem[TWI_MEM_PWRUPREL+0] = (DEF_POWERUPDELAY>>0)&0xff;
  u8TWIMem[TWI_MEM_PWRUPREL+1] = (DEF_POWERUPDELAY>>8)&0xff;

  u8TWIMem[TWI_MEM_STAYONUBat+0] = ((99999)>>0)&0xff;
  u8TWIMem[TWI_MEM_STAYONUBat+1] = (99999>>8)&0xff;

  u8TWIMem[TWI_MEM_WDT+0] = (0>>0)&0xff; 
  u8TWIMem[TWI_MEM_WDT+1] = (0>>8)&0xff;


  pinMode(PIN_ON,OUTPUT);
  digitalWrite(PIN_ON,LOW); // off 

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

int16_t analogReadDiff(uint8_t adcch)
{
  int32_t v = 0;
  ADMUX = ((INTERNAL1V1 & 0x03) << REFS0) | ((adcch & 0x3f) << MUX0); //select the channel and reference

    sbi(ADCSRA, ADPS0);
    sbi(ADCSRA, ADPS1);
    sbi(ADCSRA, ADPS2);
    cbi(ADCSRA, ADATE);

    DIDR0 = 0xff;

  if( adcch >= 8 ) 
    sbi(ADCSRB, BIN);
      else
    cbi(ADCSRB, BIN);
    
  //sbi(ADCSRB, ADLAR);

  sbi(ADCSRA, ADSC); //Start conversion
  while(ADCSRA & (1<<ADSC)); //Wait for conversion to complete.

  sbi(ADCSRA, ADSC); //Start conversion 
  while(ADCSRA & (1<<ADSC)); //Wait for conversion to complete.

  volatile uint8_t low = ADCL;
  volatile uint8_t high = ADCH;

  v = (high << 8) | low;
  
  v = v << 6;
  v = v / 64;

   return v;
}

void pm_loop()
{    
    uint32_t uref_mv = 1100;
    analogReference(INTERNAL1V1);

    /*
     * read the ADCs
     */
     {
      int16_t adc = analogReadDiff(PIN_IN_U1); // ubat
      adc = (int16_t)((float)adc * uref_mv * UMULTIPLYER / 1024);
      cli();
      u8TWIMem[TWI_MEM_U1+0] = (adc>>0)&0xff;
      u8TWIMem[TWI_MEM_U1+1] = (adc>>8)&0xff;
      sei();
      ubat_mv = ((15*ubat_mv + adc)/16);
      cli();
      u8TWIMem[TWI_MEM_Ubat+0] = (ubat_mv>>0)&0xff;
      u8TWIMem[TWI_MEM_Ubat+1] = (ubat_mv>>8)&0xff;
      sei();
     }
     {
      int16_t adc = analogReadDiff(PIN_IN_U2);
      adc = (int16_t)((float)adc * uref_mv * UMULTIPLYER / 1024);
      cli();
      u8TWIMem[TWI_MEM_U2+0] = (adc>>0)&0xff;
      u8TWIMem[TWI_MEM_U2+1] = (adc>>8)&0xff;
      sei();
     }
     {
      int16_t adc = analogReadDiff(PIN_IN_U3);
      adc = (int16_t)((float)adc * uref_mv * UMULTIPLYER / 1024);
      cli();
      u8TWIMem[TWI_MEM_U3+0] = (adc>>0)&0xff;
      u8TWIMem[TWI_MEM_U3+1] = (adc>>8)&0xff;
      sei();
     }
     {
      int16_t adc = analogReadDiff(PIN_IN_U4);
      adc = (int16_t)((float)adc * uref_mv * UMULTIPLYER / 1024);
      cli();
      u8TWIMem[TWI_MEM_U4+0] = (adc>>0)&0xff;
      u8TWIMem[TWI_MEM_U4+1] = (adc>>8)&0xff;
      sei();
     }
     {
      int16_t adc = analogReadDiff(ADC_ISOL_CH); /* ISolar  */
      adc += ADC_ISOL_OFFSET;
      //adc = (int16_t)((float)adc * uref_mv * UMULTIPLYER * ADC_ISOL_MULT / ( 1.0 * 512.0 * 0.15 ) );
      isol_ma = ((LP_N-1)*isol_ma + adc)/LP_N;
      cli();
      u8TWIMem[TWI_MEM_I1+0] = (((int16_t)isol_ma)>>0)&0xff;
      u8TWIMem[TWI_MEM_I1+1] = (((int16_t)isol_ma)>>8)&0xff;
      sei();
     }
     {
      int16_t adc = (analogReadDiff(ADC_ICHARGE_CH));
      adc += ADC_ICHARGE_OFFSET;
      //adc = (int16_t)((float)adc * uref_mv * UMULTIPLYER * ADC_ICHARGE_MULT / ( 1.0 * 512.0 * 0.15 ) );
      ich_ma = ((LP_N-1)*ich_ma + adc)/LP_N;
            
      cli();
      u8TWIMem[TWI_MEM_I2+0] = (((int16_t)ich_ma)>>0)&0xff;
      u8TWIMem[TWI_MEM_I2+1] = (((int16_t)ich_ma)>>8)&0xff;
      sei();
     }
     {
      int16_t adc = (analogReadDiff(ADC_IOUT_CH));
      adc += ADC_IOUT_OFFSET;
      //adc = (int16_t)((float)adc * uref_mv * UMULTIPLYER * ADC_IOUT_MULT / ( 1.0 * 512.0 * (0.15/3.0) ) );
      iload_ma = ((LP_N-1)*iload_ma + adc)/LP_N;
      cli();
      u8TWIMem[TWI_MEM_I3+0] = (((int16_t)iload_ma)>>0)&0xff;
      u8TWIMem[TWI_MEM_I3+1] = (((int16_t)iload_ma)>>8)&0xff;
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
      if( ubat_mv < UBAT_OFF)
      {
        pm_shdwn = 1;
      }
           
      if( pm_shdwn == 1 )
      {
        pm_shdwn = 0;
        pm_pwrup = 0;

        u8TWIMem[TWI_MEM_PWRUPCNT+0] = u8TWIMem[TWI_MEM_PWRUPREL+0];
        u8TWIMem[TWI_MEM_PWRUPCNT+1] = u8TWIMem[TWI_MEM_PWRUPREL+1];
        digitalWrite(PIN_ON,LOW); // switch off
      }
    }
    else
    {
      // off
      if(( pm_pwrup == 1 )&&( ubat_mv >= UBAT_ON))
      {
        pm_pwrup = 0;
        pm_shdwn = 0;

        u8TWIMem[TWI_MEM_SHDWNCNT+0] = u8TWIMem[TWI_MEM_SHDWNREL+0];
        u8TWIMem[TWI_MEM_SHDWNCNT+1] = u8TWIMem[TWI_MEM_SHDWNREL+1];
        digitalWrite(PIN_ON,HIGH); // switch on
      }
    }
  }
#endif  

  if( digitalRead(PIN_ON) == LOW )
  {
    if( i2c_active() == 0 )
    {
      system_sleep();
    }
  }
}
