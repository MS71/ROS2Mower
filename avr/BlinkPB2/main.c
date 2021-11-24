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
#include <avr/wdt.h>

#include "i2c.h"

volatile uint64_t u64_time_us = 0;
volatile uint8_t u8Tick = 0;

void handlePID();
void TIM0_65536us();

static uint64_t get_time_us()
{
    volatile uint8_t tov0 = TIFR0 & 1;
    volatile uint16_t tcnt0 = (tov0 << 8) | TCNT0;
    return u64_time_us + tcnt0;
}

/*
 * ~64ms ISR
 */
void TIM0_65536us()
{
    u8Tick = 1;
}

/*
 * timer0 overflow
 */
volatile uint8_t tim0_divcnt = 0;
ISR(TIM0_OVF_vect)
{
    // sei();	// allow other IRQs e.g. i2c

    // Freg = F_CPU/(8*256) = 256us ~ 3906.25Hz
    u64_time_us += (1000000UL * 8 * 256) / F_CPU;
    tim0_divcnt++;

    if((tim0_divcnt) == 0)
    {
        TIM0_65536us();
    }
}

/*
 * i2c_TwiRxHandler
 */
void i2c_TwiRxHandler(uint16_t idx, uint8_t data)
{
}

/*
 * i2c_TwiTxHandler
 */
uint8_t i2c_TwiTxHandler(uint16_t idx)
{
    return 0;
}

/*
 * main
 */
int main(void)
{
    cli();
    MCUSR = 0;
    wdt_disable(); 

    DDRB |= (1 << PB2);
    PORTB &= ~(1 << PB2);

#if 0
    while(1)
    {
        PORTB &= ~(1 << PB2);
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        PORTB |= (1 << PB2);
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
        __asm volatile("nop");
    }
#endif

    // fast PWM mode
    TCCR0A = (0 << COM0A1) | (0 << COM0A0) | (0 << COM0B1) | (0 << COM0B0) | (1 << WGM01) | (1 << WGM00);

    // fast PWM mode
    TCCR0B = (0 << CS02) | (1 << CS01) | (0 << CS00); // clock source = CLK/8, start PWM

    // Overflow Interrupt erlauben
    TIMSK0 |= (1 << TOIE0);

    // i2c_init();

    sei();
    // wdt_enable(WDTO_1S);   // Watchdog auf 1 s stellen

    for(;;)
    {
        static uint64_t _t = 0;
        if(get_time_us() > _t)
        {
            if(PORTB & (1 << PB2))
            {
                PORTB &= ~(1 << PB2);
                _t = get_time_us() + 100000UL;
            }
            else
            {
                PORTB |= (1 << PB2);
                _t = get_time_us() + 100000UL;
            }
        }

        if(i2c_idle() != 0)
        {
            // wdt_reset();

            // sleep ...
            // set_sleep_mode(SLEEP_MODE_IDLE);
            // sleep_mode();
        }
    }
    return 0; // the program executed successfully
}
/*
 * EOF
 */
