/********************************************************************************
 I2C Slave
********************************************************************************/
/********************************************************************************
                            device dependent defines
********************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "i2c.h"

#if defined( __AVR_ATtiny167__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

#if defined( __AVR_ATtiny2313__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB5
#  define PORT_USI_SCL        PB7
#  define PIN_USI_SDA         PINB5
#  define PIN_USI_SCL         PINB7
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

#if defined(__AVR_ATtiny84__) | \
     defined(__AVR_ATtiny44__)
#  define DDR_USI             DDRA
#  define PORT_USI            PORTA
#  define PIN_USI             PINA
#  define PORT_USI_SDA        PORTA6
#  define PORT_USI_SCL        PORTA4
#  define PIN_USI_SDA         PINA6
#  define PIN_USI_SCL         PINA4
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATtiny25__ ) | \
     defined( __AVR_ATtiny45__ ) | \
     defined( __AVR_ATtiny85__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATtiny26__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_STRT_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATtiny261__ ) | \
      defined( __AVR_ATtiny461__ ) | \
      defined( __AVR_ATtiny861__ )
#  define DDR_USI             DDRB
#  define PORT_USI            PORTB
#  define PIN_USI             PINB
#  define PORT_USI_SDA        PB0
#  define PORT_USI_SCL        PB2
#  define PIN_USI_SDA         PINB0
#  define PIN_USI_SCL         PINB2
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVF_vect
#endif

#if defined( __AVR_ATmega165__ ) | \
     defined( __AVR_ATmega325__ ) | \
     defined( __AVR_ATmega3250__ ) | \
     defined( __AVR_ATmega645__ ) | \
     defined( __AVR_ATmega6450__ ) | \
     defined( __AVR_ATmega329__ ) | \
     defined( __AVR_ATmega3290__ )
#  define DDR_USI             DDRE
#  define PORT_USI            PORTE
#  define PIN_USI             PINE
#  define PORT_USI_SDA        PE5
#  define PORT_USI_SCL        PE4
#  define PIN_USI_SDA         PINE5
#  define PIN_USI_SCL         PINE4
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

#if defined( __AVR_ATmega169__ )
#  define DDR_USI             DDRE
#  define PORT_USI            PORTE
#  define PIN_USI             PINE
#  define PORT_USI_SDA        PE5
#  define PORT_USI_SCL        PE4
#  define PIN_USI_SDA         PINE5
#  define PIN_USI_SCL         PINE4
#  define USI_START_COND_INT  USISIF
#  define USI_START_VECTOR    USI_START_vect
#  define USI_OVERFLOW_VECTOR USI_OVERFLOW_vect
#endif

/********************************************************************************
 functions implemented as macros
********************************************************************************/

#define SET_USI_TO_SEND_ACK( ) \
{ \
  /* prepare ACK */ \
  USIDR = 0; \
  /* set SDA as output */ \
  DDR_USI |= ( 1 << PORT_USI_SDA ); \
  /* clear all interrupt flags, except Start Cond */ \
  USISR = \
       ( 0 << USI_START_COND_INT ) | \
       ( 1 << USIOIF ) | ( 1 << USIPF ) | \
       ( 1 << USIDC )| \
       /* set USI counter to shift 1 bit */ \
       ( 0x0E << USICNT0 ); \
}

#define SET_USI_TO_READ_ACK( ) \
{ \
  /* prepare ACK */ \
  USIDR = 0; \
  /* set SDA as input */ \
  DDR_USI &= ~( 1 << PORT_USI_SDA ); \
  /* clear all interrupt flags, except Start Cond */ \
  USISR = \
       ( 0 << USI_START_COND_INT ) | \
       ( 1 << USIOIF ) | \
       ( 1 << USIPF ) | \
       ( 1 << USIDC ) | \
       /* set USI counter to shift 1 bit */ \
       ( 0x0E << USICNT0 ); \
}

#define SET_USI_TO_TWI_START_CONDITION_MODE( ) \
{ \
  USICR = \
       /* enable Start Condition Interrupt, disable Overflow Interrupt */ \
       ( 1 << USISIE ) | ( 0 << USIOIE ) | \
       /* set USI in Two-wire mode, no USI Counter overflow hold */ \
       ( 1 << USIWM1 ) | ( 0 << USIWM0 ) | \
       /* Shift Register Clock Source = External, positive edge */ \
       /* 4-Bit Counter Source = external, both edges */ \
       ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) | \
       /* no toggle clock-port pin */ \
       ( 0 << USITC ); \
  USISR = \
        /* clear all interrupt flags, except Start Cond */ \
        ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | \
        ( 1 << USIDC ) | ( 0x0 << USICNT0 ); \
}

#define SET_USI_TO_SEND_DATA( ) \
{ \
  /* set SDA as output */ \
  DDR_USI |=  ( 1 << PORT_USI_SDA ); \
  /* clear all interrupt flags, except Start Cond */ \
  USISR    =  \
       ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | \
       ( 1 << USIDC) | \
       /* set USI to shift out 8 bits */ \
       ( 0x0 << USICNT0 ); \
}

#define SET_USI_TO_READ_DATA( ) \
{ \
  /* set SDA as input */ \
  DDR_USI &= ~( 1 << PORT_USI_SDA ); \
  /* clear all interrupt flags, except Start Cond */ \
  USISR    = \
       ( 0 << USI_START_COND_INT ) | ( 1 << USIOIF ) | \
       ( 1 << USIPF ) | ( 1 << USIDC ) | \
       /* set USI to shift out 8 bits */ \
       ( 0x0 << USICNT0 ); \
}

/********************************************************************************
 typedef's
********************************************************************************/
typedef enum
{
  USI_SLAVE_CHECK_ADDRESS                = 0x00,
  USI_SLAVE_SEND_DATA                    = 0x01,
  USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA = 0x02,
  USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA   = 0x03,
  USI_SLAVE_REQUEST_DATA                 = 0x04,
  USI_SLAVE_GET_DATA_AND_SEND_ACK        = 0x05
} overflowState_t;

/********************************************************************************
 local variables
********************************************************************************/

static volatile overflowState_t overflowState;
static volatile uint16_t u8TWIByteIdx = 0;

/********************************************************************************
                                public functions
********************************************************************************/
// initialise USI for TWI slave mode

void usiTwiRxHandler( uint16_t idx, uint8_t data );
uint8_t usiTwiTxHandler( uint16_t idx );

/********************************************************************************
 USI Start Condition ISR
********************************************************************************/
ISR( USI_START_VECTOR )
{
  // This triggers on second write, but claims to the callback there is only *one* byte in buffer

  // set default starting conditions for new TWI package
  overflowState = USI_SLAVE_CHECK_ADDRESS;

  //unsigned char tmpUSISR;                                         // Temporary variable to store volatile
  //tmpUSISR = USISR;                                               // Not necessary, but prevents warnings
  //(void)tmpUSISR;
    
  // set SDA as input
  PORT_USI |= ( 1 << PORT_USI_SDA );  // Set SDA high
  DDR_USI &= ~( 1 << PORT_USI_SDA );

  // wait for SCL to go low to ensure the Start Condition has completed (the
  // start detector will hold SCL low ) - if a Stop Condition arises then leave
  // the interrupt to prevent waiting forever - don't use USISR to test for Stop
  // Condition as in Application Note AVR312 because the Stop Condition Flag is
  // going to be set from the last TWI sequence
  while (
       // SCL his high
       ( PIN_USI & ( 1 << PIN_USI_SCL ) ) &&
       // and SDA is low
       !( ( PIN_USI & ( 1 << PIN_USI_SDA ) ) )
  );

  if ( !( PIN_USI & ( 1 << PIN_USI_SDA ) ) )
  {
    // a Stop Condition did not occur

    USICR =
         // keep Start Condition Interrupt enabled to detect RESTART
         ( 1 << USISIE ) |
         // enable Overflow Interrupt
         ( 1 << USIOIE ) |
         // set USI in Two-wire mode, hold SCL low on USI Counter overflow
         ( 1 << USIWM1 ) | ( 1 << USIWM0 ) |
         // Shift Register Clock Source = External, positive edge
         // 4-Bit Counter Source = external, both edges
         ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
         // no toggle clock-port pin
         ( 0 << USITC );

  }
  else
  {
    // a Stop Condition did occur

    USICR =
         // enable Start Condition Interrupt
         ( 1 << USISIE ) |
         // disable Overflow Interrupt
         ( 0 << USIOIE ) |
         // set USI in Two-wire mode, no USI Counter overflow hold
         ( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
         // Shift Register Clock Source = external, positive edge
         // 4-Bit Counter Source = external, both edges
         ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
         // no toggle clock-port pin
         ( 0 << USITC );

  } // end if

  USISR =
       // clear interrupt flags - resetting the Start Condition Flag will
       // release SCL
       ( 1 << USI_START_COND_INT ) | ( 1 << USIOIF ) |
       ( 1 << USIPF ) |( 1 << USIDC ) |
       // set USI to sample 8 bits (count 16 external SCL pin toggles)
       ( 0x0 << USICNT0);
} // end ISR( USI_START_VECTOR )



/********************************************************************************
 USI Overflow ISR
 Handles all the communication.
 Only disabled when waiting for a new Start Condition.
********************************************************************************/
ISR( USI_OVERFLOW_VECTOR )
{
  switch ( overflowState )
  {

    // Address mode: check address and send ACK (and next USI_SLAVE_SEND_DATA) if OK,
    // else reset USI
    case USI_SLAVE_CHECK_ADDRESS:
      u8TWIByteIdx=0;
      if ( ( USIDR == 0 ) || ( ( USIDR >> 1 ) == TWI_ADDR) )
      {
        if ( USIDR & 0x01 )
        {
          overflowState = USI_SLAVE_SEND_DATA;
        }
        else
        {
          overflowState = USI_SLAVE_REQUEST_DATA;
        } // end if
        
        //PORT_USI &= ~( 1 << PORT_USI_SDA );  // Set SDA high
        SET_USI_TO_SEND_ACK( );
      }
      else
      {
        SET_USI_TO_TWI_START_CONDITION_MODE( );
      }
      break;

    // Master write data mode: check reply and goto USI_SLAVE_SEND_DATA if OK,
    // else reset USI
    case USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA:
      if ( USIDR )
      {
        // if NACK, the master does not want more data
        SET_USI_TO_TWI_START_CONDITION_MODE( );
        return;
      }
      // from here we just drop straight into USI_SLAVE_SEND_DATA if the
      // master sent an ACK

    // copy data from buffer to USIDR and set USI to shift byte
    // next USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA
    case USI_SLAVE_SEND_DATA:
      USIDR = i2c_TwiTxHandler( u8TWIByteIdx++ );
      overflowState = USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA;
      SET_USI_TO_SEND_DATA( );
      break;

    // set USI to sample reply from master
    // next USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA
    case USI_SLAVE_REQUEST_REPLY_FROM_SEND_DATA:
      overflowState = USI_SLAVE_CHECK_REPLY_FROM_SEND_DATA;
      SET_USI_TO_READ_ACK( );
      break;

    // Master read data mode: set USI to sample data from master, next
    // USI_SLAVE_GET_DATA_AND_SEND_ACK
    case USI_SLAVE_REQUEST_DATA:
      overflowState = USI_SLAVE_GET_DATA_AND_SEND_ACK;
      SET_USI_TO_READ_DATA( );
      break;

    // copy data from USIDR and send ACK
    // next USI_SLAVE_REQUEST_DATA
    case USI_SLAVE_GET_DATA_AND_SEND_ACK:
      i2c_TwiRxHandler( u8TWIByteIdx++, USIDR);
      // next USI_SLAVE_REQUEST_DATA
      overflowState = USI_SLAVE_REQUEST_DATA;
      SET_USI_TO_SEND_ACK( );
      break;

  } // end switch
} // end ISR( USI_OVERFLOW_VECTOR )

/********************************************************************************
 Setup
********************************************************************************/
// the setup function runs once when you press reset or power the board
void i2c_init() {
  overflowState = USI_SLAVE_CHECK_ADDRESS;

  // In Two Wire mode (USIWM1, USIWM0 = 1X), the slave USI will pull SCL
  // low when a start condition is detected or a counter overflow (only
  // for USIWM1, USIWM0 = 11).  This inserts a wait state.  SCL is released
  // by the ISRs (USI_START_vect and USI_OVERFLOW_vect).

  // Set SCL and SDA as output
  DDR_USI |= ( 1 << PORT_USI_SCL ) | ( 1 << PORT_USI_SDA );

  // set SCL high
  PORT_USI |= ( 1 << PORT_USI_SCL );

  // set SDA high
  PORT_USI |= ( 1 << PORT_USI_SDA );

  // Set SDA as input
  DDR_USI &= ~( 1 << PORT_USI_SDA );

  USICR =
       // enable Start Condition Interrupt
       ( 1 << USISIE ) |
       // disable Overflow Interrupt
       ( 0 << USIOIE ) |
       // set USI in Two-wire mode, no USI Counter overflow hold
       ( 1 << USIWM1 ) | ( 0 << USIWM0 ) |
       // Shift Register Clock Source = external, positive edge
       // 4-Bit Counter Source = external, both edges
       ( 1 << USICS1 ) | ( 0 << USICS0 ) | ( 0 << USICLK ) |
       // no toggle clock-port pin
       ( 0 << USITC );

  // clear all interrupt flags and reset overflow counter

  USISR = ( 1 << USI_START_COND_INT ) | ( 1 << USIOIF ) | ( 1 << USIPF ) | ( 1 << USIDC );
}

uint8_t i2c_idle()
{
  //return (((PIN_USI & ( 1 << PORT_USI_SCL ))!=0)&&((PIN_USI & ( 1 << PORT_USI_SDA ))!=0))?1:0;
  return (overflowState == USI_SLAVE_CHECK_ADDRESS)?1:0;
}

/********************************************************************************
 EOF
********************************************************************************/
