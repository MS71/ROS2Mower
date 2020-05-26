/* ----------------------------------------------------------
                           uart_printf.h

     Header fuer Softwaremodul fuer serielle Schnittstelle
     und reduziertem PRINTF fuer AVR-MCU. Protokoll 8N1

     Unterstuetzte Controller:

        Controller mit Hardware-UART

        - ATmega8
        - ATmega8515
        - ATmega48 - ATmega328
        - ATtiny2313, ATtiny4313

        Controller mit USI-Serial

        - ATtiny24 - ATtiny84
        - ATtiny25 - ATtiny85


        Anschlussbelegungen:

                MCU            PORT    Funk.1   Funk.2
        ---------------------------------------------------
        ATtiny25 - ATtiny85    PB1      TxD       Do   (6)
                               PB0      RxD       Di   (5)
        ---------------------------------------------------
        ATtiny24 - ATtiny84    PA5      TxD       Do   (8)
                               PA6      RxD       Di   (7)
        ---------------------------------------------------
        ATtiny2313 / 4313      PD1      TxD            (3)
                               PD0      RxD            (2)
        ---------------------------------------------------
        28 pol. ATmega (8, 48, 88, 168, 328)
        ---------------------------------------------------
                               PD1      TxD            (3)
                               PD0      RxD            (2)
        ---------------------------------------------------


     16.2.2019     R. Seelig

   ---------------------------------------------------------- */

#ifndef in_uart_printf
  #define in_uart_printf

  #include <stdint.h>
  #include <stdlib.h>
  #include <stdbool.h>
  #include <stdarg.h>

  #include <util/delay.h>
  #include <avr/io.h>
  #include <avr/pgmspace.h>
  #include <avr/interrupt.h>


  #define echo_enable     0
  #define readint_enable  1

  extern char printfkomma;

  /* -----------------------------------------------------------------------
      Moegliche Baudratenkombinationen fuer ATtiny24 - 84, ATtiny25 - 85

      F_CPU 1000000   BAUDRATE 1200, 2400
      F_CPU 8000000   BAUDRATE 9600, 19200
      F_CPU 12000000  BAUDRATE 9600, 19200,
      F_CPU 16000000  BAUDRATE 9600, 19200, 38400

      Moegliche Baudraten fuer ATmega / ATtiny2313

      mit 16 MHz Quarz bis 115200
     ----------------------------------------------------------------------- */

  #define BAUDRATE       19200

  #define uart_crlf()     { uart_putchar(0x0d); uart_putchar(0x0a); }


  /* -----------------------------------------------------------------------
                                  Prototypen
     ----------------------------------------------------------------------- */
  // UART
  void uart_init(void);
  void uart_putchar(uint8_t ch);
  #define putchar    uart_putchar

  uint8_t uart_ischar( void );
  uint8_t uart_getchar( void );
  #define getch   uart_getchar

  #if (readint_enable == 1)

    int checkint16(char *p, int *myi);
    int16_t readint();

  #endif

  // PRINTF
  void putint(int16_t i, char komma);
  void hexnibbleout(uint8_t b);
  void puthex(uint16_t h);
  void my_putramstring(uint8_t *p);

  void own_printf(const uint8_t *s,...);

  #define tiny_printf(str,...)  (own_printf(PSTR(str), ## __VA_ARGS__))
  #define my_printf             tiny_printf
  #define printf                my_printf


#endif
