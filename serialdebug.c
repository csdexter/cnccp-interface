/*
 * serialdebug.c
 *
 *  Created on: Apr 8, 2013
 *      Author: csdexter
 */


#include <stdint.h>

#include <avr/io.h>
#include <avr/power.h>

#include "serialdebug.h"

#if defined(__AVR_ATmega328P__)
#define UDR UDR0
#define UCSRA UCSR0A
#define UCSRB UCSR0B
#define UBRRH UBRR0H
#define UBRRL UBRR0L
#define U2X U2X0
#define TXEN TXEN0
#define UDRIE UDRIE0
#define UDRE UDRE0
#define power_usart_enable power_usart0_enable
#elif defined(__AVR_ATtiny2313A__)
#define power_usart_enable() (PRR &= (uint8_t)~(1 << PRUSART))
#endif


void host_serialconsole_init(void) {
#define BAUD CONSOLE_BAUD_RATE
#include <util/setbaud.h>

  power_usart_enable();

  UBRRH = UBRRH_VALUE;
  UBRRL = UBRRL_VALUE;
#if USE_2X
  UCSRA |= _BV(U2X);
#else
  UCSRA &= ~_BV(U2X);
#endif
#undef BAUD
  UCSRB |= _BV(TXEN);
  UCSRC = _BV(UCSZ1) | _BV(UCSZ0);
}

void host_serialconsole_write(char c) {
  while(!(UCSRA & _BV(UDRE)))
    /* NOP */;
  UDR = c;
}

