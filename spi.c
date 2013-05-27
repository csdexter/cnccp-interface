/*
 * spi.c - SPI HAL for AVR
 *
 *  Created on: Nov 18, 2012
 *      Author: csdexter
 */

#include <stdbool.h>
#include <stdint.h>

#include <avr/interrupt.h>

#include "spi.h"


/* private variables */
static volatile uint8_t *SPI_Buf;
static volatile uint8_t SPI_Count;

/* public API */
void spi_start(volatile uint8_t *buf) {
  SPI_Count = 0;
  SPI_Buf = buf;

  spi_write(SPI_Buf[SPI_Count]);
}

ISR(SPI_INTERRUPT_NAME) {
  SPI_Buf[SPI_Count] = spi_read(); /* Perform the read earliest to clear receive complete status */
  spi_int_ack();
#if PROTOCOL_TRANSACTION_SIZE > 1
  SPI_Count = (SPI_Count < PROTOCOL_TRANSACTION_SIZE - 1) ? SPI_Count + 1 : 0;
#endif
  spi_write(SPI_Buf[SPI_Count]);
}
