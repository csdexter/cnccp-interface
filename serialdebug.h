/*
 * serialdebug.h - Debug output via built-in UART
 *
 *  Created on: Apr 8, 2013
 *      Author: csdexter
 */

#ifndef SERIALDEBUG_H_
#define SERIALDEBUG_H_

#include <stdint.h>


#ifdef DEBUG
#define D(x) host_serialconsole_write(x)
#else
#define D(x)
#endif

/* Host serial console baud rate. */
#define CONSOLE_BAUD_RATE 9600

/* Serial console interface */
void host_serialconsole_init();
/* Blocks until transmitter ready and then sends one character */
void host_serialconsole_write(char c);


#endif /* SERIALDEBUG_H_ */
