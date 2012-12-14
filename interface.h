/*
 * interface.h - types and prototypes used throughout the Interface MCU code
 *
 *  Created on: Dec 13, 2012
 *      Author: csdexter
 */

#ifndef INTERFACE_H_
#define INTERFACE_H_

#include <stdint.h>


#if defined(__AVR_ATtiny2313A__)
/* avr-libc is lame enough not to have a fully orthogonal vocabulary */
#define PCIE0 5
#define PCIE1 4
#define PCIE2 3
#define GIFR EIFR
#define PCIF0 5
#define PCIF1 4
#define PCIF2 3
#define PCMSK0 PCMSK
#endif

typedef union {
  uint8_t value;
  struct {
    uint8_t GlobalEnable:1;
    uint8_t EStopTrip:1;
    uint8_t EStopClear:1;
    uint8_t WatchDogTrip:1;
    uint8_t WatchDogClear:1;
    uint8_t AutoEStop:1;
    uint8_t AutoWatchDog:1;
    uint8_t Reserved:1;
  } flags;
} TInterruptFlags;

/* Sets given bit in InterruptCauses bitmap and asserts INT# line if given bit
 * is set in InterruptFlags and global interrupts are enabled */
void SetFlagAndAssertInterrupt(uint8_t flag);
/* Called whenever the Read Last Interrupt Cause command is issued */
void ClearFlagsAndReleaseInterrupt(void);


#endif /* INTERFACE_H_ */
