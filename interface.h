/*
 * interface.h - types and prototypes used throughout the Interface MCU code
 *
 *  Created on: Dec 13, 2012
 *      Author: csdexter
 */

#ifndef INTERFACE_H_
#define INTERFACE_H_

#include <stdint.h>


#define INTERFACE_INTERRUPT_ESTOPTRIP 6
#define INTERFACE_INTERRUPT_ESTOPCLEAR 5
#define INTERFACE_INTERRUPT_WDOGTRIP 4
#define INTERFACE_INTERRUPT_WDOGCLEAR 3
#define INTERFACE_COMMAND_INTERRUPT 0x01
#define INTERFACE_COMMAND_OUTPUT 0x02
#define INTERFACE_COMMAND_CROSSBAR 0x03
#define INTERFACE_COMMAND_SPINDLE 0x04

#if defined(__AVR_ATtiny2313A__)
/* avr-libc is lame enough not to have a fully orthogonal vocabulary */
/* On closer inspection, it would appear iotn2313a.h was copy-pasted from
 * iotn2312.h without much regard to the datasheet. How lame!
 */
#define PCIE0 5
#define PCIE1 4
#define PCIE2 3
#define GIFR EIFR
#define PCIF0 5
#define PCIF1 4
#define PCIF2 3
#define PCMSK0 PCMSK
#define PCINT0_vect PCINT_B_vect
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

typedef union {
  uint8_t value;
  struct {
    uint8_t SpindleFollowsRPM:1;
    uint8_t LPTLED:2;
    uint8_t USBLED:2;
    uint8_t ChargePump:1;
    uint8_t Spindle:1;
    uint8_t Cool:1;
  } flags;
} TOutputStatus;

typedef union {
  uint8_t value;
  struct {
    uint8_t LPT:1;
    uint8_t USB:1;
    uint8_t Reserved:6;
  } flags;
} TCrossbarStatus;

/* Sets given bit in InterruptCauses bitmap and asserts INT# line if given bit
 * is set in InterruptFlags and global interrupts are enabled */
void SetFlagAndAssertInterrupt(uint8_t flag);
/* Called whenever the Read Last Interrupt Cause command is issued */
void ClearFlagsAndReleaseInterrupt(void);


#endif /* INTERFACE_H_ */
