/*
 * interface.h - types and prototypes used throughout the Interface MCU code
 *
 *  Created on: Dec 13, 2012
 *      Author: csdexter
 */

#ifndef INTERFACE_H_
#define INTERFACE_H_

#include <stdbool.h>
#include <stdint.h>

/* NOTE: no attempt has been made at making the PORT/PIN/DDR targets more
 * portable or easily configurable by defining symbolic constants for our
 * functions that would map to them in turn. This is by design: this is a low
 * level interface and we are aiming for the most compact code. As a result,
 * we do make assumptions about where our GPIOs are.
 */
#define INTERFACE_INTERRUPT_ESTOPTRIP 6
#define INTERFACE_INTERRUPT_ESTOPCLEAR 5
#define INTERFACE_INTERRUPT_WDOGTRIP 4
#define INTERFACE_INTERRUPT_WDOGCLEAR 3
#define INTERFACE_COMMAND_INTERRUPT 0x01
#define INTERFACE_COMMAND_OUTPUT 0x02
#define INTERFACE_COMMAND_CROSSBAR 0x03
#define INTERFACE_COMMAND_SPINDLE 0x04
#define INTERFACE_LED_OFF 0x00
#define INTERFACE_LED_ON 0x01
#define INTERFACE_LED_1HZ 0x02
#define INTERFACE_LED_4HZ 0x03

#define INTERFACE_TIMER_DIVISOR 8
#define OCR_12500Hz(divisor) (F_CPU / (divisor) / (12500 * 2) - 1)
#define OCR_32768Hz(divisor) (F_CPU / (divisor) / 32768 - 1)
#define INTERFACE_OCR_12KHZ OCR_12500Hz(INTERFACE_TIMER_DIVISOR)
#define INTERFACE_OCR_32KHZ OCR_32768Hz(INTERFACE_TIMER_DIVISOR)

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
/* Called whenever the output status changes */
void UpdateOutputs(TOutputStatus newOutputs);
/* Called whenever the crossbar switch's state changes */
void UpdateSwitch(TCrossbarStatus newState);
/* Called whenever the SPINDLE_PWM value changes */
void UpdateSpindlePWM(uint8_t newSpindlePWM);
/* Called whenever the CPUMP status changes */
void SetChargePump(bool mode);
/* Runs things at prescribed intervals */
void RunPeriodicTasks(void);

#endif /* INTERFACE_H_ */
