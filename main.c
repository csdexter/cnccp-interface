/*
 * main.c - Boilerplate AVR main/startup code
 *
 *  Created on: Dec 13, 2012
 *      Author: csdexter
 *
 * This is the Interface MCU firmware, it receives commands from the main MCU
 * via SPI and controls the crossbar switch, the master LEDs; generates the
 * SPINDLE, COOL and SPINDLE_PWM signals and monitors E-Stop.
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include <avr/interrupt.h>
#include <avr/io.h>

#include "boilerplate.h"
#include "interface.h"
#include "protocol.h"
#include "spi.h"


volatile bool NewCommand;
volatile uint8_t SPIBuf[2];
volatile TInterruptFlags InterruptFlags, InterruptCauses;
volatile TCrossbarStatus Crossbar;
volatile uint8_t SpindlePWM;

TOutputStatus Outputs;

ISR(PCINT0_vect) {
  if(PINB & _BV(PORTB4)) {
    NewCommand = true;
    spi_enable(SPI_OFF);
  } else {
    spi_enable(SPI_ON);
    spi_start(&SPIBuf[0], &SPIBuf[1]);
  }
}

ISR(INT0_vect) {
  if(PIND & _BV(PORTD2)) SetFlagAndAssertInterrupt(_BV(INTERFACE_INTERRUPT_ESTOPCLEAR));
  else {
    SetFlagAndAssertInterrupt(_BV(INTERFACE_INTERRUPT_ESTOPTRIP));
    if(InterruptFlags.flags.AutoEStop) {
      /* FreezeTheWorld */
    }
  }
}

void SPI_Hook(bool when) {
  if(when == SPI_HOOK_BEFORE) SPIBuf[1] = SPIBuf[0];
  else spi_start(&SPIBuf[0], &SPIBuf[1]);
}

void SetFlagAndAssertInterrupt(uint8_t flag) {
  InterruptCauses.value |= flag;
  if(InterruptFlags.flags.GlobalEnable && (InterruptFlags.value & flag))
    PORTD &= ~_BV(PORTD3);
}

void ClearFlagsAndReleaseInterrupt(void) {
  InterruptCauses.value = 0;
  PORTD |= _BV(PORTD3);
}

void UpdateOutputs(TOutputStatus newOutputs) {
  PORTD = (PORTD & ~(_BV(PORTD4) | _BV(PORTD5))) | \
      _BV(newOutputs.flags.Spindle ? PORTD4 : 0) | \
      _BV(newOutputs.flags.Cool ? PORTD5 : 0);


  Outputs = newOutputs;
}

void UpdateSwitch(TCrossbarStatus newState) {
  ;
}

void UpdateSpindlePWM(uint8_t newSpindlePWM) {
  ;
}

void init(void) {
  /* Setup GPIO ports */
  /* *_LED on PD0/1, outputs
   * ESTOP# on PD2, input and external interrupt
   * INT#, SPINDLE, COOL on PD3/4/5, outputs
   * WDT_LPT on PD6, input and pin change interrupt
   */
  DDRD = (_BV(PORTD0) | _BV(PORTD1) | _BV(PORTD3) | _BV(PORTD4) | _BV(PORTD5));
  PORTD = _BV(PORTD3);
  MCUCR |= _BV(ISC00);
  PCMSK2 = _BV(PCINT17);
  /* MOSI, MISO and SCK configured by SPI */
  spi_configure(SPI_INT_ENABLE, SPI_OFF, NULL, SPI_SLAVE, NULL, SPI_PHASE_LEADING, NULL);
  spi_hook = SPI_Hook;
  /* SS# on PB4, input and pin change interrupt
   * *_PWM on PB3/2, outputs
   * CS_*# on PB1/0, outputs
   */
  DDRB = (_BV(PORTB3) | _BV(PORTB2) | _BV(PORTB1) | _BV(PORTB0));
  PORTB = (_BV(PORTB1) | _BV(PORTB0));
  PCMSK0 = _BV(PCINT4);
  GIFR |= _BV(INTF0) | _BV(PCIF2) | _BV(PCIF0); /* Avoid spurious interrupts on startup */
  GIMSK |= _BV(INT0) | _BV(PCIE2) | _BV(PCIE0);

  /* Initialize state */
  NewCommand = false;

  /* And off we go */
  sei();
}

int main(void) {
  init();

  while(true) {
    if(NewCommand) {
      NewCommand = false;
        switch(SPIBuf[0]) {
          case (INTERFACE_COMMAND_INTERRUPT | PROTOCOL_RCOMM):
            SPIBuf[1] = InterruptFlags.value;
            break;
          case (INTERFACE_COMMAND_INTERRUPT | PROTOCOL_RDATA):
            SPIBuf[1] = (InterruptCauses.flags.EStopTrip ? _BV(7) : 0) |
                (InterruptCauses.flags.WatchDogTrip ? _BV(6) : 0);
            ClearFlagsAndReleaseInterrupt();
            break;
          case (INTERFACE_COMMAND_INTERRUPT | PROTOCOL_WCOMM):
            InterruptFlags.value = SPIBuf[0];
            break;
          case (INTERFACE_COMMAND_OUTPUT | PROTOCOL_RCOMM):
            SPIBuf[1] = Outputs.value;
            break;
          case (INTERFACE_COMMAND_OUTPUT | PROTOCOL_WCOMM):
            if(Outputs.value != SPIBuf[0]) UpdateOutputs((TOutputStatus)SPIBuf[0]);
            break;
          case (INTERFACE_COMMAND_CROSSBAR | PROTOCOL_RCOMM):
            SPIBuf[1] = Crossbar.value;
            break;
          case (INTERFACE_COMMAND_CROSSBAR | PROTOCOL_WCOMM):
            if(Crossbar.value != SPIBuf[0]) UpdateSwitch((TCrossbarStatus)SPIBuf[0]);
            break;
          case (INTERFACE_COMMAND_SPINDLE | PROTOCOL_RDATA):
            SPIBuf[1] = SpindlePWM;
            break;
          case (INTERFACE_COMMAND_SPINDLE | PROTOCOL_WDATA):
            if(SpindlePWM != SPIBuf[0]) UpdateSpindlePWM(SPIBuf[0]);
            break;
        }
    }
  }

  return 0;
}
