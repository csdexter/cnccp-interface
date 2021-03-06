/*
 * main.c - Boilerplate AVR main/startup code
 *
 * This is the Interface MCU firmware, it receives commands from the main MCU
 * via SPI and controls the crossbar switch, the master LEDs; generates the
 * SPINDLE, COOL and SPINDLE_PWM signals and monitors E-Stop.
 *
 *  Created on: Dec 13, 2012
 *      Author: csdexter
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>

#include "boilerplate.h"
#include "interface.h"
#include "protocol.h"
#include "serialdebug.h"
#include "spi.h"


volatile bool NewCommand;
volatile uint8_t SPIBuf[3];
volatile TInterruptFlags InterruptFlags, InterruptCauses;
volatile TCrossbarStatus Crossbar;
volatile uint8_t SpindlePWM;
volatile uint16_t OldWDTime[2];

uint16_t WallClock;
TOutputStatus Outputs;

/* SPI SS# */
ISR(PCINT0_vect) {
  if(PINB & _BV(PORTB4)) {
    //TODO: in the future, we may want to error out if we haven't received exactly three bytes
    NewCommand = true;
    spi_enable(SPI_OFF);
  } else {
    //TODO: in the future, we may want to error out if the last SPI transaction hasn't been processed yet
    spi_enable(SPI_ON);
    spi_start(SPIBuf);
  }
}

/* External WatchDog */
ISR(PCINT2_vect) {
  OldWDTime[0] = OldWDTime[1];
  OldWDTime[1] = WallClock;
}

/* E-Stop */
ISR(INT0_vect) {
  if(PIND & _BV(PORTD2)) {
    SetFlagAndAssertInterrupt(_BV(INTERFACE_INTERRUPT_ESTOPCLEAR));
    if(InterruptFlags.flags.AutoEStop) {
      Outputs.value = 0;
      Outputs.flags.ChargePump = true;
      Outputs.flags.LPTLED = INTERFACE_LED_OFF;
      Outputs.flags.USBLED = INTERFACE_LED_ON;
      UpdateOutputs(Outputs);
      Crossbar.flags.USB = true;
      Crossbar.flags.LPT = false;
      UpdateSwitch(Crossbar);
    }
  } else {
    SetFlagAndAssertInterrupt(_BV(INTERFACE_INTERRUPT_ESTOPTRIP));
    if(InterruptFlags.flags.AutoEStop) {
      Crossbar.value = 0x00;
      UpdateSwitch(Crossbar);
      Outputs.value = 0x00;
      Outputs.flags.LPTLED = INTERFACE_LED_4HZ;
      Outputs.flags.USBLED = INTERFACE_LED_4HZ;
      UpdateOutputs(Outputs);
    }
  }
}

/* ChargePump */
ISR(TIMER0_COMPA_vect) {
  /* Re-arm */
  OCR0A = TCNT0 + INTERFACE_OCR_12KHZ;
}

/* SysTick */
ISR(TIMER0_COMPB_vect) {
  /* Re-arm */
  OCR0B = TCNT0 + INTERFACE_OCR_32KHZ;

  WallClock++;

  RunPeriodicTasks();
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
  PORTD = (PORTD & ~(_BV(PORTD4) | _BV(PORTD5))) |
      _BV((newOutputs.flags.Spindle ||
          (newOutputs.flags.SpindleFollowsRPM && SpindlePWM)) ? PORTD4 : 0) |
      _BV(newOutputs.flags.Cool ? PORTD5 : 0);
  SetChargePump(newOutputs.flags.ChargePump);

  Outputs = newOutputs;
}

void UpdateSwitch(TCrossbarStatus newState) {
  if(!(newState.flags.LPT && newState.flags.USB)) {
    PORTB = (PORTB & ~(_BV(PORTB0) | _BV(PORTB1))) |
    _BV(newState.flags.LPT ? 0 : PORTB0) | _BV(newState.flags.USB ? 0 : PORTB1);

    Crossbar = newState;
  }
}

void UpdateSpindlePWM(uint8_t newSpindlePWM) {
  if(newSpindlePWM <= 101) {
    if(!newSpindlePWM) {
      TCCR1A &= ~_BV(COM1A1);
      PORTB &= ~_BV(PORTB3);
    } else if(newSpindlePWM == 101) {
      TCCR1A &= ~_BV(COM1A1);
      PORTB |= _BV(PORTB3);
    } else {
      OCR1A = 0x03FFUL * newSpindlePWM / 100;
      TCCR1A |= _BV(COM1A1);
    }

    if(Outputs.flags.SpindleFollowsRPM) UpdateOutputs(Outputs);

    SpindlePWM = newSpindlePWM;
  }
}

void SetChargePump(bool mode) {
  if(mode) {
    TCCR0A |= _BV(COM0A0);
    TIFR |= _BV(OCF0A);
    TIMSK |= _BV(OCIE0A);
  }
  else {
    TCCR0A &= ~_BV(COM0A0);
    TIMSK &= ~_BV(OCIE0A);
  }
}

void RunPeriodicTasks(void) {
  uint16_t WDTDelta = OldWDTime[1] - OldWDTime[0];

  /* Flashing LEDs */
  if(Outputs.flags.LPTLED || Outputs.flags.USBLED) {
    if((Outputs.flags.LPTLED == INTERFACE_LED_1HZ && !(WallClock & 0x3FFF)) ||
        (Outputs.flags.LPTLED == INTERFACE_LED_4HZ && !(WallClock & 0x0FFF)))
      PIND = _BV(PORTD0);
    if((Outputs.flags.USBLED == INTERFACE_LED_1HZ && !(WallClock & 0x3FFF)) ||
        (Outputs.flags.USBLED == INTERFACE_LED_4HZ && !(WallClock & 0x0FFF)))
      PIND = _BV(PORTD1);
  } /* Otherwise they're both off */

  /* External WatchDog detection, between 10.922kHz and 16.384kHz */
  if(WDTDelta > 1 && WDTDelta < 4 && !InterruptCauses.flags.WatchDogTrip)
    SetFlagAndAssertInterrupt(INTERFACE_INTERRUPT_WDOGTRIP);
  else if(!InterruptCauses.flags.WatchDogClear)
    SetFlagAndAssertInterrupt(INTERFACE_INTERRUPT_WDOGCLEAR);
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
  /* SS# on PB4, input and pin change interrupt
   * *_PWM on PB3/2, outputs
   * CS_*# on PB1/0, outputs
   */
  DDRB = (_BV(PORTB3) | _BV(PORTB2) | _BV(PORTB1) | _BV(PORTB0));
  PORTB = (_BV(PORTB1) | _BV(PORTB0));
  /* Interrupt setup */
  PCMSK0 = _BV(PCINT4);
  GIFR |= _BV(INTF0) | _BV(PCIF2) | _BV(PCIF0); /* Avoid spurious interrupts on startup */
  GIMSK |= _BV(INT0) | _BV(PCIE2) | _BV(PCIE0);
  /* Timer setup */
  /* Timer0.A: CPUMP output @ 12.5kHz 50% square wave) */
  OCR0A = INTERFACE_OCR_12KHZ;
  /* Timer0.B: SysTick @ 32768Hz */
  OCR0B = INTERFACE_OCR_32KHZ;
  TIFR |= _BV(OCF0A) | _BV(OCF0B); /* Avoid spurious interrupts on startup */
  TIMSK |= _BV(OCIE0A) | _BV(OCIE0B);
  TCCR0B = _BV(CS01);
  /* Timer1.A: Phase-correct PWM */
  TCCR1A = _BV(COM1A1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(CS11);

  /* Initialize state */
  NewCommand = false;
  set_sleep_mode(SLEEP_MODE_IDLE);

  /* And off we go */
  sei();
}

int main(void) {
  init();
  host_serialconsole_init();
  D('+');

  while(true) {
    if(NewCommand) {
      NewCommand = false;
      D('N');
      D(SPIBuf[0]+'!');
      D(SPIBuf[1]+'!');
      D(SPIBuf[2]+'!');
      switch(SPIBuf[0]) {
        //TODO: the implementation of the global commands could be generic enough to warrant moving to boilerplate
        //-- begin global commands implementation --
        case PROTOCOL_COMMAND_MAC:
          //TODO: add Master Control
          break;
        case (PROTOCOL_COMMAND_WAY | PROTOCOL_RCOMM):
          /* I_2013-1.1-aaa */
          SPIBuf[1] = PROTOCOL_C_WAY_E2013 | PROTOCOL_C_WAY_MA1 | PROTOCOL_C_WAY_MI1;
          SPIBuf[2] = PROTOCOL_C_WAY_FI | PROTOCOL_C_WAY_FWRa | PROTOCOL_C_WAY_HWRa | PROTOCOL_C_WAY_PCRa;
          break;
        case (PROTOCOL_COMMAND_WAY | PROTOCOL_RDATA):
          SPIBuf[1] = PROTOCOL_C_WAY_IS; /* Singleton, no indexing */
          break;
        case (PROTOCOL_COMMAND_AYT | PROTOCOL_RCOMM):
          D('C');
          SPIBuf[1] = _BV(PROTOCOL_C_AYT_YIA) |
              (InterruptCauses.flags.EStopTrip ? 0 : _BV(PROTOCOL_C_AYT_ENVOK)) |
              _BV(PROTOCOL_C_AYT_COMOK);
          break;
        case (PROTOCOL_COMMAND_AYT | PROTOCOL_RDATA):
          SPIBuf[1] = 0xFF; /* We do not provide SYNC */
          break;
        case (PROTOCOL_COMMAND_AYT | PROTOCOL_WCOMM):
          //TODO: implement "only you"
          break;
        case (PROTOCOL_COMMAND_AYT | PROTOCOL_WDATA):
          break; /* We do not accept SYNC */
        case (PROTOCOL_COMMAND_HLT | PROTOCOL_RCOMM):
        case (PROTOCOL_COMMAND_HLT | PROTOCOL_RDATA):
        case (PROTOCOL_COMMAND_HLT | PROTOCOL_WCOMM):
          /* We do not intend to ever wake up */
          cli();
          /* Turn off SPINDLE and COOL */
          Outputs.value = 0x00;
          Outputs.flags.ChargePump = true;
          UpdateOutputs(Outputs);
          /* Turn off the stepper driver */
          Outputs.flags.ChargePump = false;
          UpdateOutputs(Outputs);
          /* Isolate crossbar */
          Crossbar.value = 0x00;
          UpdateSwitch(Crossbar);
          break;
        case (PROTOCOL_COMMAND_HLT | PROTOCOL_WDATA):
          cli();
          set_sleep_mode(SLEEP_MODE_PWR_DOWN);
          break;
        //-- end global commands implementation --
        case (INTERFACE_COMMAND_INTERRUPT | PROTOCOL_RCOMM):
          SPIBuf[1] = InterruptFlags.value;
          break;
        case (INTERFACE_COMMAND_INTERRUPT | PROTOCOL_RDATA):
          SPIBuf[1] = (InterruptCauses.flags.EStopTrip ? _BV(7) : 0) |
              (InterruptCauses.flags.WatchDogTrip ? _BV(6) : 0);
          ClearFlagsAndReleaseInterrupt();
          break;
        case (INTERFACE_COMMAND_INTERRUPT | PROTOCOL_WCOMM):
          InterruptFlags.value = SPIBuf[1];
          break;
        case (INTERFACE_COMMAND_OUTPUT | PROTOCOL_RCOMM):
          SPIBuf[1] = Outputs.value;
          break;
        case (INTERFACE_COMMAND_OUTPUT | PROTOCOL_WCOMM):
          if(Outputs.value != SPIBuf[1])
            UpdateOutputs((TOutputStatus)SPIBuf[1]);
          break;
        case (INTERFACE_COMMAND_CROSSBAR | PROTOCOL_RCOMM):
          SPIBuf[1] = Crossbar.value;
          break;
        case (INTERFACE_COMMAND_CROSSBAR | PROTOCOL_WCOMM):
          if(Crossbar.value != SPIBuf[1])
            UpdateSwitch((TCrossbarStatus)SPIBuf[1]);
          break;
        case (INTERFACE_COMMAND_SPINDLE | PROTOCOL_RDATA):
          SPIBuf[1] = SpindlePWM;
          break;
        case (INTERFACE_COMMAND_SPINDLE | PROTOCOL_WDATA):
          if(SpindlePWM != SPIBuf[1]) UpdateSpindlePWM(SPIBuf[1]);
          break;
      }
    } else sleep_mode();
  }

  return 0;
}
