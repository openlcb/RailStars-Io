/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  $Id: wiring.h 249 2007-02-03 16:52:51Z mellis $
*/

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <avr/pgmspace.h>

// Fix for Tone.cpp timer errors
#if defined(__AVR_AT90CAN128__)
#define TCCR2B TCCR2A
#define TIMER2_COMPA_vect TIMER2_COMP_vect
#endif

#ifndef PCICR
#define PCICE EICR
#endif
#ifndef PCMSK
#define PCMSK EIMSK
#endif

#define NUM_DIGITAL_PINS            50
#define NUM_ANALOG_INPUTS           8

#define analogInputToDigitalPin(p)  ((p < 8) ? (p) + 40 : -1)

/* not gonna bother with PWM */
#define digitalPinHasPWM(p)         (0)

static const uint8_t SS   = 16;
static const uint8_t MOSI = 18;
static const uint8_t MISO = 19;
static const uint8_t SCK  = 17;

static const uint8_t SDA = 25;
static const uint8_t SCL = 24;
static const uint8_t LED_BUILTIN = 48;
static const uint8_t BLUE = 48;
static const uint8_t GOLD = 49;

static const uint8_t A0 = 40;
static const uint8_t A1 = 41;
static const uint8_t A2 = 42;
static const uint8_t A3 = 43;
static const uint8_t A4 = 44;
static const uint8_t A5 = 45;
static const uint8_t A6 = 46;
static const uint8_t A7 = 47;

//PC interrupts are not available on the AT90CAN128
#define digitalPinToPCICR(p)    ((uint8_t *)0)
#define digitalPinToPCICRbit(p) (1)
#define digitalPinToPCMSK(p)    ((uint8_t *)0)
#define digitalPinToPCMSKbit(p) (p)

#ifdef ARDUINO_MAIN

// these arrays map port names (e.g. port B) to the
// appropriate addresses for various functions (e.g. reading
// and writing)
const uint16_t PROGMEM port_to_mode_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &DDRA,
	(uint16_t) &DDRB,
	(uint16_t) &DDRC,
	(uint16_t) &DDRD,
	(uint16_t) &DDRE,
	(uint16_t) &DDRF,
	(uint16_t) &DDRG,
};

const uint16_t PROGMEM port_to_output_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PORTA,
	(uint16_t) &PORTB,
	(uint16_t) &PORTC,
	(uint16_t) &PORTD,
	(uint16_t) &PORTE,
	(uint16_t) &PORTF,
	(uint16_t) &PORTG,
};

const uint16_t PROGMEM port_to_input_PGM[] = {
	NOT_A_PORT,
	(uint16_t) &PINA,
	(uint16_t) &PINB,
	(uint16_t) &PINC,
	(uint16_t) &PIND,
	(uint16_t) &PINE,
	(uint16_t) &PINF,
	(uint16_t) &PING,
};

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
	PA, /* Arduino Pin  0 OUTPUTS */
	PA, /* Arduino Pin  1 */
	PA, /* Arduino Pin  2 */
	PA, /* Arduino Pin  3 */
	PA, /* Arduino Pin  4 */
	PA, /* Arduino Pin  5 */
	PA, /* Arduino Pin  6 */
	PA, /* Arduino Pin  7 */
	PC, /* Arduino Pin  8 INPUTS */
	PC, /* Arduino Pin  9 */
	PC, /* Arduino Pin 10 */
	PC, /* Arduino Pin 11 */
	PC, /* Arduino Pin 12 */
	PC, /* Arduino Pin 13 */
	PC, /* Arduino Pin 14 */
	PC, /* Arduino Pin 15 */
	PB, /* Arduino Pin 16 PORT B*/
	PB, /* Arduino Pin 17 */
	PB, /* Arduino Pin 18 */
	PB, /* Arduino Pin 19 */
	PB, /* Arduino Pin 20 */
	PB, /* Arduino Pin 21 */
	PB, /* Arduino Pin 22 */
	PB, /* Arduino Pin 23 */
	PD, /* Arduino Pin 24 PORT D/G */
	PD, /* Arduino Pin 25 */
	PD, /* Arduino Pin 26 */
	PD, /* Arduino Pin 27 */
	PD, /* Arduino Pin 28 */
	PG, /* Arduino Pin 29 PD5 is CAN: Sub in PG3 here */
	PG, /* Arduino Pin 30 PD6 is CAN: Sub in PG4 here */
	PD, /* Arduino Pin 31 */
	PE, /* Arduino Pin 32 PORT E */
	PE, /* Arduino Pin 33 */
	PE, /* Arduino Pin 34 */
	PE, /* Arduino Pin 35 */
	PE, /* Arduino Pin 36 */
	PE, /* Arduino Pin 37 */
	PE, /* Arduino Pin 38 */
	PE, /* Arduino Pin 39 */
	PF, /* Arduino Pin 40 PORT F */
	PF, /* Arduino Pin 41 */
	PF, /* Arduino Pin 42 */
	PF, /* Arduino Pin 43 */
	PF, /* Arduino Pin 44 */
	PF, /* Arduino Pin 45 */
	PF, /* Arduino Pin 46 */
	PF, /* Arduino Pin 47 */
	PG, /* Arduino Pin 48 BLUE */
	PG, /* Arduino Pin 49 GOLD */
};

const uint8_t PROGMEM digital_pin_to_bit_mask_PGM[] = {
	_BV(0), /* 0, port A Arduino Pin  0 */
	_BV(1), /* 1, port A Arduino Pin  1 */
	_BV(2), /* 2, port A Arduino Pin  2 */
	_BV(3), /* 3, port A Arduino Pin  3 */
	_BV(4), /* 4, port A Arduino Pin  4 */
	_BV(5), /* 5, port A Arduino Pin  5 */
	_BV(6), /* 6, port A Arduino Pin  6 */
	_BV(7), /* 7, port A Arduino Pin  7 */
	
	_BV(0), /* 0, port C Arduino Pin  8 */
	_BV(1), /* 1, port C Arduino Pin  9 */
	_BV(2), /* 2, port C Arduino Pin 10 */
	_BV(3), /* 3, port C Arduino Pin 11 */
	_BV(4), /* 4, port C Arduino Pin 12 */
	_BV(5), /* 5, port C Arduino Pin 13 */
	_BV(6), /* 6, port C Arduino Pin 14 */
	_BV(7), /* 7, port C Arduino Pin 15 */
	
	_BV(0), /* 0, port B Arduino Pin 16 */
	_BV(1), /* 1, port B Arduino Pin 17 */
	_BV(2), /* 2, port B Arduino Pin 18 */
	_BV(3), /* 3, port B Arduino Pin 19 */
	_BV(4), /* 4, port B Arduino Pin 20 */
	_BV(5), /* 5, port B Arduino Pin 21 */
	_BV(6), /* 6, port B Arduino Pin 22 */
	_BV(7), /* 7, port B Arduino Pin 23 */
	
	_BV(0), /* 0, port D Arduino Pin 24 */
	_BV(1), /* 1, port D Arduino Pin 25 */
	_BV(2), /* 2, port D Arduino Pin 26 */
	_BV(3), /* 3, port D Arduino Pin 27 */
	_BV(4), /* 4, port D Arduino Pin 28 */
	_BV(3), /* 3, port G Arduino Pin 29 */
	_BV(4), /* 4, port G Arduino Pin 30 */
	_BV(7), /* 7, port D Arduino Pin 31 */
	
	_BV(0), /* 0, port E Arduino Pin 32 */
	_BV(1), /* 1, port E Arduino Pin 33 */
	_BV(2), /* 2, port E Arduino Pin 34 */
	_BV(3), /* 3, port E Arduino Pin 35 */
	_BV(4), /* 4, port E Arduino Pin 36 */
	_BV(5), /* 5, port E Arduino Pin 37 */
	_BV(6), /* 6, port E Arduino Pin 38 */
	_BV(7), /* 7, port E Arduino Pin 39 */

	_BV(0), /* 0, port F Arduino Pin 40 */
	_BV(1), /* 1, port F Arduino Pin 41 */
	_BV(2), /* 2, port F Arduino Pin 42 */
	_BV(3), /* 3, port F Arduino Pin 43 */
	_BV(4), /* 4, port F Arduino Pin 44 */
	_BV(5), /* 5, port F Arduino Pin 45 */
	_BV(6), /* 6, port F Arduino Pin 46 */
	_BV(7), /* 7, port F Arduino Pin 47 */
	
	_BV(0), /* 0, port G BLUE Arduino Pin 48 */
	_BV(1), /* 1, port G GOLD Arduino Pin 49 */
};


/* Just not gonna mess with it! */
const uint8_t PROGMEM digital_pin_to_timer_PGM[] = {
	NOT_ON_TIMER, /* 0  */
	NOT_ON_TIMER, /* 1  */
	NOT_ON_TIMER, /* 2  */
	NOT_ON_TIMER, /* 3  */
	NOT_ON_TIMER, /* 4  */
	NOT_ON_TIMER, /* 5  */
	NOT_ON_TIMER, /* 6  */
	NOT_ON_TIMER, /* 7  */
	NOT_ON_TIMER, /* 8  */
	NOT_ON_TIMER, /* 9  */
	NOT_ON_TIMER, /* 10 */
	NOT_ON_TIMER, /* 11 */
	NOT_ON_TIMER, /* 12 */
	NOT_ON_TIMER, /* 13 */
	NOT_ON_TIMER, /* 14 */
	NOT_ON_TIMER, /* 15 */
	NOT_ON_TIMER, /* 16 */
	NOT_ON_TIMER, /* 17 */
	NOT_ON_TIMER, /* 18 */
	NOT_ON_TIMER, /* 19 */
	NOT_ON_TIMER, /* 20 */
	NOT_ON_TIMER, /* 21 */
	NOT_ON_TIMER, /* 22 */
	NOT_ON_TIMER, /* 23 */
	NOT_ON_TIMER, /* 24 */
	NOT_ON_TIMER, /* 25 */
	NOT_ON_TIMER, /* 26 */
	NOT_ON_TIMER, /* 27 */
	NOT_ON_TIMER, /* 28 */
	NOT_ON_TIMER, /* 29 */
	NOT_ON_TIMER, /* 30 */
	NOT_ON_TIMER, /* 31 */
	NOT_ON_TIMER, /* 32 */
	NOT_ON_TIMER, /* 33 */
	NOT_ON_TIMER, /* 34 */
	NOT_ON_TIMER, /* 35 */
	NOT_ON_TIMER, /* 36 */
	NOT_ON_TIMER, /* 37 */
	NOT_ON_TIMER, /* 38 */
	NOT_ON_TIMER, /* 39 */
	NOT_ON_TIMER, /* 40 */
	NOT_ON_TIMER, /* 41 */
	NOT_ON_TIMER, /* 42 */
	NOT_ON_TIMER, /* 43 */
	NOT_ON_TIMER, /* 44 */
	NOT_ON_TIMER, /* 45 */
	NOT_ON_TIMER, /* 46 */
	NOT_ON_TIMER, /* 47 */
	NOT_ON_TIMER, /* 48 */
	NOT_ON_TIMER, /* 49 */
};

#endif

#endif
