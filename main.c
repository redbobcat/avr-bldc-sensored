/*
 * Main.c
 * Main of bldc driver, based on atmega48a.
 * My contain a lot of noncomented debug and dumb things. Now.
 * V0.000001
 * 
 * 
 * Copyright 2016 redbobcat <redbobcat@Redbobcat-net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */

#define F_CPU 12000000 //12MHz crystal

//out regs for power stage, UH UL VH VL WH WL
//now C port, from 0 to 5
#define PHASE_DDR DDRC
#define PHASE_PORT PORTC
#define PHASE_PIN PINC
#define PHASE_MASK 0b00111111 //may be usefull, MUST BE CHANGED IF SCHEME CHANGED
//in pins for hall sensors outbut. With amp. 
//now B, 0 to 2 "U V W"
#define HALL_DDR DDRB
#define HALL_PORT PORTB
#define HALL_PIN PINB
#define HALL_MASK 0b00000111 //my be usefull, MUST BE CHANGED IF SCHEME CHANGED

#define ADC_WIPER 7 //in from potentiometer, 0-5V
#define ADC_CURRENT_SENCE 6 //in from amp, 1V to 1A

#include <avr/io.h>
#include <util/delay.h>




void main(void)
{
	
	
	
}

