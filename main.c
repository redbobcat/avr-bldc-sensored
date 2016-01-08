/*
 * Main.c
 * Main of bldc driver, based on atmega48a.
 * My contain a lot of noncomented debug and dumb things. Now.
 * V0.000001
 * 
 * 07.01.2016
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
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "usart.h"


inline void 	bldc_out_init (void); //bldc out port configure, output, sets to 0

inline void 	bldc_hall_init (void); //bldc hall in configure, input, hi-z

inline uint8_t 	bldc_hall_state (void); //returns hall sensor state

void 			bldc_out_set (uint8_t); //closes all gates and sets all to next

uint8_t 		bldc_switch (uint8_t hallin, uint8_t direction); //2 for CCW, 0 for none, 1 for CW

const uint8_t 	bldc_state[] = {0b00100001, 0b00001001, 0b00011000, 0b00010010, 0b00000110, 0b00100100}; //output table for switching

inline void 	bldc_start (uint8_t); //2 for CCW, 0 for none, 1 for CW

inline void 	bldc_stop (void);


//enable pinchange interrupt for PB0-PB2
inline void		bldc_interupt_enable (void);
inline void		bldc_interupt_disable (void);
// ISR of pinchange interupt is in the end of file

void 			bldc_pwm_enable (void);
//#define PWM_DRIVE OCR0A;

//temp section for debug
uint8_t dir=1, pwm=255;

void main(void)
{
	USART_Init(MYUBRR);
	
	bldc_hall_init();
	
	bldc_out_init();
	
	bldc_pwm_enable();
	
	OCR0A = pwm;
	bldc_interupt_enable();
	sei();
	
	for (;;)
		{
			_delay_ms(20);
		}
	
	
		
}

inline void bldc_out_init (void)
	{
		PHASE_DDR |= PHASE_MASK;
		PHASE_PORT &= ~(PHASE_MASK);
	}
	
inline void bldc_hall_init (void)
	{
		HALL_DDR &= ~(HALL_MASK);
		HALL_PORT &= ~(HALL_MASK);
	}
inline uint8_t bldc_hall_state (void)
	{
	return HALL_PIN & HALL_MASK;	
	}
void bldc_out_set (uint8_t in)
	{
	PHASE_PORT &= ~(PHASE_MASK);
	PHASE_PORT |= in;
	}

uint8_t bldc_switch (uint8_t hallin, uint8_t direction)
	{	uint8_t state_temp=0;
		if (direction==0)
			{
				return 0;
			}
		switch (hallin)
			{
				case 0b00000101: {state_temp=bldc_state[0+direction-1]; break;}
				case 0b00000001: {state_temp=bldc_state[1+direction-1]; break;}
				case 0b00000011: {state_temp=bldc_state[2+direction-1]; break;}
				case 0b00000010: {state_temp=bldc_state[3+direction-1]; break;}
				case 0b00000110: {state_temp=bldc_state[4+direction-1]; break;}
				case 0b00000100: 
					{ if (direction == 1) state_temp=bldc_state[5+direction-1];
						else state_temp=bldc_state[0];
					}
			}
				
		return state_temp;	
	}

inline void bldc_start (uint8_t cwccw)
	{
			PHASE_PORT=bldc_switch(bldc_hall_state(),cwccw);
			OCR0A = 125;
			//_delay_ms(20);
			bldc_interupt_enable();
			sei();
	}
	
inline void bldc_stop(void)
	{
			
			bldc_interupt_disable();
			PHASE_PORT &= ~(PHASE_MASK);
	}

inline void bldc_interupt_enable (void)
	{
			PCICR |= 1<<PCIE0;
			PCMSK0|= HALL_MASK;
	}
	
inline void	bldc_interupt_disable (void)
	{
			PCICR &= ~(1<<PCIE0);
			PCMSK0 &= ~(HALL_MASK);
	}

ISR(PCINT0_vect)
	{
			PHASE_PORT=bldc_switch(bldc_hall_state(), dir);
			sei();
	}

void bldc_pwm_enable (void)
	{
			TCCR0A |= (1<<COM0A1) | (1<<WGM00) | (1<<WGM01); //set fast-pwm, non-inverting out
			DDRD |= 1<<PD6;
			TCCR0B |= 1<<CS00; //start at full speed
			
	}
