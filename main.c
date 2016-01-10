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
#define PHASE_STOP_MASK 0b00010101
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
//#include <avr/pgmspace.h>
#include "usart.h"
#include "buttons.h"


//funcs for bldc drive
inline uint8_t 	bldc_hall_state (void); //returns hall sensor state
void 			bldc_out_set (uint8_t); //closes all gates and sets all to next
uint8_t 		bldc_switch (uint8_t hallin, uint8_t direction); //2 for CCW, 0 for none, 1 for CW
const uint8_t 	bldc_state[] = {0b00100001, 0b00001001, 0b00011000, 0b00010010, 0b00000110, 0b00100100}; //output table for switching
inline void 	bldc_out_init (void); //bldc out port configure, output, sets to 0
inline void 	bldc_hall_init (void); //bldc hall in configure, input, hi-z
void 			bldc_pwm_enable (void); //pwm enabling on PD6 pin, ~31KHz
#define PWM_POWER OCR0A

	//enable pinchange interrupt for PB0-PB2
inline void		bldc_interupt_enable (void);
inline void		bldc_interupt_disable (void);
	// ISR of pinchange interupt is in the end of file




//top funcs for bldc
inline void 	bldc_start (uint8_t); //2 for CCW, 0 for none, 1 for CW
inline void 	bldc_stop (void);
inline void 	bldc_all_init (void); //all init in one


//counter for serving others, 50hz
void counter2_init (void);
//ISR is in the end
//variables for counter
volatile uint8_t one_sec=51,ten_sec=6;


//ACD functions
void ADC_init (void); // init of ADC
void ADC_start (uint8_t); // starts converson on ch

volatile uint8_t adc_ch[2];
volatile uint8_t adc_ch_now=6, send;

volatile uint8_t buttons_pressed, dir=0;
#define BUTTON_RIGHT 0b10000000
#define BUTTON_LEFT 0b00100000
#define BUTTON_ONE 0b00000100

//speed mesure
volatile uint16_t ticks=0, rpm=0, current;


//temp section for debug

void main(void)
{
	USART_Init(MYUBRR);
		
	bldc_all_init();
	
	ADC_init();
	counter2_init();
	
	buttons_init();
	
	DDRD |= (1<<PD3) | (1<<PD4);
	
	sei();
	
	for (;;)
		{
			PWM_POWER=adc_ch[1];
			
			if ((buttons_pressed & BUTTON_RIGHT) && (dir)==0)
				{
					dir=1;
					bldc_start(dir);
					buttons_pressed=0;
					PORTD |= 1<<PD3;
					
				}
			if ((buttons_pressed & BUTTON_LEFT) && (dir)==0)
				{
					dir=2;
					bldc_start(dir);
					buttons_pressed=0;
					PORTD |= 1<<PD4;
				}
			if (dir && (buttons_pressed & BUTTON_ONE))
				{
					dir=0;
					bldc_stop();
					buttons_pressed=0;
					PORTD &= ~((1<<PD3)|(1<<PD4));
				}
				
			if (send==1)
				{
					USART_Transmit('R');

					USART_Transmit(0x30+(rpm/10000));
					USART_Transmit(0x30+((rpm/1000)%10));
					USART_Transmit(0x30+((rpm/100)%10));
					USART_Transmit(0x30+((rpm/10)%10));
					USART_Transmit(0x30+(rpm%10));
					
					USART_Transmit(10);
					
					current = adc_ch[0]*20;

					USART_Transmit('I');

					USART_Transmit(0x30+(current/10000));
					USART_Transmit(0x30+((current/1000)%10));
					USART_Transmit(0x30+((current/100)%10));
					USART_Transmit(0x30+((current/10)%10));
					USART_Transmit(0x30+(current%10));
					
					USART_Transmit(10);
					send=0;
						
				}
			
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
		//HALL_PORT |= (HALL_MASK);
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
		if (ticks<65535) ticks++;
		
		if (direction==0)
			{
				return 0;
			}
		if (direction==1) //CW
			{
		
		switch (hallin)
				{
				case 0b00000101: {state_temp=bldc_state[3+direction-1]; break;}
				case 0b00000001: {state_temp=bldc_state[4+direction-1]; break;}
				case 0b00000011: {state_temp=bldc_state[5+direction-1]; break;}
				case 0b00000010: {state_temp=bldc_state[0+direction-1]; break;}
				case 0b00000110: {state_temp=bldc_state[1+direction-1]; break;}
				case 0b00000100: {state_temp=bldc_state[2+direction-1]; break;}
				}
			}
			
		if (direction==2) //CCW must be changed!!! CHANGED, test it!
			{
				switch (hallin)
				{
				case 0b00000101: {state_temp=bldc_state[5+direction-1]; break;}
				case 0b00000001: {state_temp=bldc_state[0+direction-1]; break;}
				case 0b00000011: {state_temp=bldc_state[1+direction-1]; break;}
				case 0b00000010: {state_temp=bldc_state[2+direction-1]; break;}
				case 0b00000110: {state_temp=bldc_state[3+direction-1]; break;}
				case 0b00000100: {state_temp=bldc_state[4+direction-1]; break;}
				}
			}
				
		return state_temp;	
	}

inline void bldc_start (uint8_t cwccw)
	{
			PHASE_PORT=bldc_switch(bldc_hall_state(),cwccw);
			PWM_POWER = 255;
			_delay_ms(30);
			bldc_interupt_enable();
			sei();
	}
	
inline void bldc_stop(void)
	{
			
			bldc_interupt_disable();
			PHASE_PORT &= ~(PHASE_MASK);
			//PHASE_PORT |= PHASE_STOP_MASK;
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
			//USART_Transmit(0x30+bldc_hall_state());
			sei();
	}

void bldc_pwm_enable (void)
	{
			TCCR0A |= (1<<COM0A1) | (1<<WGM00) | (1<<WGM01); //set fast-pwm, non-inverting out
			DDRD |= 1<<PD6;
			TCCR0B |= 1<<CS00; //start at full speed
			
	}

inline void bldc_all_init(void)
	{
		bldc_hall_init();
		bldc_out_init();
		bldc_pwm_enable();
		
	}
	
void counter2_init (void)
	{
		TCCR2A |= (1<<WGM21); //CTC mode
		TCCR2B |= 7;//(7<<CS20); //prescaler /1024
		OCR2A=234;  //234 gives 50 ints/s
		TIMSK2 |= (1<<OCIE2A); //enabling A interupt
	}
	
ISR (TIMER2_COMPA_vect) //50 times/s
	{
			if (one_sec==0)
				{
					send=1;
					one_sec=51;
				}
			if (ten_sec==0)
				{
					rpm = ticks/3;
					rpm *= 50;
					ticks =0;
					
						buttons_pressed |= buttons();
						if (adc_ch_now==6)
							{
								adc_ch[0] = ADCH;
								adc_ch_now=7;
								ADC_start(adc_ch_now); 
							}
						else
							{
								adc_ch[1] = ADCH;
								adc_ch_now=6;
								ADC_start(adc_ch_now); 
							}
						
						ten_sec=6;
				}
			
			ten_sec--;
			one_sec--;
			sei();
	}
	
void ADC_init (void)
	{
		ADMUX |= (1<<REFS0) | (1<<ADLAR); //ref to VCC, left adjust
		ADCSRA |= (1<<ADEN) | (6<<ADPS0); //enabling ADC, pescaling /64, 180Khz
	}
void ADC_start (uint8_t chno)
	{
		ADMUX = (ADMUX & 0b11110000) | (chno); //set chanel
		ADCSRA |= (1<<ADEN) | (1<<ADSC);
	}
