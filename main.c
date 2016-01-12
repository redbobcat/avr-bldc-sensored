/*
 * Main.c
 * Main of bldc driver, based on atmega48a.
 * My contain a lot of noncomented debug and dumb things. Now.
 * V0.000001
 * 
 * 01.2016
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

#define ADC_WIPER 7 //in from potentiometer, 0-5V
#define ADC_CURRENT_SENCE 6 //in from amp, 1V to 1A

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
#include "usart.h"
#include "buttons.h"
#include "bldc.h"




//counter for serving others, 50hz
void counter2_init (void);
//ISR is in the end
//variables for counter
volatile uint8_t one_sec=51,ten_sec=6;


//ACD functions
void ADC_init (void); // init of ADC
void ADC_start (uint8_t); // starts converson on ch

volatile uint8_t adc_ch[2];
volatile uint8_t adc_ch_now=6, send=0;

//speed mesure and current
volatile uint16_t rpm=0, current=0;


//buttons and some other
volatile uint8_t buttons_pressed;
#define BUTTON_RIGHT 0b10000000
#define BUTTON_LEFT 0b00100000
#define BUTTON_ONE 0b00000100

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
//END OF MAIN
void counter2_init (void)
	{
		TCCR2A |= (1<<WGM21); //CTC mode
		TCCR2B |= 7;//(7<<CS20); //prescaler /1024
		OCR2A=234;  //234 gives 50 ints/s
		TIMSK2 |= (1<<OCIE2A); //enabling A interupt
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



