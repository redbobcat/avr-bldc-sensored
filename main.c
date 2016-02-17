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

//#define DEBUG

#include <avr/io.h>
//#include <util/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
#include "buttons.h"
#include "bldc.h"
#include "pid.h"

#ifdef DEBUG
	#include "uart.h"
	#include <stdio.h>
#endif



//counter for serving others, 50hz
inline void counter2_init (void);
//ISR is in the end
//variables for counter
volatile uint8_t ten_sec=6;
//one_sec=51

//ACD functions
inline void ADC_init (void); // init of ADC
void ADC_start (uint8_t); // starts converson on ch

//volatile uint8_t adc_ch[2], adc_ch_now=6,;


volatile uint8_t status_byte=0;

#define SETBIT(x,y) (x |= (y)) /* set bit in x */
#define CLEARBIT(x,y) (x &= (~y)) /* clear bit in x */
#define CHECKBIT(x,y) (x & (y)) /* check bit in x */
 
#define _send 0x01 
#define _pid 0x02
//#define EMPTY 0x04
//#define FULL 0x08

//speed mesure and current
volatile uint16_t rpm=0, target_rpm=0;


//buttons and some other
volatile uint8_t buttons_pressed;
#define BUTTON_RIGHT 0b10000000
#define BUTTON_LEFT 0b00100000
#define BUTTON_ONE 0b00000100

//temp section for debug

void main(void)
{
#ifdef DEBUG
	uart_init(UART_BAUD_SELECT(19200,F_CPU));
#endif

	struct PID_DATA firstPID;
	initializePID(&firstPID, 0.02, 0.0001, 0.00002); //0.01 0.0001 0.00001 works
		
	bldc_all_init();
	
	ADC_init();
	counter2_init();
	
	buttons_init();
	
	DDRD |= (1<<PD3) | (1<<PD4); //leds on board
	
	sei();
	
	for (;;)
		{
			//PWM_POWER=adc_ch[1];
			
			if ((buttons_pressed & BUTTON_RIGHT) && (dir)==0) //start CW
				{
					dir=1;
					bldc_start(dir);
					buttons_pressed=0;
					PORTD |= 1<<PD3;
					
					
				}
			if ((buttons_pressed & BUTTON_LEFT) && (dir)==0) //start CCW
				{
					dir=2;
					bldc_start(dir);
					buttons_pressed=0;
					PORTD |= 1<<PD4;
					
					
				}
			if (dir && (buttons_pressed & BUTTON_ONE)) //stop
				{
					dir=0;
					bldc_stop();
					buttons_pressed=0;
					PORTD &= ~((1<<PD3)|(1<<PD4));
					CLEARBIT(status_byte,_send);
				}
		#ifdef DEBUG
			if (CHECKBIT(status_byte,_send))
						{
							char out[5];
														
							sprintf(out, "%d", rpm);
							uart_puts(out);
							uart_putc(',');
							
							sprintf(out, "%d", target_rpm);
							uart_puts(out);
							uart_putc('\n');
							CLEARBIT(status_byte,_send);
											
						}
		#endif
						
			if (CHECKBIT(status_byte,_pid))
					{
						PORTD |= 1<<PD4;
						
						int16_t pwm_temp;
						pwm_temp = PWM_POWER;
							//target_rpm=3000;
						pwm_temp += stepPID(&firstPID, rpm, target_rpm);
						if (pwm_temp>255) pwm_temp=255;
						if (pwm_temp<0) pwm_temp=0;
						PWM_POWER = pwm_temp;
						CLEARBIT(status_byte,_pid);
						
						PORTD &= ~((1<<PD4));
						
					}
				
		}
	
	
	
		
}
//END OF MAIN
inline void counter2_init (void)
	{
		TCCR2A |= (1<<WGM21); //CTC mode
		TCCR2B |= 7;//(7<<CS20); //prescaler /1024
		OCR2A=234;  //234 gives 50 ints/s
		TIMSK2 |= (1<<OCIE2A); //enabling A interupt
	}
	
inline void ADC_init (void)
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
			/*if (one_sec==0) //1 time in sec
				{
					
					one_sec=51;
				}*/
			if (ten_sec==0) //10 tims in sec
				{
					rpm = ticks/3;
					rpm *= 50;
					ticks =0;
					if (dir) 
						{
							SETBIT(status_byte,_send);
							SETBIT(status_byte,_pid);
						}
						
					
						buttons_pressed |= buttons();
						/*if (adc_ch_now==6)
							{ //from isense resistor
								adc_ch[0] = ADCH;
								adc_ch_now=7;
								ADC_start(adc_ch_now); 
							}
						else
							{ //from potentiometer
								adc_ch[1] = ADCH;
								adc_ch_now=6;
								ADC_start(adc_ch_now);
								target_rpm = adc_ch[1]*50;
							}*/
							
						target_rpm = ADCH*50; //set target rpm
						
						ADC_start(7);
						
						ten_sec=6;
						
				}
			
			
			ten_sec--;
			//one_sec--;
			sei();
	}



