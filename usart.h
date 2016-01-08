/*
 * usart.h
 *
 *  Created on: 09.05.2011
 *      Author: red_bobcat
 */
//#define F_CPU 8000000UL  // 8 MHz
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1



void USART_Init( unsigned int ubrr);

unsigned char USART_Receive( void );

void USART_Transmit( unsigned char data );
