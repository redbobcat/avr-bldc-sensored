//#define F_CPU 8000000UL  // 8 MHz
//#define BAUD 4800
//#define MYUBRR F_CPU/16/BAUD-1
#include <avr/io.h>


void USART_Init( unsigned int ubrr)
{
/*Set baud rate*/
UBRR0H = (unsigned char)(ubrr>>8);
UBRR0L = (unsigned char)ubrr;
/*Enable receiver and transmitter*/
UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);//|(1<<TXCIE)
/*Set frame format: 8data, 1stop bit*/
UCSR0C = (3<<UCSZ00);//
}

unsigned char USART_Receive( void )
{
while ( !(UCSR0A & (1<<RXC0)) );
return UDR0;
}

void USART_Transmit( unsigned char data )
{
while ( !( UCSR0A & (1<<UDRE0)) );
UDR0 = data;
}
