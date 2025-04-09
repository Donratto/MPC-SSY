/*
 * uart.c
 *
 * Created: 10.2.2020 13:26:29
 *  Author: Krajsa
 */ 
#include <avr/io.h>
#include <stdio.h>
#include "../makra.h"
void UART_init(uint16_t Baudrate,uint8_t interrupt){
	//volatile double test=((8000000/16)/Baudrate)-1;
	int ubrr=((F_CPU/16/Baudrate)-1);
	UBRR1H = (uint8_t)(ubrr>>8);
	UBRR1L = (uint8_t)ubrr;
	UCSR1B = (1<<RXEN1)|(1<<TXEN1);// Povolit prijem/vydej
	UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);// Async, Parity-Disabled, 1 Stop bit, 8 data bits
	if (interrupt) {
	UCSR1B |= (1 << RXCIE1); //Povoleni preruseni pri prijmu
	}
}
void UART_SendChar(uint8_t data)
{
	while ( !( UCSR1A & (1<<UDRE0)) )
	;
	UDR1 = data;
}
uint8_t UART_GetChar( void )
{
	while ( !(UCSR1A & (1<<RXC0)) )
	;
	return UDR1;
}
void UART_SendString(char *text)
{
	while (*text != 0x00)
	{
		UART_SendChar(*text);
		text++;
	}
}
int printCHAR(char character, FILE *stream)
{
	while ((UCSR1A & (1 << UDRE1)) == 0) {};

	UDR1 = character;

	return 0;
}
