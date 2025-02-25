/*
 * LAB2.c
 *
 * Created: 2/25/2025 12:20:23
 * Author : Student
 */ 
/************************************************************************/
/* INCLUDE                                                              */
/************************************************************************/
#include <avr/io.h>
//#include <util/delay.h>
#include <stdint.h>

//#include "./makra.h"
//#include "libs/libprintfuart.h"





/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

//#define F_CPU 8000000
#define BAUD 38400
#define MYUBRR ((F_CPU/(16*BAUD))-1)
// F_CPU definovano primo v projektu!!! Debug->Properties->Toolchain->Symbols

/************************************************************************/
/* VARIABLES                                                            */
/************************************************************************/

//musime vytvorit soubor pro STDOUT
//FILE uart_str = FDEV_SETUP_STREAM(printCHAR, NULL, _FDEV_SETUP_RW);

uint8_t pismeno = 0x41; //A
char *string = "Hello";

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void board_init();
void USART_Init();
void USART_Transmist(uint8_t symbol);
uint8_t USART_Receive(void);

void UART_SendChar(uint8_t data);
void UART_SendString(char *text);

/************************************************************************/
/* FUNCTIONS                                                            */
/************************************************************************/

void USART_Init(uint16_t _ubrr)
{
	UBRR1H = (uint8_t)(_ubrr>>8);
	UBRR1L = (uint8_t)(_ubrr);
	
	UCSR1B = (1<<RXEN1) | (1<<TXEN1); //zapnuti prijmu a vysilani
	
	UCSR1C &=~ (1<<UCSZ11) | (1<<UCSZ10); // frame size 8 bits
	//UCSR1B &=~ (1<<UCSZ11); // default val frame size
	
	//UCSR1C |= (1<<UPM11)|(1<<UPM10); //  default val asynchro mod
	//UCSR1C&=~(1<<UMSEL11)|(1<<UMSEL10); // default val parity
	
	//UCSR1C &=~ (1<<USBS1); //default val stop bits = 1
}

void USART_Transmist(uint8_t data)
{
	while ( !( UCSR1A & (1<<UDRE1)) );
	/* Put data into buffer, sends the data */
	UDR1 = data;
}

uint8_t USART_Receive(void)
{
	/* Wait for data to be received */
	while ( !(UCSR1A & (1<<RXC1)) );
	/* Get and return received data from buffer */
	return UDR1;
}


void UART_SendChar(uint8_t data)
{
	while ( !( UCSR1A & (1<<UDRE1)) );
	/* Put data into buffer, sends the data */
	UDR1 = data;
}

void UART_SendString(char *text)
{
	while (*text != 0x00){
		UART_SendChar(*text);
		text++;
	}
}

void board_init()
{
	USART_Init(MYUBRR);
	//UART_init(38400); //nastaveni rychlosti UARTu, 38400b/s
	//stdout = &uart_str; //presmerovani STDOUT
}


int main(void)
{
	
	board_init();
	
	
	UART_SendChar(pismeno);
	UART_SendChar(pismeno);
	UART_SendChar(pismeno);
	UART_SendChar(pismeno);
	UART_SendChar(pismeno);
	UART_SendString(string);
	while(1)
	{ 
		//printf("slysim/n/r");
		uint8_t recv;
		/* Recv character */
		recv = USART_Receive();
		/* Send back the character */
		USART_Transmist(recv);
	}
	return 0;
}

