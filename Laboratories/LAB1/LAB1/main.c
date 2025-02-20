/*
 * LAB1.c
 *
 * Created: 02.02.2020 9:01:38
 * Author : Ondra
 */ 

/************************************************************************/
/* INCLUDE                                                              */
/************************************************************************/
#include <avr/io.h>
#include <util/delay.h>
#include "libs/libprintfuart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

// F_CPU definovano primo v projektu!!! Debug->Properties->Toolchain->Symbols

#define ODECET
//#define ODECET
#define KONST 2

#define DIRECTION_UP 
#define DIRECTION_DOWN 

#define NORMAL_CASE 
//#define UPPER_CASE 
#define PROGRAM_ERROR "PROGRAM ERROR"



/************************************************************************/
/* VARIABLES                                                            */
/************************************************************************/

//musime vytvorit soubor pro STDOUT
FILE uart_str = FDEV_SETUP_STREAM(printCHAR, NULL, _FDEV_SETUP_RW);

int a = 10;
int b = 10;
unsigned char c = 255;
unsigned char d = 255;
uint16_t e;
int f = 24;
int g = 200;
char h[] = "Hodnota=";
char str1[80];
char str2[80];
char buffer [sizeof(int)*8+1];
char abc[80];


/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void board_init();

int generateField();

void capsLetters();

void printField();



/************************************************************************/
/* FUNCTIONS                                                            */
/************************************************************************/
void board_init(){
	UART_init(38400); //nastaveni rychlosti UARTu, 38400b/s
	stdout = &uart_str; //presmerovani STDOUT
}

int generateField(){
	int abcStart;
	
	#ifdef UPPER_CASE
		abcStart = 65;
	#endif
	#ifdef NORMAL_CASE
		abcStart = 97;
	#endif
	/*
	#ifndef (UPPER_CASE & NORMAL_CASE)
		printf(PROGRAM_ERROR);
		return 1;
	#endif
	*/
	int i;
	for (i = 0, i < 26, i++)
	{
		abc[i] = abcStart+i;
	}
	return 0;
}

void capsLetters();

void printField();


int main(void)
{ 	
	board_init();
	_delay_ms(1000);
	
	printf("Ukol c. 4\n\r");
	printf("a= %d \n\r", a);
	printf("b= %d \n\r", b);
	printf("Hello word\n\r");
	#ifndef ODECET
	a = a - KONST;
	#endif
	#ifndef ODECETT
	b = b - KONST;
	#endif
	printf("a= %d \n\r", a);
	printf("b= %d \n\r", b);
	
	printf("Ukol c. 5\n\r");
	e = (int)c + d; // e = 255 + 255 = 510
	printf("e jest: %d \n\r",e);
	
	printf("Ukol c. 6\n\r");
	f = f >> 3; // ~ f = f/(2)^3 = 24/8 = 3
	printf("f = f >> 3: %d \n\r", f);
	f = f - 1; // f = 3 - 1 = 2
	printf("f = f - 1: %d \n\r", f);
	f = f & 0x2; // f = 0b10 & 0b10 = 0b10 = 2 
	printf("f = f & 0x2; %d \n\r", f);
	
	printf("Ukol c. 7\n\r");
	strcpy(str1, h);
	strcat(str1, itoa(g, buffer,10));
	printf("%s \n\r",str1);
	sprintf(str2, "%s%d",h,g);
	printf("%s \n\r",str2);
	
	printf("Ukol c. 8\n\r");
	generateField();
	
	
    _delay_ms(1000);
	int i=0;
    while (1) 
    {
	_delay_ms(10000);
	i++;
	printf("Test x = %d \n\r", i);
    }
}

