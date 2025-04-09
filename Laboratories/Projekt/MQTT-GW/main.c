/*
* LAB2.c
*
* Created: 10.2.2020 13:05:19
* Author : Krajsa
*/

// LAB 4

/************************************************************************/
/* INCLUDE                                                              */
/************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "UART_lib/uart.h"
#include "ADC/ADC.h"
#include "TIMERS/Timers.h"
#include "makra.h"
#include <stdio.h>
#include "I2Clib/i2clib.h"
#include "at30tse758.h"
/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

// F_CPU definovano primo v projektu!!!


/************************************************************************/
/* VARIABLES                                                            */
/************************************************************************/
volatile uint8_t recv; //prijaty znak z preruseni
volatile uint8_t state;
uint8_t skore_tlacitka=0;
uint8_t generuj2hz=0;
uint8_t automat=0;
FILE uart_str = FDEV_SETUP_STREAM(printCHAR, NULL, _FDEV_SETUP_RW);
/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void board_init();
void menu_level1();
void osetreni_stavu1();
void LED2blink();

/************************************************************************/
/* FUNCTIONS                                                            */
/************************************************************************/
void board_init(){
	sbi(DDRB,DDRB6); //PORTB6 vystupni
	sbi(DDRB,DDRB5); //PORTB5 vystupni
	sbi(DDRB,DDRB4); //PORTB4 vystupni
	sbi(DDRE,DDRE3);//PORTE3 vystupni
	Timer2_fastpwm_start(0);
	LED2ON; //rozsviti LED2
	sbi(DDRE,DDRE5); //PORTE - Button - vstupni
	sbi(PORTE,PORTE5); //pull-up
	UART_init(38400,1); //nastaveni rychlosti UARTu, 38400b/s, povoleni preruseni
	ADC_Init(4,2);
	i2cInit();
	at30_setPrecision(2);
	stdout = &uart_str;
	sei(); //povoleni globalniho preruseni - ALE BACHA, musí být obsluhy preruseni!!!!!
	
}
void menu_level1(){
	UART_SendString("\033[1;31;40m"); //cervene pismo na cernem pozadi
	UART_SendString("MENU: \r\n");
	UART_SendString("0 - Konec programu \r\n");
	UART_SendString("1 - Mala abeceda \r\n");
	UART_SendString("2 - Velka abeceda \r\n");
	UART_SendString("3 - Blikne LED 2 \r\n");
	UART_SendString("4 - Precist osvetleni \r\n");
	UART_SendString("5 - Start/Stop 2Hz \r\n");
	UART_SendString("+ - Zvysi jas LED0 \r\n");
	UART_SendString("- - Snizi jas LED0 \r\n");
	UART_SendString("A - Start/Stop Automatickeho rizeni \r\n");
	UART_SendString("T - Teplota z I2C cidla \r\n");
	UART_SendString("Stisk Button1 - jako volba 3\r\n");
	UART_SendString("\033[0;37;40m"); //bile pismo na cernem pozadi
}
void osetreni_stavu1(){
	switch (recv)  //podle prijateho znaku se rozhoduji co udelam
	{
		case '0': //pokud je prijaty znak 0
		UART_SendString("\r\n Nula - konec...\r\n");
		//while(1);
		break;
		case '1':
		UART_SendString("mala abeceda\r\n");
		break;
		case '2':
		UART_SendString("VELKA ABECEDA\r\n");
		break;
		case '3':
		UART_SendString("Blika LED2\r\n"); //navrat na zacatek radku + dva nove radky
		LED2blink();
		UART_SendString("\033[2J"); //smaze obrazovku
		menu_level1();
		break;
		
		case '4':
		//UART_SendString("Hodnota z cidla osvetleni je: \r\n"); //navrat na zacatek radku + dva nove radky
		printf("Hodnota z AD prevodniku je %d \r\n" ,ADC_get(3));
		menu_level1();
		break;
		case '5':
		//UART_SendString("Hodnota z cidla osvetleni je: \r\n"); //navrat na zacatek radku + dva nove radky
		if(generuj2hz){
			printf("STOP 2Hz na PB5 \r\n");
			Timer1_Stop();
			generuj2hz=0;
		}
		else{
			printf("START 2Hz na PB5 \r\n");
			Timer1_cmp_A_start(1956);
			generuj2hz=1;
		}
		//menu_level1();
		break;
		case '+':
		printf("Zvysuji jas \r\n");
		//muzeme menit primo hodnotu k porovnani v registru OCR2A, nemusime volat znovu start timeru atd...
		if(OCR2A<=10){
			OCR2A=0;
		}
		else {
			OCR2A-=10;
		}
		break;
		case '-':
		printf("Snizuji jas \r\n");
		if(OCR2A>=244){
			OCR2A=255;
		}
		else {
			OCR2A+=10;
		}
		break;
		
		case 'A':
		if(automat){
			printf("STOP auto rizeni LED0 \r\n");
			automat=0;
			Timer1_Stop();
		}
		else{
			printf("START auto rizeni LED0 \r\n");
			automat=1;
			Timer1_cmp_B_start(200);
			ADC_Start_per(4,2,3,5); //4 - preddelicka, 2-reference, 3 -kanal, 5 - zdroj spousteni = Timer1 compare B
		}
		//menu_level1();
		break;
		
		case 'T':
		UART_SendString("Merime teplotu\r\n");
		printf("vysledek=%f °C \n\r",at30_readTemp()); //funkce z at30tse758.c

		break;
		
		default:
		UART_SendString("Neznamy povel, mozne hodnoty jsou: \n\r");
		menu_level1();
	}
	state=0; // vratim se ke stavu 0;
}
void LED2blink(){
	for(int i=0;i<6;i++){
		LED2CHANGE; //makro, meni stav bitu...
		_delay_ms(500);
	}
}

int main(void)
{
	board_init(); //init periferii
	menu_level1();//prvni vypsani menu
	
	/* Replace with your application code */
	while (1)
	{
		if (state==1) osetreni_stavu1(); //jen kontroluji, jestli se zmeni stav...
		if(!tbi(PINE,PINE5)){
			skore_tlacitka++;
		}
		else{
			skore_tlacitka=0;
		}
		if(skore_tlacitka>10){
			UART_SendString("Blika LED2\r\n"); //navrat na zacatek radku + dva nove radky
			LED2blink();
			UART_SendString("\033[2J"); //smaze obrazovku
			menu_level1();
			skore_tlacitka=0;
		}
	}
}
/************************************************************************/
/* PRERUSENI                                                            */
/************************************************************************/
ISR(USART1_RX_vect)
{
	recv = UART_GetChar(); //nactu znak funkci...
	UART_SendChar(recv);//echo, neni nutne
	state=1; //dam programu vedet, ze se neco stalo, on si to osetri
}
ISR (TIMER1_COMPA_vect)
{
	LED3CHANGE;}ISR (TIMER1_COMPB_vect)
{
	//bud vypnout interrupt u periferie, nebo musi byt alespon prazne osetreni}ISR (TIMER2_OVF_vect)
{
	//bud vypnout interrupt u periferie, nebo musi byt alespon prazne osetreni}ISR (ADC_vect)
{
	TCNT1=0;// doslo ke spustni AD prevodu=cinac nacital do OCR1B a bude pokracovat do 65553, ale to nechceme...
	OCR2A=255-(ADC/4);}