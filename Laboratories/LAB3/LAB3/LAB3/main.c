/*
 * LAB2-a.c
 *
 * Created: 15.02.2023 12:18:20
 * Author : Ondra
 */ 
/************************************************************************/
/* INCLUDE                                                              */
/************************************************************************/
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "makra.h"
#include "uart/uart.h"
/************************************************************************/
/* DEFINES                                                              */
/************************************************************************/

// F_CPU definovano primo v projektu!!!Defined in project properties

/************************************************************************/
/* VARIABLES                                                            */
/************************************************************************/

FILE uart_str = FDEV_SETUP_STREAM(printCHAR, NULL, _FDEV_SETUP_RW);//soubor pro stdout

volatile int pruchod=0;  //
volatile uint8_t klik=0; //
volatile char recv;
volatile uint8_t state=0;

char text[]="label"; //= 'l''a''b''e''l''/0'

uint8_t strida = 50;

/************************************************************************/
/* PROTOTYPES                                                           */
/************************************************************************/

void board_init();
void menu1();
void prvniUkoly();


void Timer1_tct_start(uint16_t porovnej);
void Timer2_fastpwm_start(uint8_t strida);

/************************************************************************/
/* FUNCTIONS                                                            */
/************************************************************************/
void board_init(){
	cli(); //disable interrupts - defaultne je sice vyple, ale pro jistotu
	
	UART_init(38400); //baudrate 38400b/s
	
	UCSR1B |= (1 << RXCIE1);// UART receive interrupt enable
	//sbi(UCSR1B,RXCIE1);
	stdout = &uart_str; //standartní výstup/std output
	
	//LED0 vystup/output
	sbi(DDRB,4);
	sbi(PORTB,4); //turn the LED0 down

	//LED1
	sbiX(DDRB,5);
	sbi(PORTB,5);
	
	//Tlacitka vstup/buttons as input
	cbi(DDRE,4);
	//Tlacitka pull-up
	sbi(PORTE,4);
	
	sbi(EIMSK,4); //povolit preruseni INT5 - tlacitko button0 /INT5 enable
	sbi(EICRB,ISC41);//nastupna hrana /rising edge
	sbi(EICRB,ISC40);//nastupna hrana
	
	sei(); // enable interrupts
}

void menu1(){
	UART_SendChar(27);//escape
	UART_SendString("[2J");//clear and home
	UART_SendChar(27);//escape
	UART_SendString("[0;32;40m");//barva pozadi a textu
	
	printf("-------------------------------------\n\r");
	printf("MENU: \n\r");
	printf("0 - Konec\n\r");
	printf("1 - Rozsvit LED1 \n\r");
	printf("2 - Zhasni LED1 \n\r");
	printf("3 - Vypis intezitu \n\r");
	printf("4 - Vypis abecedu \n\r");
	printf("5 - Blikej 2Hz \n\r");
	printf("'+'/'-' - Zmena intezity LED0 \n\r");
	printf("-------------------------------------\n\r");
	
	UART_SendChar(27);//escape
	UART_SendString("[0;37;40m");//reset
}

void prvniUkoly(){
	DDRB=0b01110000;
	PORTB=0b01110000;
	_delay_ms(500);
	PORTB=0;
	_delay_ms(500);
	
	PORTB=112;
	_delay_ms(500);
	PORTB=0;
	_delay_ms(500);
	
	PORTB=0x70;
	_delay_ms(500);
	PORTB=0;
	_delay_ms(500);
	
	PORTB=(7<<4);
	_delay_ms(500);
	PORTB=0;
	_delay_ms(500);
	
	DDRB=0;
	PORTB=0;
	
}

void stopFun()
{
	TCCR1A = 0; //nulovani
	TCCR1B = 0; //nulovani
	TCCR1C = 0; //nulovani
	
	LED0OFF;
	LED1OFF;
	LED2OFF;
	LED3OFF;
}

void Timer1_tct_start(uint16_t porovnej)
{
	// predelicka=1024 a OCR1A=1952,125
	cli(); //zakazani preruseni
	TCCR1A = 0; //nulovani
	TCCR1B = 0; //nulovani
	TCCR1C = 0; //nulovani
	TIMSK1 = 0; //nulovani
	
	OCR1A = porovnej ;
	
	TCCR1B |= (1 << WGM12); // TCT mod
	// preddelicka N = 1024
	TCCR1B |= (1 << CS12); 
	TCCR1B |= (1 << CS10);
	
	//TIMSK1 |= (1 << OCIE1A); //povoluje preruseni
	
	TCCR1A |= (1 << COM1A0); //vystup na pin OC1A, t o g g l e
	
	sei();
}

void Timer2_fastpwm_start(uint8_t strida)
{
	cli(); //zakazani preruseni
	TCCR2A = 0; //nulovani
	TCCR2B = 0; //nulovani
	TIMSK2 = 0; //nulovani
	
	//OCR2A = (255*strida)/100;
	
	TCCR2B |= (1 << WGM21); // fastpwm mod
	TCCR2B |= (1 << WGM20); // fastpwm mod
	
	// preddelicka N = 1024
	/*
	TCCR2B |= (1 << CS22); 
	TCCR2B |= (1 << CS21);
	TCCR2B |= (1 << CS20);
	*/
	TCCR2B |= 7;
	TIMSK2 |= (1 << TOIE2); //povoluje preruseni
	
	TCCR2A |= (1 << COM2A1);
	
	sei();
}

/************************************************************************/
/* MAIN                                                                 */
/************************************************************************/
int main(void)
{ 	
	prvniUkoly(); // jen blikani LED, 
	
	board_init();
	_delay_ms(100);
	menu1();
	_delay_ms(100);
	
	int i=0;
	while (1)
	{
		if (state==1){
			switch (recv)  			{
				case '0':
				stopFun();
				printf("\r\nKoncim program... \r\n");
				menu1();
				break;
				
				case '1':
				LED1ON;
				printf("\r\n");
				break;
				case '2':
				printf("\r\n");
				LED1OFF;
				break;
				case '3':
				printf("\r\n");
				printf("strida est: %d", strida);
				printf("\n\r");
				break;
				case '4':
				printf("\r\n");
				for (int i=65;i<90;i++)
				{
					UART_SendChar(i);
				}
				printf("\n\r");
				break;
				case '5':
				printf("\r\n");
				Timer1_tct_start(1952.125);
				break;
				case '+':
				printf("\r\n");
				if (strida == 100)
				{
					printf("nelze, dosazena max strida: %d", strida);
					break;
				}
				Timer2_fastpwm_start(strida++);
				break;
				case '-':
				printf("\r\n");
				if (strida == 0)
				{
					printf("nelze, dosazena min strida: %d", strida);
					break;
				}
				Timer2_fastpwm_start(strida--);
				break;
				
				default:
				printf("\r\nNeznamy prikaz.. \r\n");
				menu1();
				//code - unknown command
				
			}
			state=0;
		}
		_delay_ms(100);
		i++;
		//printf("Test x = %d \n\r", i);
		//printf("Klik = %d \n\r", klik);
	}
	
}

/************************************************************************/
/* INTERRUPTS                                                           */
/************************************************************************/

ISR(INT4_vect){
	//	"debounce", hodne spatny debounce....
	cbi(EIMSK,INT4);//zakazeme preruseni INT4
	
	_delay_ms(200); // - zpozdeni - jen cekame az odezni zakmity
	klik++;
	LED0CHANGE;
	sbi(EIFR,INTF4);// nakonci nastavime prizkak, ze doslo k preruseni na 1 - tim se vynuluje (ano, je to zvlastni, ale je to tak)
	sbi(EIMSK,INT4);//Povolime preruseni INT4
}

ISR(USART1_RX_vect)
{
	recv = UART_GetChar(); // zbytecné
	//recv = UDR1;//nejspíš lepší
	UART_SendChar(recv);//echo, pro jistotu...Jinak nebudeme videt, co piseme
	state=1; //state machine
}

ISR (TIMER1_COMPA_vect)
{
	LED1CHANGE; 
}


ISR (TIMER2_COMPA_vect)
{
	LED0CHANGE; 
}

ISR (TIMER2_OVF_vect)
{
	OCR2A = (255*strida)/100;
}