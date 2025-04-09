/*
 * Timers.c
 *
 * Created: 19.2.2018 10:30:50
 *  Author: Krajsa
 */ 

#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "Timers.h"
#include "../makra.h"

void Timer0_cmp_start(uint8_t porovnani){
	
	cli();          // zakazat preruseni
	TCCR0A = 0;     // vycistit kontrolni registry
	TCCR0B = 0; //
	TIMSK0 = 0;    //
	// nastavit hodnotu pro porovnani
	OCR0A = porovnani;
	// CTC mod:
	TCCR0A |= (1 << WGM01);
	// 1024 preddelicka:
	//TCCR0B |= (1 << CS00);
	//TCCR0B |= (1 << CS02);
	TCCR0B |= PRESCALE;
	// povolit preruseni:
	TIMSK0 |= (1 << OCIE0A);
	//vystup na pin OC0A, toggle
	TCCR0A |= (1 << COM0A0);
	sei();          // povolit globalni preruseni
}

void Timer0_ovf_start(){
	cli();          // zakazat preruseni
	TCCR0A = 0;     // vycistit kontrolni registry
	TCCR0B = 0;
	TIMSK0 = 0;     //
	//Nastav_timer(mils);
	// nastavit pocatecni hodnotu
	//TCNT0=255-Nastav_timer(mils);
	// 1024 preddelicka:
	TCCR0B |= PRESCALE;
	//vystup na pin OC0A, toggle
	TCCR0A |= (1 << COM0A0);
	// povolit preruseni:
	TIMSK0 |= (1 << TOIE0);
	sei(); // povolit globalni preruseni
}

void Timer0_fastpwm_start(uint8_t strida){
	
	cli();          // zakazat preruseni
	TCCR0A = 0;     // vycistit kontrolni registry
	TCCR0B = 0;  
	TIMSK0 = 0;   //
	// nastavit hodnotu pro PWM  
	OCR0A = (255*strida)/100;
	// fastpwm mod:
	TCCR0A |= (1 << WGM01);
	TCCR0A |= (1 << WGM00);
	// 1024 preddelicka:
	TCCR0B |= PRESCALE;
	// povolit preruseni:
	TIMSK0 |= (1 << TOIE0);
	//vystup na pin OC0A
	TCCR0A |= (1 << COM0A1);
	
	sei();          // povolit globalni preruseni
}

void Timer0Stop(){
	TCCR0B=0;
}

void Timer1_cmp_A_start(uint16_t porovnani){
	
	cli();          // zakazat preruseni
	TCCR1A = 0;     // vycistit kontrolni registry
	TCCR1B = 0; //
	TIMSK1 = 0;    //
	// nastavit hodnotu pro porovnani
	OCR1A = porovnani;
	// CTC mod:
	//TCCR1A |= (1 << WGM11);
	TCCR1B |=(1<<WGM12);
	// 1024 preddelicka:
	//TCCR0B |= (1 << CS10);
	//TCCR0B |= (1 << CS12);
	TCCR1B |= PRESCALE;
	// povolit preruseni:
	TIMSK1 |= (1 << OCIE1A);
	//vystup na pin OC1A, toggle
	TCCR1A |= (1 << COM1A0);
	sei();          // povolit globalni preruseni
}
void Timer1_cmp_B_start(uint16_t porovnani){
	
	cli();          // zakazat preruseni
	TCCR1A = 0;     // vycistit kontrolni registry
	TCCR1B = 0; //
	TIMSK1 = 0;    //
	// nastavit hodnotu pro porovnani
	OCR1B = porovnani;
	//OCR1A = porovnani+1;//Procpak?
	// CTC mod:
	//TCCR1A |= (1 << WGM11);
	// 1024 preddelicka:
	//TCCR0B |= (1 << CS10);
	//TCCR0B |= (1 << CS12);
	TCCR1B |= PRESCALE;
	// povolit preruseni:
	TIMSK1 |= (1 << OCIE1B);
	//vystup na pin OC1A, toggle
	//TCCR1A |= (1 << COM1A0);
	sei();          // povolit globalni preruseni
}
//drobna zmena, umoznime zacit citat jinou hodnotu. citame zbytek, tedy registr bude MAX-zbytek  
void Timer1_ovf_start(uint16_t zbytek){
	cli();          // zakazat preruseni
	TCCR1A = 0;     // vycistit kontrolni registry
	TCCR1B = 0;
	TIMSK1 = 0;     //
	//Nastav_timer(mils);
	// nastavit pocatecni hodnotu
	TCNT1=65535-zbytek;
	// 1024 preddelicka:
	TCCR1B |= PRESCALE;
	//vystup na pin OC0A, toggle
	//TCCR1A |= (1 << COM1A0);
	// povolit preruseni:
	TIMSK1 |= (1 << TOIE1);
	sei(); // povolit globalni preruseni
}	  

void Timer1_fastpwm_start(uint8_t strida){
	cli();          // zakazat preruseni
	TCCR1A = 0;     // vycistit kontrolni registry
	TCCR1B = 0;  
	TIMSK1 = 0;   //
	// nastavit hodnotu pro PWM  
	OCR1A = (255*strida)/100;
	// fastpwm mod:
	TCCR1A |= (1 << WGM11);
	TCCR1A |= (1 << WGM10);
	// 1024 preddelicka:
	TCCR1B |= PRESCALE;
	// povolit preruseni:
	TIMSK1 |= (1 << TOIE1);
	//vystup na pin OC0A
	TCCR1A |= (1 << COM1A1);
	
	sei();          // povolit globalni preruseni
}

void Timer1_Stop(){
	TCCR1B=0;
}
void Timer2_cmp_start(uint8_t porovnani){
	
	cli();          // zakazat preruseni
	TCCR2A = 0;     // vycistit kontrolni registry
	TCCR2B = 0; //
	TIMSK2 = 0;    //
	// nastavit hodnotu pro porovnani
	OCR2A = porovnani;
	// CTC mod:
	TCCR2A |= (1 << WGM11);
	// 1024 preddelicka:
	//TCCR0B |= (1 << CS10);
	//TCCR0B |= (1 << CS12);
	TCCR2B |= 5;
	// povolit preruseni:
	TIMSK2 |= (1 << OCIE2A);
	//vystup na pin OC1A, toggle
	TCCR2A |= (1 << COM2A0);
	sei();          // povolit globalni preruseni
}

void Timer2_ovf_start(){
	cli();          // zakazat preruseni
	TCCR2A = 0;     // vycistit kontrolni registry
	TCCR2B = 0;
	TIMSK2 = 0;     //
	//Nastav_timer(mils);
	// nastavit pocatecni hodnotu
	//TCNT0=255-Nastav_timer(mils);
	// 1024 preddelicka:
	TCCR2B |= 5;
	//vystup na pin OC0A, toggle
	TCCR2A |= (1 << COM2A0);
	// povolit preruseni:
	TIMSK2 |= (1 << TOIE2);
	sei(); // povolit globalni preruseni
}

void Timer2_fastpwm_start(uint8_t strida){
	cli();          // zakazat preruseni
	TCCR2A = 0;     // vycistit kontrolni registry
	TCCR2B = 0;
	TIMSK2 = 0;   //
	// nastavit hodnotu pro PWM
	OCR2A = (255*strida)/100;
	// fastpwm mod:
	TCCR2A |= (1 << WGM21);
	TCCR2A |= (1 << WGM20);
	// 1024 preddelicka:
	TCCR2B |= 5;
	// povolit preruseni:
	TIMSK2 |= (1 << TOIE2);
	//vystup na pin OC2A
	TCCR2A |= (1 << COM2A1);
	
	sei();          // povolit globalni preruseni
}

void Timer2_Stop(){
	TCCR2B=0;
}

void Timer3_cmp_start(uint16_t porovnani){
	
	cli();          // zakazat preruseni
	TCCR3A = 0;     // vycistit kontrolni registry
	TCCR3B = 0; //
	TIMSK3 = 0;    //
	// nastavit hodnotu pro porovnani
	OCR3A = porovnani;
	// CTC mod:
	TCCR3A |= (1 << WGM31);
	// 1024 preddelicka:
	//TCCR0B |= (1 << CS10);
	//TCCR0B |= (1 << CS12);
	TCCR3B |= PRESCALE;
	// povolit preruseni:
	TIMSK3 |= (1 << OCIE3A);
	//vystup na pin OC1A, toggle
	//TCCR3A |= (1 << COM3A0);
	sei();          // povolit globalni preruseni
}
void Timer3_ovf_start(uint16_t zbytek){
	cli();          // zakazat preruseni
	TCCR3A = 0;     // vycistit kontrolni registry
	TCCR3B = 0;
	TIMSK3 = 0;     //
	//Nastav_timer(mils);
	// nastavit pocatecni hodnotu
	TCNT3=65535-zbytek;
	// 1024 preddelicka:
	TCCR3B |= PRESCALE;
	//vystup na pin OC0A, toggle
	//TCCR1A |= (1 << COM1A0);
	// povolit preruseni:
	TIMSK3 |= (1 << TOIE3);
	sei(); // povolit globalni preruseni
}
void Timer3_Stop(){
	TCCR3B=0;
}