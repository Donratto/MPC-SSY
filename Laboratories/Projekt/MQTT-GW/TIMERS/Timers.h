/*
 * Timers.h
 *
 * Created: 19.2.2018 10:30:37
 *  Author: Krajsa
 */ 


#ifndef TIMERS_H_
#define TIMERS_H_


//PRESCALE pro TIMER0,1,3,4,5, je potreba vybrat vzdy PRESCALE a PRESCALE VALUE. NE por TIMER2
#define PRESCALE 5 //1024
#define PRESCALE_VALUE 1024
//#define PRESCALE 4 //256
//#define PRESCALE_VALUE 256
//#define PRESCALE 3 //64
//#define PRESCALE_VALUE 64
//#define PRESCALE 2 //8
//#define PRESCALE_VALUE 8
//#define PRESCALE 1 //1
//#define PRESCALE_VALUE 1



void Timer0_cmp_start(uint8_t porovnani);
void Timer0_ovf_start();
void Timer0_fastpwm_start(uint8_t strida);
void Timer0_Stop();

void Timer1_cmp_B_start(uint16_t porovnani);
void Timer1_cmp_A_start(uint16_t porovnani);
void Timer1_ovf_start(uint16_t zbytek);
void Timer1_fastpwm_start(uint8_t strida);
void Timer1_Stop();

void Timer2_cmp_start(uint8_t porovnani);
void Timer2_ovf_start();
void Timer2_fastpwm_start(uint8_t strida);
void Timer2_Stop();

void Timer3_cmp_start(uint16_t porovnani);
void Timer3_ovf_start(uint16_t zbytek);
void Timer3_Stop();


#endif /* TIMERS_H_ */