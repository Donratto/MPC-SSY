/*
 * uart.h
 *
 * Created: 10.2.2020 13:26:45
 *  Author: Krajsa
 */ 
#include <stdio.h>

#ifndef UART_H_
#define UART_H_
void UART_init(uint16_t Baudrate,uint8_t interrupt);
void UART_SendChar(uint8_t data);
uint8_t UART_GetChar( void );
void UART_SendString(char *text);
int printCHAR(char character, FILE *stream);
#endif /* UART_H_ */