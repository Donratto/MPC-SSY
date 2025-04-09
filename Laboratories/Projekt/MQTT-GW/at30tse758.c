/*
* at30tse758.c
*
* Created: 01.03.2020 20:14:02
*  Author: Ondra
*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "makra.h"
#include <stdio.h>
#include "I2Clib/i2clib.h"
#include "at30tse758.h"
#include "UART_lib/uart.h"

uint8_t at30_setPrecision(uint8_t prec){
	uint_fast16_t config_register=0;
	config_register |= (uint16_t)(prec << R0);
	i2cStart();
	i2cWrite(TempSensorAddrW);
	if (i2cGetStatus() != 0x18) {
		UART_SendString("Error 18\n\r");
		return 0;
	};
	i2cWrite(Temp_configRegister);
	if (i2cGetStatus() != 0x28) {
		UART_SendString("Error 28\n\r");
		return 0;
	};
	i2cWrite((uint8_t)(config_register>>8));
	if (i2cGetStatus() != 0x28) {
		UART_SendString("Error 28\n\r");
		return 0;
	};
	i2cWrite((uint8_t)(config_register));
	if (i2cGetStatus() != 0x28) {
		UART_SendString("Error 28\n\r");
		return 0;
	};
	i2cStop();
	return 1;
}
uint8_t at30_readPrecision(void){
	uint_fast16_t config_register=0;
	uint8_t buffer[2];
	i2cStart();
	i2cWrite(TempSensorAddrW);
	if (i2cGetStatus() != 0x18) {
		UART_SendString("Error 18\n\r");
	};
	i2cWrite(Temp_configRegister);
	if (i2cGetStatus() != 0x28) {
		UART_SendString("Error 28\n\r");
	};
	i2cStop();
	
	i2cStart();
	i2cWrite(TempSensorAddrR);
	if (i2cGetStatus() != 0x40) {
		UART_SendString("Error 40\n\r");
		return 0;
	};
	buffer[0]=i2cReadACK();
	if (i2cGetStatus() != 0x50) {
		UART_SendString("Error 50\n\r");
		
	};
	buffer[1]=i2cReadNACK();
	if (i2cGetStatus() != 0x58) {
		UART_SendString("Error 58\n\r");
		
	};
	i2cStop();
	config_register= (buffer[0]<<8)|buffer[1];
	uint8_t prec = 0x03&(config_register>>R0);
	return prec;
	
}
float at30_readTemp(void){
	volatile uint8_t buffer[2];
	//float teplota;
	volatile int16_t teplotaTMP;
	
/************************************************************************/
/* Do Pointer registru nastavime co chceme cist - Temp registr          */
/************************************************************************/
	i2cStart();
	i2cWrite(TempSensorAddrW);
	if (i2cGetStatus() != 0x18) {
		UART_SendString("Error 18\n\r");
	};
	i2cWrite(Temp_tempRegister);
	if (i2cGetStatus() != 0x28) {
		UART_SendString("Error 28\n\r");
	};
	i2cStop();
/************************************************************************/
/* A cteme dany temp registr   start/adresa/cteme 2 byty                */
/************************************************************************/
	i2cStart();
	if (i2cGetStatus() != 0x08) {
		UART_SendString("Error 08\n\r");
	};
	//_delay_ms(50);
	i2cWrite(TempSensorAddrR);
	if (i2cGetStatus() != 0x40) {
		UART_SendString("Error 40\n\r");
	};
	
	buffer[0]=i2cReadACK();
	if (i2cGetStatus() != 0x50) {
		UART_SendString("Error 50\n\r");
		
	};
	buffer[1]=i2cReadNACK();
	if (i2cGetStatus() != 0x58) {
		UART_SendString("Error 58\n\r");
		
	};
	i2cStop();
	teplotaTMP=(buffer[0]<<8)|buffer[1];
	/*
	uint8_t sign=buffer[0]>>7;
	uint8_t precision=at30_readPrecision();
	if (precision==0){
		teplotaTMP=(buffer[0]<<8)|buffer[1];
		teplotaTMP &= (uint16_t)~(1 << 15);
		teplotaTMP=(teplotaTMP>>7);
		if (sign){
			teplotaTMP |= (uint16_t)(1 << 15);
		}
		return (float)teplotaTMP/2;
	}
	else if (precision==1){
		teplotaTMP=(buffer[0]<<8)|buffer[1];
		teplotaTMP &= (uint16_t)~(1 << 15);
		teplotaTMP=(teplotaTMP>>6);
		if (sign){
			teplotaTMP |= (uint16_t)(1 << 15);
		}
		return (float)teplotaTMP/4;
	}
	else if (precision==2){
		teplotaTMP=(buffer[0]<<8)|buffer[1];
		teplotaTMP &= (uint16_t)~(1 << 15);
		teplotaTMP=(teplotaTMP>>5);
		if (sign){
			teplotaTMP |= (uint16_t)(1 << 15);
		}
		return (float)teplotaTMP/8;
	}
	else if (precision==3){
		teplotaTMP=(buffer[0]<<8)|buffer[1];
		teplotaTMP &= (uint16_t)~(1 << 15);
		teplotaTMP=(teplotaTMP>>4);
		if (sign){
			teplotaTMP |= (uint16_t)(1 << 15);
		}
		return (float)teplotaTMP/16;
	}
	*/
	return (float)teplotaTMP/256;
}