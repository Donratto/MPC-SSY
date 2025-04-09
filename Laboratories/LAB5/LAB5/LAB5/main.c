/*
 * LWM_MSSY.c
 *
 * Created: 6.4.2017 15:42:46
 * Author : Krajsa
 */ 

#include <avr/io.h>
/*- Includes ---------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "config.h"
#include "hal.h"
#include "phy.h"
#include "sys.h"
#include "nwk.h"
#include "sysTimer.h"
#include "halBoard.h"
#include "halUart.h"
#include "main.h"

/*- Definitions ------------------------------------------------------------*/
#ifdef NWK_ENABLE_SECURITY
#define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
#else
#define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE
#endif

/*- Types ------------------------------------------------------------------*/
typedef enum AppState_t
{
	APP_STATE_INITIAL,
	APP_STATE_IDLE,
} AppState_t;

/*- Prototypes -------------------------------------------------------------*/
static void appSendData(void);

/*- Variables --------------------------------------------------------------*/
static AppState_t appState = APP_STATE_INITIAL;
static SYS_Timer_t appTimer;
static NWK_DataReq_t appDataReq;
static bool appDataReqBusy = false;
static uint8_t appDataReqBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBufferPtr = 0;

/*- Implementations --------------------------------------------------------*/




/*************************************************************************//**
*****************************************************************************/
static void appDataConf(NWK_DataReq_t *req)
{
appDataReqBusy = false;
(void)req;
}

/*************************************************************************//**
*****************************************************************************/
static void appSendData(void)
{
if (appDataReqBusy || 0 == appUartBufferPtr)
return;

memcpy(appDataReqBuffer, appUartBuffer, appUartBufferPtr);

appDataReq.dstAddr = 1-APP_ADDR;
appDataReq.dstEndpoint = APP_ENDPOINT;
appDataReq.srcEndpoint = APP_ENDPOINT;
appDataReq.options = NWK_OPT_ENABLE_SECURITY;
appDataReq.data = appDataReqBuffer;
appDataReq.size = appUartBufferPtr;
appDataReq.confirm = appDataConf;
NWK_DataReq(&appDataReq);

appUartBufferPtr = 0;
appDataReqBusy = true;
}
/*- Implementations --------------------------------------------------------*/
static void appSendDataRecv(void)
{
	static char ack = 06;
	appDataReq.dstAddr = 1-APP_ADDR;
	appDataReq.dstEndpoint = APP_ENDPOINT;
	appDataReq.srcEndpoint = APP_ENDPOINT;
	appDataReq.options = NWK_OPT_ENABLE_SECURITY;
	appDataReq.data = &ack;
	appDataReq.size = sizeof(char);
	appDataReq.confirm = appDataConf;
	NWK_DataReq(&appDataReq);

	appUartBufferPtr = 0;
	appDataReqBusy = true;
}

static void appSendDataTemp(void)
{
	static char ack = 59;
	appDataReq.dstAddr = 1-APP_ADDR;
	appDataReq.dstEndpoint = APP_ENDPOINT;
	appDataReq.srcEndpoint = APP_ENDPOINT;
	appDataReq.options = NWK_OPT_ENABLE_SECURITY;
	appDataReq.data = &ack;
	appDataReq.size = sizeof(char);
	appDataReq.confirm = appDataConf;
	NWK_DataReq(&appDataReq);

	appUartBufferPtr = 0;
	appDataReqBusy = true;
}


/*************************************************************************//**
*****************************************************************************/
void HAL_UartBytesReceived(uint16_t bytes)
{
for (uint16_t i = 0; i < bytes; i++)
{
uint8_t byte = HAL_UartReadByte();

if (appUartBufferPtr == sizeof(appUartBuffer))
appSendData();

if (appUartBufferPtr < sizeof(appUartBuffer))
appUartBuffer[appUartBufferPtr++] = byte;
}

SYS_TimerStop(&appTimer);
SYS_TimerStart(&appTimer);
}

/*************************************************************************//**
*****************************************************************************/
static void appTimerHandler(SYS_Timer_t *timer)
{
appSendData();
(void)timer;
}

/*************************************************************************//**
*****************************************************************************/
/*
static bool appDataInd(NWK_DataInd_t *ind)
{
for (uint8_t i = 0; i < ind->size; i++)
HAL_UartWriteByte(ind->data[i]);
return true;
}
*/

static bool appDataInd(NWK_DataInd_t *ind)
{
	if ((ind->data[0] == 06 && ind->size == 1 ) || (ind->data[0] == 79 && ind->data[1] == 75 && ind->size == 2 )) {
		HAL_UartWriteByte('|');
		/*
		HAL_UartWriteByte('s');
		HAL_UartWriteByte('r');
		HAL_UartWriteByte('c');
		HAL_UartWriteByte(':');
		
		uint8_t array[2]={ ind->dstAddr & 0xff, ind->dstAddr >> 8 };
		HAL_UartWriteByte(array[1]);
		HAL_UartWriteByte(array[0]);
		HAL_UartWriteByte(ind->rssi);
		HAL_UartWriteByte(ind->dstAddr >> 8 );
		HAL_UartWriteByte(ind->dstAddr & 0xff);
		*/
		} else {
		for (uint8_t i = 0; i < ind->size; i++) {
			HAL_UartWriteByte(ind->data[i]);
		}
		appSendDataRecv();
	}
	return true;
}

static SYS_Timer_t appTimerMOJ;
static void appTimerHandlerMOJ(SYS_Timer_t *timer)
{
	// handle timer event
	appSendDataTemp();
	
	//If (timeToStop)
	//SYS_TimerStop(timer);
}
static void startTimerMOJ(void)
{
	appTimerMOJ.interval = 2000;
	appTimerMOJ.mode = SYS_TIMER_PERIODIC_MODE;
	appTimerMOJ.handler = appTimerHandlerMOJ;
	SYS_TimerStart(&appTimerMOJ);
}


/*************************************************************************//**
*****************************************************************************/
static void appInit(void)
{
NWK_SetAddr(APP_ADDR);
NWK_SetPanId(APP_PANID);
PHY_SetChannel(APP_CHANNEL);
#ifdef PHY_AT86RF212
PHY_SetBand(APP_BAND);
PHY_SetModulation(APP_MODULATION);
#endif
PHY_SetRxState(true);

NWK_OpenEndpoint(APP_ENDPOINT, appDataInd);

HAL_BoardInit();

appTimer.interval = APP_FLUSH_TIMER_INTERVAL;
appTimer.mode = SYS_TIMER_INTERVAL_MODE;
appTimer.handler = appTimerHandler;
}

/*************************************************************************//**
*****************************************************************************/
static void APP_TaskHandler(void)
{
switch (appState)
{
case APP_STATE_INITIAL:
{
appInit();
appState = APP_STATE_IDLE;
} break;

case APP_STATE_IDLE:
break;

default:
break;
}
}

/*************************************************************************//**
*****************************************************************************/
int main(void)
{
SYS_Init();
HAL_UartInit(38400);
HAL_UartWriteByte('a');
startTimerMOJ();

while (1)
{
SYS_TaskHandler();
HAL_UartTaskHandler();
APP_TaskHandler();
}
}
