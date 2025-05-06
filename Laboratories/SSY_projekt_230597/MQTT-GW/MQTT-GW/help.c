/* 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <compat/deprecated.h>  //sbi, cbi etc..
#include "avr/wdt.h" // WatchDog
#include <stdio.h>  // printf etc..
#include "uart_extd.h"
#include "spi.h"
#include "string.h"

#include "stdbool.h"
#include "Ethernet/socket.h"
#include "Ethernet/wizchip_conf.h"
#include "Application/loopback/loopback.h"
#include "Application/PING/ping.h"
#include "Internet/DNS/dns.h"
#include "Internet/MQTT/mqtt_interface.h"
#include "Internet/MQTT/MQTTClient.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "config.h"
#include "stack/hal/atmega256rfr2/inc/hal.h"
#include "stack/phy/atmega256rfr2/inc/phy.h"
#include "stack/sys/inc/sys.h"
#include "stack/nwk/inc/nwk.h"
#include "stack/sys/inc/sysTimer.h"
#include "stack/hal/drivers/atmega256rfr2/inc/halBoard.h"
#include "stack/hal/drivers/atmega256rfr2/inc/halUart.h"



/*- Definitions ------------------------------------------------------------*/
#define LOOPBACK_DATA_BUF_SIZE 512
//SPI CLOCK 4 Mhz
#define SPI_4_MHZ
#define SPRINTF(__S, FORMAT, args...) sprintf_P(__S, PSTR(FORMAT),##args)
#define PRINTF_EN 1
#if PRINTF_EN
#define PRINTF(FORMAT,args...) printf_P(PSTR(FORMAT),##args)
#else
#define PRINTF(...)
#endif
#define UART_BAUD_RATE      38400
#define DNS_INFO
#define TICK_PER_SEC 1000UL
// MQTT
#define SOCK_MQTT       2
// Receive Buffer
#define MQTT_BUFFER_SIZE	512     // 2048
// MQTT topics
#define PUBLISH "topic/ssy/test"
#define SUBSCRIBE "topic/ssy/test"
// MQTT end
//LWM
#ifdef NWK_ENABLE_SECURITY
#define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
#else
#define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE
#endif
// LWM end
#define ETH_MAX_BUF_SIZE 512
// DNS
#define SOCK_DNS       6
// DNS end


/*- Variables --------------------------------------------------------------*/
// LWM
static SYS_Timer_t appTimer;
static NWK_DataReq_t appDataReq;
static bool appDataReqBusy = false;
static uint8_t appDataReqBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBufferPtr = 0;
static char LWMmsg[128] = "";
static int LWMlen = 0;
// LWM end
// MQTT
uint8_t MQTT_targetIP[4] = {0, 0, 0, 0};      // MQTT broker IP
static Client mqtt_client; // MQTT client
uint8_t mqtt_readBuffer[MQTT_BUFFER_SIZE];
volatile uint16_t mes_id;
static char ClientID[32] = "mqttx_0e8304f0";
static char ClientUsername[32] = "mqtt_projekt_x";
static char ClientPassword[32] = "\0";
// MQTT end
extern unsigned long millis(void);
volatile unsigned long _millis; // for millis tick !! Overflow every ~49.7 days
uint8_t ping_ip[4] = { 192, 168, 53, 109 }; //Ping IP address
//NIC metrics PC (IP configuration)
wiz_NetInfo netInfo = { .mac  = {0x00, 0x08, 0xdc, 0xab, 0xcd, 0xef}, // Mac address
		.ip   = {192, 168, 53, 199},         // IP address
		.sn   = {255, 255, 255, 0},         // Subnet mask
		.dns =  {8,8,8,8},			  // DNS address (google dns)
		.gw   = {192, 168, 53, 1}, // Gateway address
		.dhcp = NETINFO_STATIC};    //Static IP configuration
const char str_prog_name[] PROGMEM   = "\r\nAtMega256RFR2 LWM -> MQTT Gateway WIZNET_5500 ETHERNET\r\n"; // Program name
#if defined( UART_2 )
#if defined(__AVR_ATmega128__)
const char PROGMEM str_mcu[] = "ATmega128"; //CPU is m128
#elif defined (__AVR_ATmega2560__)
const char PROGMEM str_mcu[] = "ATmega2560"; //CPU is m2560
#elif defined (__AVR_ATmega2561__)
const char PROGMEM str_mcu[] = "ATmega2561"; //CPU is m2561
#elif defined (__AVR_ATmega256RFR2__)
const char PROGMEM str_mcu[] = "ATmega256RFR2"; //CPU is m256rfr2
#elif defined (__AVR_ATmega328P__)
const char PROGMEM str_mcu[] = "ATmega328P"; //CPU is m328p
#elif defined (__AVR_ATmega32U4__)
const char PROGMEM str_mcu[] = "ATmega32u4"; //CPU is m32u4
#elif defined (__AVR_ATmega644P__)
const char PROGMEM str_mcu[] = "ATmega644p"; //CPU is m644p
#elif defined (__AVR_ATmega1284P__)
const char PROGMEM str_mcu[] = "ATmega1284p"; //CPU is m1284p
#else
const char PROGMEM str_mcu[] = "Unknown CPU"; //CPU is unknown
#endif
#endif
static char _msg[64] = "\0";
static int _len;
//***************** DNS: BEGIN
//////////////////////////////////////////////////
// Socket & Port number definition for Examples //
//////////////////////////////////////////////////

unsigned char gDATABUF_DNS[ETH_MAX_BUF_SIZE];

////////////////
// DNS client //
////////////////
uint8_t DNS_2nd[4]    = {192, 168, 53, 1};      	// Secondary DNS server IP
uint8_t Domain_name[] = "broker.emqx.io";    		// domain name
uint8_t Domain_IP[4]  = {0, };               		// Translated IP address by DNS Server
//***************** DNS: END


/*- Prototypes -------------------------------------------------------------*/
// LWM
static void appSendData(void);
static void appDataConf(NWK_DataReq_t *req);
void HAL_UartBytesReceived(uint16_t bytes);
static void appTimerHandler(SYS_Timer_t *timer);
static void appInit(void);
static void APP_TaskHandler(void);
// LWM end
// MQTT
void messageArrived(MessageData* md);
void mqtt_pub(Client* mqtt_client, char * mqtt_topic, char * mqtt_msg, int mqtt_msg_len);
// MQTT end
static void avr_init(void);
void timer0_init(void);
//Wiznet FUNC header
void print_network_information(void);
// RAM Memory usage test
int freeRam (void);
unsigned long millis(void);
void icmp_cb(uint8_t socket,\
		uint8_t* ip_query,\
		uint8_t type_query,\
		uint16_t id_query,\
		uint16_t seq_query,\
		uint16_t len_query);


//START LWM
/*- Types ------------------------------------------------------------------*/
typedef enum AppState_t
{
	APP_STATE_INITIAL,
	APP_STATE_IDLE,
} AppState_t;

static AppState_t appState = APP_STATE_INITIAL;

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
static bool appDataInd(NWK_DataInd_t *ind)
{
strcpy(LWMmsg, "");
LWMlen = SPRINTF(LWMmsg,"%s" ,ind->data);
mqtt_pub(&mqtt_client, PUBLISH, LWMmsg, LWMlen);
for (uint8_t i = 0; i < ind->size; i++){
HAL_UartWriteByte(ind->data[i]);
}
return true;
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

//END LWM


//***********Prologue for fast WDT disable & and save reason of reset/power-up: BEGIN
uint8_t mcucsr_mirror __attribute__ ((section (".noinit")));
// This is for fast WDT disable & and save reason of reset/power-up
void get_mcusr(void) \
  __attribute__((naked)) \
  __attribute__((section(".init3")));
void get_mcusr(void)
{
  mcucsr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}
//***********Prologue for fast WDT disable & and save reason of reset/power-up: END


//******************* MQTT: BEGIN

//MQTT subscribe call-back is here
void messageArrived(MessageData* md)
{
	char _topic_name[64] = "\0";
	char _message[128] = "\0";
	char _msg[172] = "\0";

	MQTTMessage* message = md->message;
	MQTTString* topic = md->topicName;
	strncpy(_topic_name, topic->lenstring.data, topic->lenstring.len);
	strncpy(_message, message->payload, message->payloadlen);
	
	_len = SPRINTF(_msg,"<<MQTT Sub: [%s] %s", _topic_name , _message);
	HAL_UARTWriteString(_msg);
	
	
	// Check if it is received message (received message must contain { char)
	char * ptr;
	int    ch = '{';
	ptr = strchr(_msg, ch );
	
	if (ptr!=NULL)
	{
	int i_1;
	int i_2;
	char str_1[15], str_2[14], str_3[2];
	int mqtt_timer = 0;

	// Getting values from string
	// Message must be like {"mqtt_timer": 5000 ,"vypis_cau": 1 }
	sscanf (_message,"%15s%d%14s%d%s",str_1,&i_1,str_2,&i_2,str_3);
	// Checking if string contain substring
	if (strstr(str_1, "mqtt_timer") != NULL) {
		mqtt_timer=i_1;
		strcpy(_msg, "");
		_len = SPRINTF(_msg,"\r\nHodnota promenne mqtt_timer je: %d\r\n", mqtt_timer);
		HAL_UARTWriteString(_msg);
	} else if (strstr(str_2, "mqtt_timer") != NULL)
	{
		mqtt_timer=i_2;
		strcpy(_msg, "");
		_len = SPRINTF(_msg,"\r\nHodnota promenne mqtt_timer je: %d\r\n", mqtt_timer);
		HAL_UARTWriteString(_msg);
	} 
	// Checking if string contain substring
	if (strstr(str_1, "vypis_cau") != NULL)
	{
		if (i_1==1)
		{
			HAL_UARTWriteString("\r\nCau\r\n");
		}
	} else if (strstr(str_2, "vypis_cau") != NULL)
	{
		if (i_2==1)
		{
			HAL_UARTWriteString("\r\nCau\r\n");
		}
	}	
	}
}

void mqtt_pub(Client* mqtt_client, char * mqtt_topic, char * mqtt_msg, int mqtt_msg_len)
{
	static uint32_t mqtt_pub_count = 0;
	static uint8_t mqtt_err_cnt = 0;
	int32_t mqtt_rc;

	wdt_reset();

	PRINTF(">>MQTT pub msg %lu ", ++mqtt_pub_count);
	MQTTMessage pubMessage;
	pubMessage.qos = QOS0;
	pubMessage.id = mes_id++;
	pubMessage.payloadlen = (size_t)mqtt_msg_len;
	pubMessage.payload = mqtt_msg;
	mqtt_rc = MQTTPublish(mqtt_client, mqtt_topic , &pubMessage);
	//Analize MQTT publish result (for MQTT failover mode)
	if (mqtt_rc == SUCCESSS)
	{
		mqtt_err_cnt  = 0;
		PRINTF(" - OK\r\n");
	}
	else
	{
		PRINTF(" - ERROR\r\n");
		//Reboot device after 20 continuous errors (~ 20sec)
		if(mqtt_err_cnt++ > 20)
		{
			PRINTF("Connection with MQTT Broker was lost!!\r\nReboot the board..\r\n");
			while(1);
		}
	}
}
//******************* MQTT: END


// RAM Memory usage test
int freeRam (void)
{
	extern int __heap_start, *__brkval;
	int v;
	int _res = (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
	return _res;
}


//******************* MILLIS ENGINE: BEGIN
ISR (TIMER0_COMPA_vect)
{
	// Compare match Timer0
	// Here every 1ms
	_millis++; // INC millis tick
}

unsigned long millis(void)
{
	unsigned long i;
	cli();
	// Atomic tick reading
	i = _millis;
	sei();
	return i;
}
//******************* MILLIS ENGINE: END

//***************** UART0: BEGIN
// Assign I/O stream to UART

#if defined( UART_2 )

static int uart0_putchar(char ch,FILE *stream);
//static void uart0_rx_flash(void);

static FILE uart0_stdout = FDEV_SETUP_STREAM(uart0_putchar, NULL, _FDEV_SETUP_WRITE);
//PS. stdin ?? ????????????, ?.?. ??????? ? ??? ???????? ????? uart.h - api:

/*
 * ?.?. ???????? ???
        c = uart1_getc();
        if (( c & UART_NO_DATA ) == 0)
        {
           uart1_putc( (unsigned char)c );
        }
 ??? ???? ?????? ??? ????? ?????? ?? ???? ? ????? ???? ????????????? (+ ???????? ????? UART RX RINGBUFFER),
 ? ???? ???????? ? ????? stdin->getchar() ??? ????? ??????????? ???? ?????? ?? ????? ?????? (???????)
 ????? UART1_RX, ?.?. ????????.
 */

// STDOUT UART0 TX handler
static int uart0_putchar(char ch,FILE *stream)
{
	uart1_putc(ch);
	return 0;
}

/*
// ??????? ????? ?????? UART1 RX (?????? ?????)
static void uart0_rx_flash(void)
{
	// ????????? ??? ?? ring-buffer UART1 RX
	unsigned int c;
	do
	{
		c = uart1_getc();
	} while (( c & UART_NO_DATA ) == 0); // Check RX1 none-empty

}*/

//***************** UART0: END
#endif


//***************** WIZCHIP INIT: BEGIN

unsigned char ethBuf0[ETH_MAX_BUF_SIZE];
unsigned char ethBuf1[ETH_MAX_BUF_SIZE];

void cs_sel() {
	SPI_WIZNET_ENABLE();
}

void cs_desel() {
	SPI_WIZNET_DISABLE();
}

uint8_t spi_rb(void) {
	uint8_t rbuf;
	//HAL_SPI_Receive(&hspi1, &rbuf, 1, HAL_MAX_DELAY);
	SPI_READ(rbuf);
	return rbuf;
}

void spi_wb(uint8_t b) {
	//HAL_SPI_Transmit(&hspi1, &b, 1, HAL_MAX_DELAY);
	SPI_WRITE(b);
}

void spi_rb_burst(uint8_t *buf, uint16_t len) {
	//HAL_SPI_Receive_DMA(&hspi1, buf, len);
	//while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
	for (uint16_t var = 0; var < len; var++) {
		SPI_READ(*buf++);
	}
}

void spi_wb_burst(uint8_t *buf, uint16_t len) {
	//HAL_SPI_Transmit_DMA(&hspi1, buf, len);
	//while(HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
	for (uint16_t var = 0; var < len; var++) {
		SPI_WRITE(*buf++);
	}
}

void IO_LIBRARY_Init(void) {
	uint8_t bufSize[] = {2, 2, 2, 2, 2, 2, 2, 2};

	reg_wizchip_cs_cbfunc(cs_sel, cs_desel);
	reg_wizchip_spi_cbfunc(spi_rb, spi_wb);
	reg_wizchip_spiburst_cbfunc(spi_rb_burst, spi_wb_burst);

	wizchip_init(bufSize, bufSize);
	wizchip_setnetinfo(&netInfo);
	//wizchip_setinterruptmask(IK_SOCK_0);
}
//***************** WIZCHIP INIT: END


//ICMP callback (fire on ICMP request/reply from ping_srv)
/*
 * socket - socket number
 * ip_query - IP from which ICMP query (like 192.168.0.x)
 * type_query - ICMP query type: PING_REQUEST or PING_REPLY
 * id_query - ICMP query Identificator: ID ICMP [0..0xFFFF]
 * seq_query - ICMP query Sequence Number : ID Seq num [0..0xFFFF]
 * len_query - ICMP query length of the data
 */
void icmp_cb(uint8_t socket,\
		uint8_t* ip_query,\
		uint8_t type_query,\
		uint16_t id_query,\
		uint16_t seq_query,\
		uint16_t len_query)
{
	PRINTF( "<< PING %s from %d.%d.%d.%d ID:%x Seq:%x data:%u bytes\r\n",\
			type_query? "Request": "Reply",\
			(int16_t) ip_query[0],\
			(int16_t) ip_query[1],\
			(int16_t) ip_query[2],\
			(int16_t) ip_query[3],\
			id_query,\
			seq_query,\
			len_query);
}

// Timer0
// 1ms IRQ
// Used for millis() timing
void timer0_init(void)
{
	TCCR0A = (1<<WGM01); //TIMER0 SET-UP: CTC MODE
	TCCR0B = (1<<CS01)|(1<<CS00); // PS 1:64
	OCR0A = 249; // 1ms reach for clear (16mz:64=>250kHz:250-=>1kHz)
	TIMSK0 |= 1<<OCIE0A;	 //IRQ on TIMER0 output compareA
}

static void avr_init(void)
{
	// Initialize device here.
	// WatchDog INIT
	wdt_enable(WDTO_8S);  // set up wdt reset interval 2 second
	wdt_reset(); // wdt reset ~ every <2000ms

	timer0_init();// Timer0 millis engine init

	// Initial UART Peripheral
	/*
	 *  Initialize uart11 library, pass baudrate and AVR cpu clock
	 *  with the macro
	 *  uart1_BAUD_SELECT() (normal speed mode )
	 *  or
	 *  uart1_BAUD_SELECT_DOUBLE_SPEED() ( double speed mode)
	 */
#if defined( UART_2 )
#if	(UART_BAUD_RATE == 115200)
	uart1_init( UART_BAUD_SELECT_DOUBLE_SPEED(UART_BAUD_RATE,F_CPU) ); // To works without error on 115200 bps/F_CPU=16Mhz
#else
	uart1_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
#endif
	// Define Output/Input Stream
	stdout = &uart0_stdout;
#endif
	
	sei(); //re-enable global interrupts

	return;
}

void print_network_information(void)
{

	uint8_t tmpstr[6] = {0,};
	ctlwizchip(CW_GET_ID,(void*)tmpstr); // Get WIZCHIP name
    PRINTF("\r\n=======================================\r\n");
    PRINTF(" WIZnet chip:  %s \r\n", tmpstr);
    PRINTF("=======================================\r\n");

	wiz_NetInfo gWIZNETINFO;
	wizchip_getnetinfo(&gWIZNETINFO);
	if (gWIZNETINFO.dhcp == NETINFO_STATIC)
		PRINTF("STATIC IP\r\n");
	else
		PRINTF("DHCP IP\r\n");
	printf("Mac address: %02x:%02x:%02x:%02x:%02x:%02x\n\r",gWIZNETINFO.mac[0],gWIZNETINFO.mac[1],gWIZNETINFO.mac[2],gWIZNETINFO.mac[3],gWIZNETINFO.mac[4],gWIZNETINFO.mac[5]);
	printf("IP address : %d.%d.%d.%d\n\r",gWIZNETINFO.ip[0],gWIZNETINFO.ip[1],gWIZNETINFO.ip[2],gWIZNETINFO.ip[3]);
	printf("SM Mask	   : %d.%d.%d.%d\n\r",gWIZNETINFO.sn[0],gWIZNETINFO.sn[1],gWIZNETINFO.sn[2],gWIZNETINFO.sn[3]);
	printf("Gate way   : %d.%d.%d.%d\n\r",gWIZNETINFO.gw[0],gWIZNETINFO.gw[1],gWIZNETINFO.gw[2],gWIZNETINFO.gw[3]);
	printf("DNS Server : %d.%d.%d.%d\n\r",gWIZNETINFO.dns[0],gWIZNETINFO.dns[1],gWIZNETINFO.dns[2],gWIZNETINFO.dns[3]);
} */