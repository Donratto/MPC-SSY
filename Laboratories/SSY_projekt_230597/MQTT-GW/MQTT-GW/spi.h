#ifndef SPI_H_
#define SPI_H_

/* SPI input/output registers. */
#define SPI_TXBUF SPDR
#define SPI_RXBUF SPDR

#define BV(bitno) _BV(bitno)

#define SPI_WAITFOREOTx() do { while (!(SPSR & BV(SPIF))); } while (0)
#define SPI_WAITFOREORx() do { while (!(SPSR & BV(SPIF))); } while (0)

//M128
//#define SCK            1  /* - Output: SPI Serial Clock (SCLK) - ATMEGA128 PORTB, PIN1 */
//#define MOSI           2  /* - Output: SPI Master out - slave in (MOSI) - ATMEGA128 PORTB, PIN2 */
//#define MISO           3  /* - Input:  SPI Master in - slave out (MISO) - ATMEGA128 PORTB, PIN3 */
//#define CSN            0  /*SPI - SS*/
//#define FLASH_CS       6       /* PB.6 Output as CS*/

//M644p/M1284p
//#define SCK            7  /* - Output: SPI Serial Clock (SCLK) - ATMEGA644/1284 PORTB, PIN7 */
//#define MOSI           5  /* - Output: SPI Master out - slave in (MOSI) -  ATMEGA644/1284 PORTB, PIN5 */
//#define MISO           6  /* - Input:  SPI Master in - slave out (MISO) -  ATMEGA644/1284 PORTB, PIN6 */
//#define CSN            4  /*SPI - SS*/

//256rfr2
#define SCK            1  /* - Output: SPI Serial Clock (SCLK) - ATMEGA256rfr2 PORTB, PIN1 */
#define MOSI           2  /* - Output: SPI Master out - slave in (MOSI) -  ATMEGA256rfr2 PORTB, PIN2 */
#define MISO           3  /* - Input:  SPI Master in - slave out (MISO) -  ATMEGA256rfr2 PORTB, PIN3 */
#define CSN            4  /*SPI - SS*   PORTD*/

//#define FLASH_CS       3       /* PB.2 Output as CS*/
//#define FLASH_CS       2       /* PB.2 Output as CS*/
//#define CAN_CS         1       /* PB.1 Output as CS for CAN MCP2515*/

//#define SPI_FLASH_ENABLE()  ( PORTB &= ~BV(FLASH_CS) )
//#define SPI_FLASH_DISABLE() ( PORTB |=  BV(FLASH_CS) )

#define WIZNET_CS       6       /* PD.6 Output as CS for Wiznet ETHERNET*/
#define SPI_WIZNET_ENABLE()  ( PORTD &= ~BV(WIZNET_CS) )
#define SPI_WIZNET_DISABLE() ( PORTD |=  BV(WIZNET_CS) )

/*
#define SD_CS       0        PB.0 Output as CS for SD-reader
#define SPI_SD_ENABLE()  ( PORTB &= ~BV(SD_CS) )
#define SPI_SD_DISABLE() ( PORTB |=  BV(SD_CS) )
*/



/* Define macros to use for checking SPI transmission status depending
   on if it is possible to wait for TX buffer ready. This is possible
   on for example MSP430 but not on AVR. */
#ifdef SPI_WAITFORTxREADY
#define SPI_WAITFORTx_BEFORE() SPI_WAITFORTxREADY()
#define SPI_WAITFORTx_AFTER()
#define SPI_WAITFORTx_ENDED() SPI_WAITFOREOTx()
#else /* SPI_WAITFORTxREADY */
#define SPI_WAITFORTx_BEFORE()
#define SPI_WAITFORTx_AFTER() SPI_WAITFOREOTx()
#define SPI_WAITFORTx_ENDED()
#endif /* SPI_WAITFORTxREADY */

void spi_init(void);

/* Write one character to SPI */
#define SPI_WRITE(data)                         \
  do {                                          \
    SPI_WAITFORTx_BEFORE();                     \
    SPI_TXBUF = data;                           \
    SPI_WAITFOREOTx();                          \
  } while(0)

/* Write one character to SPI - will not wait for end
   useful for multiple writes with wait after final */
#define SPI_WRITE_FAST(data)                    \
  do {                                          \
    SPI_WAITFORTx_BEFORE();                     \
    SPI_TXBUF = data;                           \
    SPI_WAITFORTx_AFTER();                      \
  } while(0)

/* Read one character from SPI */
#define SPI_READ(data)   \
  do {                   \
    SPI_TXBUF = 0;       \
    SPI_WAITFOREORx();   \
    data = SPI_RXBUF;    \
  } while(0)

/* Flush the SPI read register */
#ifndef SPI_FLUSH
#define SPI_FLUSH() \
  do {              \
    SPI_RXBUF;      \
  } while(0);
#endif

#endif /* SPI_H_ */