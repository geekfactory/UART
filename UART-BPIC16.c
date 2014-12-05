/*
 * Simple driver for USART on PIC16 "midrange" devices
 *
 * Tested on the following devices: PIC16F628, PIC16F88
 */

#include "UART.h"


uint8_t txbuf[CONFIG_UART_TXBUF_LEN];
uint8_t rxbuf[CONFIG_UART_RXBUF_LEN];

uint8_t tx_read_offset;
uint8_t tx_write_offset;
uint8_t txcount;

uint8_t rx_read_offset;
uint8_t rx_write_offset;
uint8_t rxcount;

volatile BOOL txbussy = FALSE;

xUARTHandle uart_init(enum enUARTModules eUART)
{
	// Initialize IO ports
	// On PIC16F devices USART I/O Ports should be configured as Inputs
	// On PIC16F family all devices have only one hardware USART
	if (eUART != E_UART_1)
		return NULL;
	// Reset UART registers
	TXSTA = 0x00;
	RCSTA = 0x00;
	SPBRG = 0x00;
	// Clear interrupt falgs
	PIR1bits.RCIF = 0;
	PIR1bits.TXIF = 0;
	// Enable global interrupts, requiered by this driver
	INTCONbits.GIE = 1;
	INTCONbits.PEIE = 1;
	// Clear ongoing TX flag
	txbussy = FALSE;
	// Create buffers
	tx_read_offset = 0;
	tx_write_offset = 0;
	// Return module identifier
	return eUART;
}

int uart_control(xUARTHandle uartd, uint32_t ctrl, uint32_t arg)
{
	// The  USART on PIC16F devices only supports 1 stopbit and has no parity
	// generator built in, parity however can be emulated using the ninth bit
	if (uartd != 1)
		return -1; // Invalid UART
	// High baudrate synchronous operation
	TXSTAbits.BRGH = 1;
	TXSTAbits.SYNC = 0;

	if (FALSE) {
		RCSTAbits.RX9 = 1;
		TXSTAbits.TX9 = 1;
	} else {
		RCSTAbits.RX9 = 0;
		TXSTAbits.TX9 = 0;
	}
	// Calculate baudrate
	SPBRG = ((uint32_t) CONFIG_TIMING_MAIN_CLOCK / (4 * (uint32_t) arg)) - 1;

	return 0;
}

BOOL uart_open(xUARTHandle uartd)
{
	if (uartd != 1)
		return FALSE;
	// Clear interrupt flags
	PIR1bits.RCIF = 0;
	PIR1bits.TXIF = 0;
	txbussy = FALSE;
	// Async mode enabled, enable serial port
	TXSTAbits.SYNC = 0;
	RCSTAbits.SPEN = 1;
	//TXSTAbits.TXEN = 1;
	RCSTAbits.CREN = 1;
	// Enable receive and transmit interrupts
	PIE1bits.RCIE = 1;
	PIE1bits.TXIE = 1;

	return TRUE;
}

BOOL uart_close(xUARTHandle uartd)
{
	if (uartd != 1)
		return FALSE;

	TXSTA = 0;
	RCSTA = 0;

	// Disable serial interrupts, clear flags
	PIE1bits.RCIE = 0;
	PIE1bits.TXIE = 0;
	PIR1bits.RCIF = 0;
	PIR1bits.TXIF = 0;

	return TRUE;
}

void uart_write(xUARTHandle uartd, uint8_t data)
{
	if (uartd != 1)
		return;
	// Disable interrupts
	di();
	// Add to TX fifo //fifo_add(txfifo, (void*) &data);
	// wait while buffer is full
	while (txcount == CONFIG_UART_TXBUF_LEN);
	txbuf[tx_write_offset++] = data;
	if (tx_write_offset >= CONFIG_UART_TXBUF_LEN) {
		tx_write_offset = 0;
		txcount++;
	}
	// Enable TX (causes interrupt)
	if (txbussy == FALSE) {
		txbussy = TRUE;
		TXSTAbits.TXEN = 1;
	}
	ei();
}

void uart_writew(xUARTHandle uartd, uint16_t data)
{
	if (uartd != 1)
		return;

	if (TXSTAbits.TX9) {
		if (data & (1 << 8)) // Transmit ninth data bit
			TXSTAbits.TX9D = 1;
		else
			TXSTAbits.TX9D = 0;
	}
	TXREG = (uint8_t) data & 0x00FF; // Write to Tx register
	while (!(PIR1bits.TXIF)); // Wait while transmission ends
}

uint8_t uart_read(xUARTHandle uartd)
{
	uint8_t rxdata;

	// Wait for data to become available
	while (!uart_available(uartd));

	di();
	//fifo_get(rxfifo, &rxdata);
	rxdata = rxbuf[rx_read_offset++];
	if (rx_read_offset >= CONFIG_UART_RXBUF_LEN) {
		rx_read_offset = 0;
		rxcount--;
	}
	ei();

	return rxdata;
}

uint16_t uart_readw(xUARTHandle uartd)
{
	if (uartd != 1)
		return 0;

	uint16_t retval = 0;
	while (!(PIR1bits.RCIF)); // Wait until data recieved
	if (RCSTAbits.RX9) {
		if (RCSTAbits.RX9D)
			retval |= (1 << 8);
	}
	retval |= RCREG;
	return retval;
}

void uart_write_array(xUARTHandle uartd, const uint8_t * txbuf, uint16_t len)
{
	while (len) {
		uart_write(uartd, *txbuf++);
		len--;
	}
}

void uart_read_array(xUARTHandle uartd, uint8_t * rxbuf, uint16_t len)
{
	while (len) {
		*rxbuf++ = uart_read(uartd);
		len--;
	}
}

void uart_puts(xUARTHandle uartd, const uint8_t * string)
{
	while (*string != '\0') {
		uart_write(uartd, *string++);
	}
}

void uart_gets(xUARTHandle uartd, uint8_t * rxbuf, uint16_t len)
{
	do {
		*rxbuf = uart_read(uartd);
		len--;
	} while (*rxbuf++ != '\0' && len);
}

BOOL uart_available(xUARTHandle uartd)
{
	BOOL ret;

	di();
	if (rxcount != 0)
		ret = TRUE;
	else
		ret = FALSE;
	ei();

	return ret;
}

void uart_txisr()
{
	uint8_t txchar;
	if (PIR1bits.TXIF) {
		// try to get item from TX fifo
		if (fifo_get(txfifo, &txchar)) {
			// Transmit data
			TXREG = txchar;
		} else {
			// transmission is finished, no data on fifo right now
			TXSTAbits.TXEN = 0;
			txbussy = FALSE;
		}
	}
}

void uart_rxisr()
{
	uint8_t rxchar;
	if (PIR1bits.RCIF) {
		// Ack interrupt
		//PIR1bits.TXIF = 0;
		// Read receive buffer
		rxchar = RCREG;
		// Add to RX fifo buffer
		fifo_add(rxfifo, &rxchar);
	}
}