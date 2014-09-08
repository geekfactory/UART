/*
 * Simple driver for USART on PIC16 "midrange" devices
 *
 * Tested on the following devices: PIC16F628, PIC16F88
 */

#include "UART.h"

xUARTHandle uart_init( enum enUARTModules eUART )
{
	// Initialize IO ports
	// On PIC16F devices USART I/O Ports should be configured as Inputs
	// On PIC16F family all devices have only one hardware USART
	if( eUART != E_UART_1 )
		return 0;
	// Reset UART registers
	TXSTA = 0x00;
	RCSTA = 0x00;
	SPBRG = 0x00;

	//uart_control ( uart1, UART_DATABITS_8 | UART_PARITY_NONE | UART_STOPBITS_1, 9600 );

	// Return module identifier
	return eUART;
}

PORTBASE uart_control( xUARTHandle uartd, DWORD ctrl, DWORD arg )
{
	// The  USART on PIC16F devices only supports 1 stopbit and has no parity
	// generator built in, parity however can be emulated using the ninth bit

	if ( uartd != 1 )
		return -1; // Invalid UART

	TXSTAbits.BRGH = 1;
	TXSTAbits.SYNC = 0;

	if( FALSE ) {
		RCSTAbits.RX9 = 1;
		TXSTAbits.TX9 = 1;
	} else {
		RCSTAbits.RX9 = 0;
		TXSTAbits.TX9 = 0;
	}

	SPBRG = ( (DWORD) 1000000 / ( 4 * (DWORD) arg ) ) - 1;
	
	return 0;
}

BOOL uart_open( xUARTHandle uartd )
{
	if( uartd != 1 )
		return FALSE;

	TXSTAbits.SYNC = 0;
	RCSTAbits.SPEN = 1;
	TXSTAbits.TXEN = 1;
	RCSTAbits.CREN = 1;

	return TRUE;
}

BOOL uart_close( xUARTHandle uartd )
{
	if( uartd != 1 )
		return FALSE;

	TXSTA = 0;
	RCSTA = 0;

	return TRUE;
}

void uart_write( xUARTHandle uartd, BYTE data )
{
	if( uartd != 1 )
		return;
	TXREG = data; // Write to Tx register
	while( !( PIR1bits.TXIF ) ); // Wait while transmission ends
}

void uard_write_word( xUARTHandle uartd, WORD data )
{
	if( uartd != 1 )
		return;

	if( TXSTAbits.TX9 ) {
		if( data & ( 1 << 8 ) ) // Transmit ninth data bit
			TXSTAbits.TX9D = 1;
		else
			TXSTAbits.TX9D = 0;
	}
	TXREG = (BYTE) data & 0x00FF; // Write to Tx register
	while( !( PIR1bits.TXIF ) ); // Wait while transmission ends
}

BYTE uart_read( xUARTHandle uartd )
{
	if( uartd != 1 )
		return 0;
	while( !( PIR1bits.RCIF ) ); // Wait until data recieved
	return RCREG;
}

WORD uart_read_word( xUARTHandle uartd )
{
	if( uartd != 1 )
		return 0;

	WORD retval = 0;
	while( !( PIR1bits.RCIF ) ); // Wait until data recieved
	if( RCSTAbits.RX9 ) {
		if( RCSTAbits.RX9D )
			retval |= ( 1 << 8 );
	}
	retval |= RCREG;
	return retval;
}

void uart_write_array( xUARTHandle uartd, const BYTE * txbuf, unsigned short len )
{
	while( len ) {
		TXREG = *txbuf++; // Write to Tx register
		while( !( PIR1bits.TXIF ) ); // Wait while transmission ends
		len--;
	}
}

void uart_read_array( xUARTHandle uartd, BYTE * rxbuf, unsigned short len )
{
	while( len ) {
		while( !( PIR1bits.RCIF ) ); // Wait until data recieved
		*rxbuf++ = RCREG;
		len--;
	}
}

void uart_puts( xUARTHandle uartd, const BYTE * string )
{
	if( uartd != 1 )
		return;
	while( *string != '\0' ) {
		TXREG = *string++; // Write to Tx register
		while( !( PIR1bits.TXIF ) ); // Wait while transmission ends
	}
}

void uart_gets( xUARTHandle uartd, BYTE * rxbuf, unsigned short len )
{
	if( uartd != 1 )
		return;
	do {
		while( !( PIR1bits.RCIF ) ); // Wait until data recieved
		*rxbuf = RCREG;
		len--;
	} while( *rxbuf++ != '\0' && len );
}

BOOL uart_available( xUARTHandle uartd )
{
	if( uartd != 1 )
		return FALSE;

	if( RCSTAbits.OERR ) {
		RCSTAbits.CREN = 0;
		RCSTAbits.CREN = 1;
	}
	return ( PIR1bits.RCIF ) ? TRUE : FALSE;
}

