/*	UART Interface Driver
	Copyright (C) 2014 Jesus Ruben Santa Anna Zamudio.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

	Author website: http://www.geekfactory.mx
	Author e-mail: ruben at geekfactory dot mx
 */
#ifndef UART_H
#define UART_H
/*-------------------------------------------------------------*/
/*		Includes and dependencies			*/
/*-------------------------------------------------------------*/
#include "UARTPort.h"

/*-------------------------------------------------------------*/
/*		Macros and definitions				*/
/*-------------------------------------------------------------*/
// Databits per frame
#define	UART_DATABITS_5 0x00000030
#define	UART_DATABITS_6 0x00000040
#define	UART_DATABITS_7 0x00000050
#define	UART_DATABITS_8 0x00000010
#define	UART_DATABITS_9 0x00000020

// Defines for the parity checking modes
#define UART_PARITY_NONE	0x00000100
#define	UART_PARITY_EVEN	0x00000200
#define	UART_PARITY_ODD		0x00000300

// Number of stop bits to send/receive
#define UART_STOPBITS_1		0x00001000
#define UART_STOPBITS_0_5	0x00002000
#define UART_STOPBITS_1_5	0x00003000
#define UART_STOPBITS_2		0x00004000

//Defines the flow control scheme
#define UART_FLOW_CONTROL_NONE		0x00010000
#define UART_FLOW_CONTROL_CTS		0x00020000
#define UART_FLOW_CONTROL_RTS		0x00030000
#define UART_FLOW_CONTROL_RTS_CTS	0x00040000

#if !defined(CONFIG_UART_TXBUF_LEN)
#define CONFIG_UART_TXBUF_LEN   20
#endif
#if !defined(CONFIG_UART_RXBUF_LEN)
#define CONFIG_UART_RXBUF_LEN   20
#endif

/*-------------------------------------------------------------*/
/*		Typedefs enums & structs			*/
/*-------------------------------------------------------------*/

/**
 * Defines the available UART hardware modules
 */
enum enUARTModules {
	E_UART_NULL = 0,
	E_UART_1 = 1,
	E_UART_2 = 2,
	E_UATR_3 = 3,
	E_UART_4 = 4,
	E_UART_5 = 5,
};

#if !defined( BOOL_TYPEDEF )
#define BOOL_TYPEDEF

typedef enum _BOOL {
	FALSE = 0, TRUE
} BOOL;
#endif
/*-------------------------------------------------------------*/
/*		Function prototypes				*/
/*-------------------------------------------------------------*/

/**
 * @brief Prepares the UART resources operation
 *
 * Prepares the required data structures and Operating System resources (if
 * needed) and initializes the registers to a known state.
 *
 * This function returns a handle to the UART resource that will be used to get
 * access to other API calls.
 *
 * If the specified module is not available, or the initialization progress
 * fails by any other cause this function will return NULL.
 * 
 * @param enUARTModules The number of the UART module to initialize.
 *
 * @return A resource handle that identifies the UART module, or NULL if the
 * requested device doesn´t exist or it cannot be initialized.
 */
xUARTHandle uart_init(enum enUARTModules eUART);

/**
 * @brief Sets the control parameters for the UART module.
 *
 * This function sets the configuration for the UART operation. If the UART
 * cannot be configured using the provided parameters, due to incorrect or
 * unsupported settings, this function will return a negative value which
 * indicates the type of error encountered.
 * 
 * The UART module should be closed before calling this function.
 *
 * @param uartd The UART reference of the module to be configured
 * @param ctl A control bitfield with UART options
 * @param arg An argument defining the UART baudrate
 *
 * @return Returns 0
 */
int uart_control(xUARTHandle uartd, uint32_t ctrl, uint32_t arg);

/**
 * @brief Enables UART module operation
 *
 * This function enables the UART module operation, the module takes control
 * over the UART pins. This function returns TRUE if the UART module is not busy.
 *
 * This function can be used as a semaphore when multiple programs or tasks use
 * one UART module in order to implement mutual exclusion between them.
 *
 * @param uartd The UART reference of the module to open
 * 
 * @return Returns TRUE if the UART is available to use and ready or FALSE
 * otherwise
 */
BOOL uart_open(xUARTHandle uartd);

/**
 * @brief Disables UART module operation
 *
 * Closes the UART module and returns the semaphore for mutually exclusive
 * access to the UART resource.
 * 
 * @param uartd The UART reference of the module to close
 *
 * @return Returns TRUE if the UART module is closed succesfully FALSE otherwise
 */
BOOL uart_close(xUARTHandle uartd);

/**
 * @brief Writes a single byte to the UART.
 *
 * Writes a data bit to the UART. This is a blocking call, program (or task)
 * execution will stop untill the data is transmitted or buffer space is
 * available to enqueue the data.
 *
 * @param uartd The UART descriptor of the module to write
 * @param data The data byte to send
 */
void uart_write(xUARTHandle uartd, uint8_t data);

/**
 * @brief Writes a single data word to the UART.
 *
 * Writes a single data word to the UART. This function is intended to be used
 * when transmitting 9 bit data. This is a blocking call, program (or task)
 * execution will stop untill the data is transmitted.
 * 
 * @param xUART The UART hardware descriptor of the module to write
 * @param xTxChar The data byte to send
 */
void uart_writew(xUARTHandle uartd, uint16_t data);

/**
 * @brief Reads a single byte from the UART.
 * 
 * Reads a single data byte from the UART. This is a blocking call, program
 * execution will stop untill a data byte is received.
 * 
 * @param xUART The UART hardware descriptor of the module to read
 *
 * @return Returns the received data byte
 */
uint8_t uart_read(xUARTHandle uartd);

/**
 * @brief Reads a single word from the UART.
 * 
 * Reads a single data word from the UART. This function is intended to be used
 * when receiving 9 bit data. This is a blocking call, program execution will
 * stop untill a data word is received.
 * 
 * @param xUART The UART hardware descriptor of the module to read
 *
 * @return Returns the received data word
 */
uint16_t uart_readw(xUARTHandle uartd);

/**
 * @brief Writes an array of bytes to the UART
 *
 * This function writes an array of bytes to the UART. This is a blocking call,
 * program execution will stop untill data is transmitted.
 *
 * @param xUART The UART hardware descriptor of the module to write
 * @param pcTxBuffer The buffer containing the data to send
 * @param usBufferLength The lenght of the data to send
 */
void uart_write_array(xUARTHandle uartd, const uint8_t * txbuf, uint16_t len);

/**
 * @brief Reads an array of bytes from the UART
 * 
 * This function reads an array of bytes from the UART. This is a blocking call,
 * program execution will stop untill data is received.
 * 
 * @param xUART The UART hardware descriptor of the module to read
 * @param pcRxBuffer The buffer to hold the received data
 * @param usBufferLength The number of bytes to receive
 */
void uart_read_array(xUARTHandle uartd, uint8_t * rxbuf, uint16_t len);

/**
 * @brief Writes a null terminated string to the UART.
 *
 * This function writes a string to the UART, the string should be null
 * terminated. This is a blocking call, program execution will stop untill data
 * is transmitted.
 * 
 * @param xUART The UART hardware descriptor of the module to write
 * @param pcString Pointer to the string to send
 */
void uart_puts(xUARTHandle uartd, const uint8_t * string);

/**
 * @brief Reads a null terminated string from the UART.
 *
 * This function reads a string from the UART, the string should be null
 * terminated. This is a blocking call, program execution will stop untill data
 * is received.
 *
 * @param The UART hardware descriptor of the module to write
 * @param pcBufer Pointer to the buffer to store the received string
 * @param usBufferLength The lenght of the receive buffer, max lenght of string
 */
void uart_gets(xUARTHandle uartd, uint8_t * rxbuf, uint16_t len);

/**
 * @brief Checks if data is available to be read from UART.
 * 
 * Checks if data is available on the UART receive buffer to be read inmediately.
 * This can be used to check for incomming data without blocking program
 * execution.
 * If this function returns TRUE, it means that at least 1 data byte can be read
 * using any uart_read() functions.
 * 
 * @param The UART hardware descriptor of the module to check
 *
 * @return Returns TRUE if data is available to receive, false otherwise.
 */
uint8_t uart_available(xUARTHandle uartd);

void uart_txisr();
void uart_rxisr();

#endif
// End of Header file
