/************************* http://geekfactory.mx *******************************
 *
 * This program demonstrates the transmission and reception of data through the
 * USART peripheral. You can review the code on UART.h and UART-PIC16.c to see
 * how each function is implemented.
 *
 * Este programa demuestra la transmisión y recepción de datos a través de la
 * USART del PIC. Puede revisarse el código fuente de las funciones en los
 * archivos UART.h y UART-PIC16.c
 *
 * AUTHOR/AUTOR: Jesus Ruben Santa Anna Zamudio
 * COMPILER/COMPILADOR: Microchip XC8 http://www.microchip.com/compilers
 *
 ********************************************************************************/
#include <xc.h>			// Encabezado para el compilador XC8
#include "../UART.h"

#pragma config FOSC = XT        // Oscillator Selection bits (XT oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is MCLR)
#pragma config BOREN = OFF      // Brown-out Reset Enable bit (BOR disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable bit (RB3 is digital I/O, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EE Memory Code Protection bit (Code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off)
#pragma config CCPMX = RB0      // CCP1 Pin Selection bit (CCP1 function on RB0)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal External Switchover mode disabled)

void main()
{
	// Buffer para almacenar teclas
	uint8_t buf[10];
	uint8_t index = 0;

	ANSEL = 0x00;
	TRISA = 0xFF;
	TRISB = 0b00000100;

	// Get UART ID, The UART library works on parts with more than one UART/USART modules
	xUARTHandle uart1 = uart_init(E_UART_1);

	// Configure UART (standard serial port settings)
	if (uart_control(uart1, UART_DATABITS_8 | UART_PARITY_NONE | UART_STOPBITS_1, 9600))
		for (;;); // Invalid UART settings, should not get here

	// Open UART and begin usage
	if (uart_open(uart1)) {
		// Send a string
		uart_puts(uart1, "PIC UART Demo\r\n");
		// Main loop
		for (;;) {
			// Check if UART received some data
			if (uart_available(uart1)) {
				// receive a single byte
				buf[index] = uart_read(uart1);
				// send a single byte (echo)
				uart_write(uart1, buf[index]);
				// Send a string with control characters
				uart_puts(uart1, "\r\n");
				// Increase counter and check limits
				index++;

				if (index == 10) {
					// Send other string
					uart_puts(uart1, "\r\nRX: ");
					// Send the 10 characters array
					uart_write_array(uart1, buf, 10);
					uart_puts(uart1, "\r\n");
					// Reset index
					index = 0;
				}
			}
		}
	}
	for (;;); // Should not get here

}
