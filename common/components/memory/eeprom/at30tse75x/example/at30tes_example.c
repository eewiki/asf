/**
 * \file
 *
 * \brief AT30TSE75x Temperature Sensor Example.
 *
 * Copyright (c) 2013 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage AT30TSE75x Temperature Sensor Example
 *
 * \section Purpose
 *
 * The application demonstrates how to access AT30TSE75x temperature sensor.
 *
 * \section Requirements
 *
 * This package can be used with SAM4N Xplained Pro with I01 Xplained Pro
 * attached on EXT1.
 *
 * \section Description
 * There are 2 stages in the example. In 1st stage, some patterns are
 * written to the specified memory address of the EEPROM in AT30TSE75x. Then
 * the memory is read and checked. In 2nd stage, the temperature sampled by
 * AT30TSE75x is read every second.
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC and IAR EWARM.
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit
 * <A href="http://www.atmel.com/">Atmel</A>.\n
 * Support and FAQ: http://support.atmel.com/
 *
 */

#include "asf.h"
#include <stdio.h>
#include <string.h>

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
extern "C" {
#endif
/**INDENT-ON**/
/// @endcond


#define STRING_EOL    "\r"
#define STRING_HEADER "-- AT30TSE75x Temperature Sensor Example --\r\n" \
		"-- "BOARD_NAME" --\r\n" \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

#define NB_PAGE 16
#define NB_BYTE 16

uint8_t rx[NB_BYTE], tx[NB_BYTE];

/**
 *  \brief Configure the Console UART.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/**
 * \brief Application entry point for AT30TSE75x Component Example.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	uint32_t i;
	double temp = 0;

	/* Initialize the SAM system */
	sysclk_init();

	/* Initialize the board */
	board_init();

	/* Initialize the console UART */
	configure_console();

	/* Output example information */
	puts(STRING_HEADER);

	memset(tx, 0xFF, NB_BYTE);

	/* Initialize AT30TSE75x */
	at30tse_init();

	/* Write pages in EEPROM */
	for (i = 0; i < NB_PAGE; i++) {
		tx[NB_PAGE - 1] = i;
		if (at30tse_eeprom_write(tx, NB_BYTE, 0, i) != TWI_SUCCESS) {
			puts("Write EEPROM error\r");
			return 0;
		}
		delay_ms(5);
	}
	puts("Write EEPROM OK\r");

	/* Read each page in EEPROM and compare them */
	for (i = 0; i < NB_PAGE; i++) {
		memset(rx, 0, NB_BYTE);
		if (at30tse_eeprom_read(rx, NB_BYTE, 0, i) != TWI_SUCCESS) {
			puts("Read EEPROM error\r");
			return 0;
		} else {
			if (memcmp(tx, rx, NB_BYTE - 1) && (rx[NB_PAGE - 1] != i)) {
				puts("Comparison error\r");
				return 0;
			}
		}
	}
	puts("Read EEPROM & Compare OK\r");

	/* Read temperature every second */
	while (1) {
		if (at30tse_read_temperature(&temp) != TWI_SUCCESS) {
			puts("Read temperature error\r");
			return 0;
		}
		printf("Read temperature:\t%d\r\n", (int)temp);
		delay_ms(1000);
	}
}

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond
