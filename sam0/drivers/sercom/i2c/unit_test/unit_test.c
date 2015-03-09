/**
 * \file
 *
 * \brief SAM0 SERCOM I2C Unit test
 *
 * Copyright (C) 2015 Atmel Corporation. All rights reserved.
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
 * \mainpage SAM I2C Unit Test
 * See \ref appdoc_main "here" for project documentation.
 * \copydetails appdoc_preface
 *
 *
 * \page appdoc_preface Overview
 * This unit test carries out tests for the I2C driver.
 * It consists of test cases for the following functionalities:
 *      - Test for I2C initialization.
 *      - Test for I2C master transfer.
 *      - Test for I2C master full speed transfer.
 */

/**
 * \page appdoc_main SAM0 I2C Unit Test
 *
 * Overview:
 * - \ref asfdoc_sam0_i2c_unit_test_intro
 * - \ref asfdoc_sam0_i2c_unit_test_setup
 * - \ref asfdoc_sam0_i2c_unit_test_usage
 * - \ref asfdoc_sam0_i2c_unit_test_compinfo
 * - \ref asfdoc_sam0_i2c_unit_test_contactinfo
 *
 * \section asfdoc_sam0_i2c_unit_test_intro Introduction
 * \copydetails appdoc_preface
 *
 * The following kit is required for carrying out the test:
 *  - SAM D21 Xplained Pro board
 *
 * \section asfdoc_sam0_i2c_unit_test_setup Setup
 * This unit_test is for master,
 * slave should be integrated with the slave_for_unit_test program,
 * The following connections has to be made using wires:
 *  - \b master PA08 (EXT1 PIN11) <-----> slave PA08 (EXT1 PIN11) 
 *  - \b master PA09 (EXT1 PIN12) <-----> slave PA09 (EXT1 PIN12)
 *
 * To run the test:
 *  - Connect the supported Xplained Pro board to the computer using a
 *    micro USB cable.
 *  - Open the virtual COM port in a terminal application.
 *    \note The USB composite firmware running on the Embedded Debugger (EDBG)
 *          will enumerate as debugger, virtual COM port and EDBG data
 *          gateway.
 *  - Build the project, program the target and run the application.
 *    The terminal shows the results of the unit test.
 *
 * \section asfdoc_sam0_i2c_unit_test_usage Usage
 *  - The unit test configures a I2C master, write to slave and then read from slave.
 *  - compare the write and read results
 *
 * \section asfdoc_sam0_i2c_unit_test_compinfo Compilation Info
 * This software was written for the GNU GCC and IAR for ARM.
 * Other compilers may or may not work.
 *
 * \section asfdoc_sam0_i2c_unit_test_contactinfo Contact Information
 * For further information, visit
 * <a href="http://www.atmel.com">http://www.atmel.com</a>.
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <stdio_serial.h>
#include <string.h>
#include "conf_test.h"

/* Structure for UART module connected to EDBG (used for unit test output) */
struct usart_module cdc_uart_module;

//! [packet_data]
#define DATA_LENGTH 10
#define SLAVE_ADDRESS 0x12
static uint8_t write_buffer[DATA_LENGTH] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
};


//! [packet_data]

struct i2c_master_module i2c_master_instance;

/**
 * \brief Initialize the USART for unit test
 *
 * Initializes the SERCOM USART (SERCOM4) used for sending the
 * unit test status to the computer via the EDBG CDC gateway.
 */
static void cdc_uart_init(void)
{
	struct usart_config usart_conf;

	/* Configure USART for unit test output */
	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = CONF_STDIO_MUX_SETTING;
	usart_conf.pinmux_pad0 = CONF_STDIO_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = CONF_STDIO_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = CONF_STDIO_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = CONF_STDIO_PINMUX_PAD3;
	usart_conf.baudrate    = CONF_STDIO_BAUDRATE;

	stdio_serial_init(&cdc_uart_module, CONF_STDIO_USART, &usart_conf);
	usart_enable(&cdc_uart_module);
}

/**
 * \internal
 * \brief Test for I2C master initialization.
 *
 * This test initializes the i2c master module and checks whether the
 * initialization is successful or not.
 *
 * \param test Current test case.
 */
static void run_i2c_init_test(const struct test_case *test)
{
	enum status_code status;
	struct i2c_master_config config_i2c_master;

	i2c_master_get_config_defaults(&config_i2c_master);
	config_i2c_master.buffer_timeout = 10000;
	status = i2c_master_init(&i2c_master_instance, SERCOM2, &config_i2c_master);
		/* Check for successful initialization */
	test_assert_true(test, status == STATUS_OK,
			"I2C master initialization failed");
	i2c_master_enable(&i2c_master_instance);
}

/**
 * \internal
 * \brief Test for I2C master transfer.
 *
 * First test transfer function with stop.
 * write to slave, read from slave and then compare the data.
 * the slave send out the data it received,
 * so master write and read data should be the same.
 *
 * Then test transfer function without stop.
 * write to slave, then use i2c_master_send_stop to complete writing,
 * read from slave, compare the data.
 * finally, use function with stop to complete the transfer.
 *
 * \param test Current test case.
 */
static void run_i2c_master_transfer_test(const struct test_case *test)
{
	uint32_t timeout_cycles = 1000;
	uint32_t i;
	bool status = true;
	uint8_t read_buffer[DATA_LENGTH] = {0};
	struct i2c_master_packet packet = {
		.address     = SLAVE_ADDRESS,
		.data_length = DATA_LENGTH,
		.data        = write_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};
	/* with stop function: master transfer test */
	/* wait the master write to complete */
	do {
		timeout_cycles--;
		if (i2c_master_write_packet_wait(&i2c_master_instance, &packet) ==
			STATUS_OK) {
			break;
		}
	} while (timeout_cycles > 0);
	test_assert_true(test, timeout_cycles > 0,
			"i2c master write failed");
	
	/* wait the master read to complete */
	packet.data = read_buffer;
	timeout_cycles = 1000;
	do {
		timeout_cycles--;
		if (i2c_master_read_packet_wait(&i2c_master_instance, &packet) ==
			STATUS_OK) {
			break;
		}
	} while (timeout_cycles > 0);
	test_assert_true(test, timeout_cycles > 0,
			"i2c master read failed");
	
		/* Compare the sent and the received */
	for (i = 0; i < DATA_LENGTH; i++) {
		if (read_buffer[i] != write_buffer[i]) {
			status = false;
			break;
		}
	}
	test_assert_true(test, status == true,
			"i2c master transfer comparsion failed");
	/* with stop function master transfer test end */
	
	/* without stop function: master transfer test*/
	/* wait the master write to finish */
	packet.data = write_buffer;
	timeout_cycles = 1000;
	do {

		timeout_cycles--;
		if (i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &packet) ==
		STATUS_OK) {
			break;
		}
	} while (timeout_cycles > 0);
	test_assert_true(test, timeout_cycles > 0,
	"i2c master write without stop failed");
	
	/* use i2c_master_send_stop to complete master writing */
	i2c_master_send_stop(&i2c_master_instance);
	
	/* wait the master read to finish */
	packet.data = read_buffer;
	timeout_cycles = 1000;
	do {
		timeout_cycles--;
		if (i2c_master_read_packet_wait_no_stop(&i2c_master_instance, &packet) ==
		STATUS_OK) {
			break;
		}
	} while (timeout_cycles > 0);
	test_assert_true(test, timeout_cycles > 0,
	"i2c master read without stop failed");
	
	/* Compare the sent and the received */
	for (i = 0; i < DATA_LENGTH; i++) {
		if (read_buffer[i] != write_buffer[i]) {
			status = false;
			break;
		}
	}
	test_assert_true(test, status == true,
	"i2c master transfer without stop comparsion failed");
	
	/* use i2c_master_write_packet_wait to complete the transfer */
	packet.data = write_buffer;
	do {
		timeout_cycles--;
		if (i2c_master_write_packet_wait(&i2c_master_instance, &packet) ==
		STATUS_OK) {
			break;
		}
	} while (timeout_cycles > 0);
	test_assert_true(test, timeout_cycles > 0,
	"i2c master write with repeated start failed");
	
	/* without stop function: master transfer test end*/
}

/**
 * \internal
 * \brief Test full speed mode master transfer.
 *
 * test function with stop in full speed mode.
 * \param test Current test case.
 */
static void run_i2c_full_speed_test(const struct test_case *test)
{
	enum status_code status;
	struct i2c_master_config config_i2c_master;
	
	/* init i2c master in full speed mode*/
	i2c_master_get_config_defaults(&config_i2c_master);
	config_i2c_master.buffer_timeout = 10000;
	config_i2c_master.baud_rate = I2C_MASTER_BAUD_RATE_400KHZ;
	i2c_master_disable(&i2c_master_instance);
	status = i2c_master_init(&i2c_master_instance, SERCOM2, &config_i2c_master);
		/* Check for successful initialization */
	test_assert_true(test, status == STATUS_OK,
			"I2C master fast-mode initialization failed");
	i2c_master_enable(&i2c_master_instance);

	uint32_t timeout_cycles = 1000;
	uint32_t i;
	bool status1 = true;
	struct i2c_master_packet packet = {
		.address     = SLAVE_ADDRESS,
		.data_length = DATA_LENGTH,
		.data        = write_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};
	
	 uint8_t read_buffer[DATA_LENGTH] = {0};
	
	/* wait master write complete */	 
	do {
		timeout_cycles--;
		if (i2c_master_write_packet_wait(&i2c_master_instance, &packet) ==
			STATUS_OK) {
			break;
		}
	} while (timeout_cycles > 0);
	test_assert_true(test, timeout_cycles > 0,
			"i2c master write failed");
	
	/* wait master read complete */
	packet.data = read_buffer;
	timeout_cycles = 1000;
	do {
		timeout_cycles--;
		if (i2c_master_read_packet_wait(&i2c_master_instance, &packet) ==
			STATUS_OK) {
			break;
		}
	} while (timeout_cycles > 0);
	test_assert_true(test, timeout_cycles > 0,
			"i2c master read failed");
	
		/* Compare the sent and the received */
	for (i = 0; i < DATA_LENGTH; i++) {
		if (read_buffer[i] != write_buffer[i]) {
			status1 = false;
			break;
		}
	}
	test_assert_true(test, status1 == true,
			"i2c master transfer comparsion failed");
}

	
/**
 * \brief Run I2C master unit tests
 *
 * Initializes the system and serial output, then sets up the
 * I2C master unit test suite and runs it.
 */
int main(void)
{
	system_init();
	cdc_uart_init();

	/* Define Test Cases */
	DEFINE_TEST_CASE(i2c_init_test,
			NULL,
			run_i2c_init_test,
			NULL,
			"Testing I2C Initialization");

	DEFINE_TEST_CASE(i2c_master_transfer_test,
			NULL,
			run_i2c_master_transfer_test,
			NULL,
			"Testing I2C master data transfer");

	DEFINE_TEST_CASE(i2c_full_speed_test,
			NULL,
			run_i2c_full_speed_test,
			NULL,
			"Testing I2C change speed transfer");

	/* Put test case addresses in an array */
	DEFINE_TEST_ARRAY(i2c_tests) = {
		&i2c_init_test,
		&i2c_master_transfer_test,
		&i2c_full_speed_test,

	};

	/* Define the test suite */
	DEFINE_TEST_SUITE(i2c_test_suite, i2c_tests,
			"SAM I2C driver test suite");

	/* Run all tests in the suite*/
	test_suite_run(&i2c_test_suite);

	while (true) {
		/* Intentionally left empty */
	}
}
