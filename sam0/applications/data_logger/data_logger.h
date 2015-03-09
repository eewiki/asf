/**
 * \file
 *
 * \brief SAM D21 Data Logger Application
 *
 * Copyright (C) 2014 Atmel Corporation. All rights reserved.
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

#ifndef DLOG_H_INCLUDED
#define DLOG_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

#include <asf.h>
#include <conf_example.h>
#include <conf_at25dfx.h>
#include "math.h"

/** We expect a single byte command through USART */
#define CMD_RX_BUFFER_LENGTH                 1
#define CMD_GET_ALL_LOGS					 '1'

/**
 * Define the number of bytes transferred into the serial flash at a time.
 * Keep this number low, since different buffers are allocated depending
 * on the transfer size
 */
#define SF_TRANSFER_SIZE                     10
/** Beat count for the DMA resource which transfers data from ADC to SRAM.
 * This number should be the same as SF_TRANSFER_SIZE, but putting it
 * as a macro for readability.
 */
#define NUM_OF_BEATS_IN_ADC_SRAM_TRANSFER    10
/**
 * The number of locations written, before we start again from address 0
 * in a 4K block of the serial flash
 */
#define SF_ADDRESS_LIMIT                     1000
/** Size of the buffer used for storing different messages */
#define INFO_BUFFER_SIZE                     100
/** Macros used when accessing the Serial Flash */
#define SF_4K_BLOCK0_START                   0x0000
#define SF_4K_BLOCK1_START                   0x1000
#define SF_4K_BLOCK2_START                   0x2000
/** Two bytes are used for representing a temperature reading */
#define BYTES_USED_FOR_A_READING             2

/**
 * Prescaler for the RTC.
 */
#define TIMER_PRESCALER                      RTC_COUNT_PRESCALER_DIV_1

/**
 * Parameters for the ADC channel to which the thermistor is connected
 * ADC reference will be selected as ADC_REFERENCE_INTVCC0 -> VCC/1.48
 */
#define THERM_ADC_RESOLUTION                 16
#define THERM_ADC_REF_VOLT                   (float)(3.3 / 1.48)
#define THERM_ADC_VOLTS_PER_BIT              ((float)THERM_ADC_REF_VOLT / \
												(pow(2, THERM_ADC_RESOLUTION)))
/** Series resistor from VCC to thermistor */
#define THERM_SERIES_RESISTOR                10000
/** VCC is used for exciting the thermistor */
#define THERM_EXCITATION_VOLT                (float)3.3

/**
 * Define the Steinhart-Hart coefficients
 * The equation is
 * 1/T = A + B ln(R) + C (ln(R))^3
 * Reference points taken are 0, 25 and 50 Deg C
 */
#define SHH_COEFF_A                          0.0008913055475
#define SHH_COEFF_B                          0.0002507779917
#define SHH_COEFF_C                          0.0000001957724056

/** @} */
#ifdef __cplusplus
}
#endif
#endif /* DLOG_H_INCLUDED */
