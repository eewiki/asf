 /**
 * @file pal.c
 *
 * @brief Performs interface functionalities between the TAL layer and ASF drivers
 *  Copyright (c) 2013-2014 Atmel Corporation. All rights reserved.
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
 */
/*
 * Copyright (c) 2013, Atmel Corporation All rights reserved.
 *
 * Licensed under Atmel's Limited License Agreement --> EULA.txt
 */

#include "board.h"
#if SAMD20
#include "spi.h"
#else
#include "spi_master.h"
#endif
#include "pal_ext_trx.h"
#include "trx_access.h"
#include "pal.h"
#include "delay.h"
#include "interrupt.h"
#include "conf_trx_access.h"
#include "conf_board.h"



void pal_spi_init(void)
{
	/* Initialize SPI in master mode to access the transceiver */
	trx_spi_init();
}

uint8_t pal_trx_reg_read(uint8_t addr)
{
    /* call TRX register read */
	trx_reg_read(addr);
}

void pal_trx_reg_write(uint8_t addr, uint8_t data)
{

    trx_reg_write(addr,data);
}


void pal_trx_irq_init(FUNC_PTR trx_irq_cb)
{
    trx_irq_init(trx_irq_cb);

}

uint8_t pal_trx_bit_read(uint8_t addr, uint8_t mask, uint8_t pos)
{
    trx_bit_read(addr,mask,pos);
}

void pal_trx_bit_write(uint8_t reg_addr, uint8_t mask, uint8_t pos, uint8_t new_value)
{
    trx_bit_write(reg_addr,mask,pos,new_value);
}

void pal_trx_frame_read(uint8_t *data, uint8_t length)
{
	
    trx_frame_read(data, length);

}

void pal_trx_frame_write(uint8_t *data, uint8_t length)
{
     trx_frame_write(data,length);
}

/**
 * @brief Writes data into SRAM of the transceiver
 *
 * This function writes data into the SRAM of the transceiver
 *
 * @param addr Start address in the SRAM for the write operation
 * @param data Pointer to the data to be written into SRAM
 * @param length Number of bytes to be written into SRAM
 */
void pal_trx_sram_write(uint8_t addr, uint8_t *data, uint8_t length)
{
     trx_sram_write(addr,data,length);
}

/**
 * @brief Reads data from SRAM of the transceiver
 *
 * This function reads from the SRAM of the transceiver
 *
 * @param[in] addr Start address in SRAM for read operation
 * @param[out] data Pointer to the location where data stored
 * @param[in] length Number of bytes to be read from SRAM
 */
void pal_trx_sram_read(uint8_t addr, uint8_t *data, uint8_t length)
{
     trx_sram_read(addr,data,length);
}

/**
 * @brief Writes and reads data into/from SRAM of the transceiver
 *
 * This function writes data into the SRAM of the transceiver and
 * simultaneously reads the bytes.
 *
 * @param addr Start address in the SRAM for the write operation
 * @param idata Pointer to the data written/read into/from SRAM
 * @param length Number of bytes written/read into/from SRAM
 */
void pal_trx_aes_wrrd(uint8_t addr, uint8_t *idata, uint8_t length)
{
    trx_aes_wrrd(addr,idata,length);
}

