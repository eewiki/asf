/**
 * \file
 *
 * \brief ATmegaRFX RCB board header file.
 *
 * This file contains definitions and services related to the features of the
 * ATmega256RFR2 Xplained Pro board.
 *
 * To use this board, define BOARD= ATMEGA256RFR2_XPLAINED_PRO.
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
#ifndef _ATMEGARFX_RCB_
#define _ATMEGARFX_RCB_
#include "compiler.h"
#include "conf_board.h"
#include "helper.h"



#if AVR8_PART_IS_DEFINED(ATmega128RFA1)
#define MCU_SOC_NAME        "ATMEGA128RFA1"

#ifdef  SENSOR_TERMINAL_BOARD
    #define BOARD_NAME          "RF4CE-EK"
#else    
   #define BOARD_NAME          "ATmega128RFA1 RCB"   
#endif

#elif AVR8_PART_IS_DEFINED(ATmega256RFR2) 
#define MCU_SOC_NAME        "ATMEGA256RFR2"

#ifdef  SENSOR_TERMINAL_BOARD
    #define BOARD_NAME          "ATmega256RFR2-EK"
#else
    #define BOARD_NAME          "ATmega256RFR2 RCB" 
#endif
#endif



#define NUM_CHECK 10

/*
 * Various helper macros for accessing the memory mapped External RAM on the
 * Sensor Terminal Board
 */
#define XRAM_DATA_SETINP()   do{ XRAM_DATA_DDR = 0x00; XRAM_DATA_PORT = 0x00; }while(0)
#define XRAM_DATA_SETOUTP()  do{ XRAM_DATA_DDR = 0xFF; }while(0)
#define XRAM_CTRL_RD_LO()    do{ XRAM_CTRL_PORT &= ~XRAM_RD; }while(0)
#define XRAM_CTRL_RD_HI()    do{ XRAM_CTRL_PORT |= XRAM_RD; }while(0)
#define XRAM_CTRL_WR_LO()    do{ XRAM_CTRL_PORT &= ~XRAM_WR; }while(0)
#define XRAM_CTRL_WR_HI()    do{ XRAM_CTRL_PORT |= XRAM_WR; }while(0)

/**
 * \name Macros for XRAM access
 * \{
 */

/* XRAM specific port macros */

/* XRAM control port output register */
#define XRAM_CTRL_PORT                   (PORTE)

/* XRAM control port direction register */
#define XRAM_CTRL_DDR                    (DDRE)

/* Port pin which is used to signal a read access from the XRAM */
#define XRAM_RD                          (0x20)  /* PE5 */

/* Port pin which is used to signal a write access to the XRAM */
#define XRAM_WR                          (0x10)  /* PE4 */

/* XRAM data port output register */
#define XRAM_DATA_PORT                   (PORTB)

/* XRAM data port direction register */
#define XRAM_DATA_DDR                    (DDRB)

/* XRAM data port input register */
#define XRAM_DATA_PIN                    (PINB)

/* Port where the latch's ALE signal is connected to */
#define XRAM_ALE_PORT                    (PORTG)

/* Data direction register used to activate the ALE signal */
#define XRAM_ALE_DDR                     (DDRG)

/* Pin where the latch's ALE signal is connected to */
#define XRAM_ALE_PIN                     (0x04)  /* PG2 */

/*
 * PINs where buttons are connected
 */
#define BUTTON_PIN_0                    (PB0)

#define LED0                 LED0_GPIO
#define LED1                 LED1_GPIO
#define LED2                 LED2_GPIO
#define LED0_GPIO			  LED_0  
#define LED1_GPIO			  LED_1 
#define LED2_GPIO			  LED_2 
#define LED_COUNT             3

#define LED0_RCB			  IOPORT_CREATE_PIN(PORTE, 2)	  
#define LED1_RCB		      IOPORT_CREATE_PIN(PORTE, 3)	  
#define LED2_RCB			  IOPORT_CREATE_PIN(PORTE, 4)	  

#define LED_ADDR_DEC_DDR                (DDRD)
#define _BV(x) (1 << (x))
/* LED address decoding port output register */
#define LED_ADDR_DEC_PORT               (PORTD)

#define GPIO_PUSH_BUTTON_0			  IOPORT_CREATE_PIN(PORTE, 5)

#ifdef SENSOR_TERMINAL_BOARD

/* Button address decoding port output register */
#define BUTTON_ADDR_DEC_PORT            (PORTD)
#define BUTTON_PORT                     (PORTB)
#define BUTTON_PORT_DIR                 (DDRB)

/* Button address decoding port direction register */
#define BUTTON_ADDR_DEC_DDR             (DDRD)
/* LED address decoding port direction register */

#define BUTTON_INPUT_PINS               (PINB)

/**
 * \name FTDI based USB macros
 * \{
 */

/* USB specific port macros */

/* USB control port output register */
#define USB_CTRL_PORT                   (PORTE)

/* USB control port direction register */
#define USB_CTRL_DDR                    (DDRE)

/* USB control port input register */
#define USB_CTRL_PIN                    (PINE)

/* Port pin which gives indication of reception of byte */
#define USB_RXF                         (0x80)  /* PE7*/

/* Port pin which gives indication of transmission of byte */
#define USB_TXE                         (0x40)  /* PE6*/

/* Port pin which is used to signal a read access from the FT245 */
#define USB_RD                          (0x20)  /* PE5 */

/* Port pin which is used to signal a write access to the FT245 */
#define USB_WR                          (0x10)  /* PE4 */

/* USB data port output register */
#define USB_DATA_PORT                   (PORTB)

/* USB data port direction register */
#define USB_DATA_DDR                    (DDRB)

/* USB data port input register */
#define USB_DATA_PIN                    (PINB)

/* Memory address mapped to USB FIFO */
#define USB_FIFO_AD                     (0x2200)

/* Port where the latch's ALE signal is connected to */
#define USB_ALE_PORT                    (PORTG)

/* Data direction register used to activate the ALE signal */
#define USB_ALE_DDR                     (DDRG)

/* Pin where the latch's ALE signal is connected to */
#define USB_ALE_PIN                     (0x04)  /* PG2 */

/* USB address decoding port output register */
#define USB_ADDR_DEC_PORT               (PORTD)

/* USB address decoding port direction register */
#define USB_ADDR_DEC_DDR                (DDRD)

//! \}
/*
 * USB0 non-generic (board specific) initialization part.
 * If this is required, the following macro is filled with the proper
 * implementation.
 */
/* Enable USB address decoding. */
#define USB_INIT_NON_GENERIC()      do { \
        USB_ADDR_DEC_PORT &= ~_BV(6);        \
        USB_ADDR_DEC_DDR |= _BV(6);          \
        USB_ADDR_DEC_PORT &= ~_BV(7);        \
        USB_ADDR_DEC_DDR |= _BV(7);          \
    } while (0)           

#endif //STB


#endif  /* _ATMEGARFX_RCB_ */
