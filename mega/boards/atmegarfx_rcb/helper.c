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

#include "compiler.h"
#include "conf_board.h"
# include "helper.h"

/**
 * \brief Read XRAM
 *
 * \param
 */
uint8_t xram_read(uint16_t addr)
{
    uint8_t data;

    /* Set HIGH before switching to output to prevent RD cycle */
    XRAM_CTRL_PORT |= XRAM_RD;
    XRAM_CTRL_DDR |= XRAM_RD;

    /* Set HIGH before switching to output to prevent WR cycle */
    XRAM_CTRL_PORT |= XRAM_WR;
    XRAM_CTRL_DDR |= XRAM_WR;


    PORTB = (uint8_t)(0x00FF & addr);
    DDRB = 0xFF;

    PORTC = (uint8_t)(0x000F & (addr >> 8));
    DDRC |= 0x0F;

    PORTD = (uint8_t)(0x00F0 & (addr >> 8));
    DDRD |= 0xF0;

    XRAM_ALE_DDR |= XRAM_ALE_PIN;
    XRAM_ALE_PORT |= XRAM_ALE_PIN;
    nop();
    nop();
    XRAM_ALE_PORT &= ~XRAM_ALE_PIN;

    /* Read a character from the fifo address. */
    XRAM_DATA_SETINP();
    XRAM_CTRL_RD_LO();
    nop();
    nop();
    data = XRAM_DATA_PIN;
    XRAM_CTRL_RD_HI();

    return data;
}

/**
 * \brief Read XRAM
 *
 * \param
 */
void xram_write(uint16_t addr, uint8_t data)
{
    /* Set HIGH before switching to output to prevent RD cycle */
    XRAM_CTRL_PORT |= XRAM_RD;
    XRAM_CTRL_DDR |= XRAM_RD;

    /* Set HIGH before switching to output to prevent WR cycle */
    XRAM_CTRL_PORT |= XRAM_WR;
    XRAM_CTRL_DDR |= XRAM_WR;

    /* Set Address */
    PORTB = (uint8_t)(0x00FF & addr);
    DDRB = 0xFF;

    PORTC = (uint8_t)(0x000F & (addr >> 8));
    DDRC |= 0x0F;

    PORTD = (uint8_t)(0x00F0 & (addr >> 8));
    DDRD |= 0xF0;

    XRAM_ALE_DDR |= XRAM_ALE_PIN;
    XRAM_ALE_PORT |= XRAM_ALE_PIN;
    nop();
    nop();
    XRAM_ALE_PORT &= ~XRAM_ALE_PIN;
    XRAM_DATA_SETOUTP();
    XRAM_DATA_PORT = data; // XRAM_FIFO_AD >> 8; /* high byte of FIFO address */
    XRAM_CTRL_WR_LO();
    nop();
    nop();
    XRAM_CTRL_WR_HI();
    XRAM_DATA_SETINP();
}




 void board_identify(void)
{
    uint8_t i;
    uint8_t count = 0;
    mem_test_t mem_vals[NUM_CHECK];
    for (i = 0; i < NUM_CHECK ; i++)
    {
        mem_vals[i].addr = 0x8000 + (i * 10);
        mem_vals[i].val = 4;
        xram_write(mem_vals[i].addr, mem_vals[i].val);
    }
    for (i = 0; i < NUM_CHECK ; i++)
    {
        if (mem_vals[i].val == xram_read(mem_vals[i].addr))
        {
            count++;
        }
    }
    if (count)
    {
        board_type = SENSOR_TERM_BOARD;

    }
    else
    {
        board_type = PLAIN;    
    }   


}
#ifdef  SENSOR_TERMINAL_BOARD
bool stb_button_read(void)
{

switch (board_type)
    {
        case SENSOR_TERM_BOARD:
            {
                uint8_t cur_button_state;

                /*
                 * Enable button address decoding.
                 * This is similar to USB, but with other settings.
                 */
                BUTTON_ADDR_DEC_PORT |= _BV(6);    // Different to USB
                BUTTON_ADDR_DEC_DDR |= _BV(6);
                BUTTON_ADDR_DEC_PORT &= ~_BV(7);
                BUTTON_ADDR_DEC_DDR |= _BV(7);


                PORTE &= ~_BV(5);
                DDRE |= _BV(5);

                /* Switch port to input. */
                BUTTON_PORT |= (1 << BUTTON_PIN_0);
                BUTTON_PORT_DIR &= ~(1 << BUTTON_PIN_0);

                cur_button_state = BUTTON_INPUT_PINS;

                PORTE |= _BV(5);

                /* Switch port back to output. */
                BUTTON_PORT_DIR |= (1 << BUTTON_PIN_0);

                /*
                 * Disable button address decoding.
                 * This enables USB again.
                 */
                BUTTON_ADDR_DEC_PORT &= ~_BV(6);
                BUTTON_ADDR_DEC_DDR |= _BV(6);
                BUTTON_ADDR_DEC_PORT &= ~_BV(7);
                BUTTON_ADDR_DEC_DDR |= _BV(7);

                if (cur_button_state & 0x01)
                {
                    return true;
                }
                else
                {
                    return false;
                }
                //break;
            }

		case PLAIN:
		{
		if (ioport_get_pin_level(GPIO_PUSH_BUTTON_0)) {
		return false;
		} else {
		return true;
			}
        }

        default:
            {
                break;
            }
    }
    return false;

}
#endif

void led_ctrl(led_id_t led_no, led_action_t led_setting)
{
  switch (board_type)
    {
case SENSOR_TERM_BOARD:
{
    uint8_t pin;
    /* New values of LED pins based on new LED state. */
    uint8_t led_pin_value;
    /*
    * Original value of LED port before writing new value.
    * This value needs to be restored.
    */
    uint8_t orig_led_port = LED_PORT & ~LED_BIT_MASK;
    
    /* Both LEDs need to be updated, since several peripherals
    * are dealing with the same port for this board
    * (USB, EEPROM, LEDs, Button).
    */
    LED_PORT_DIR |= (1 << LED_BIT_0);
    LED_PORT_DIR |= (1 << LED_BIT_1);
    if (led_no == LED_2)
      led_no = LED_0;
    switch (led_no)
    {
    case LED_0:
    pin = LED_BIT_0;
    break;
    case LED_1:
    pin = LED_BIT_1;
    break;
    default:
    return;
    }
    
    switch (led_setting)
    {
    case LED_ON:
    led_state |= _BV(pin);
    break;
    
    case LED_OFF:
    led_state &= ~_BV(pin);
    break;
    
    case LED_TOGGLE:
    default:
    if (led_state & _BV(pin))
    {
    /*
    * LED is currently on,
    * Switch it off
    */
    led_state &= ~_BV(pin);
    }
    else
    {
    /*
    * LED is currently off,
    * Switch it on
    */
    led_state |= _BV(pin);
    }
    break;
    }
    
    led_pin_value = (uint8_t)(~(uint16_t)led_state);  // Implicit casting required to avoid IAR Pa091.
    led_pin_value &= LED_BIT_MASK;
    
    LED_PORT = orig_led_port | led_pin_value;
    
    led_helper_func();
    break;
}

case PLAIN:
{

    switch (led_setting)
        {
        case LED_ON:
          if(led_no == LED_0)
        gpio_set_pin_low(LED0_RCB);
          else if(led_no == LED_1)
        gpio_set_pin_low(LED1_RCB);
          else
        gpio_set_pin_low(LED2_RCB);
        break;
        
        case LED_OFF:
          if(led_no == LED_0)
        gpio_set_pin_high(LED0_RCB);
          else if(led_no == LED_1)
        gpio_set_pin_high(LED1_RCB);
          else
        gpio_set_pin_high(LED2_RCB);
        break;
        
        case LED_TOGGLE:
          if(led_no == LED_0)
        gpio_toggle_pin(LED0_RCB);
          else if(led_no == LED_1)
        gpio_toggle_pin(LED1_RCB);
          else
        gpio_toggle_pin(LED2_RCB);
        break;
        }
    }
}
}


void led_helper_func(void)
{
    /*
     * Enable LED address decoding.
     * This is similar to USB, but with other settings.
     */
    LED_ADDR_DEC_PORT |= _BV(6);    // Different to USB
    LED_ADDR_DEC_DDR |= _BV(6);
    LED_ADDR_DEC_PORT &= ~_BV(7);
    LED_ADDR_DEC_DDR |= _BV(7);

    /* Enable desired LED state. */
    DDRE |= _BV(4);
    /* Set PE4 to 0 and back to 1. */
    PORTE &= ~_BV(4);
    PORTE |= _BV(4);

    /*
     * Disable LED address decoding.
     * This enables USB again.
     */
    LED_ADDR_DEC_PORT &= ~_BV(6);
    LED_ADDR_DEC_DDR |= _BV(6);
    LED_ADDR_DEC_PORT &= ~_BV(7);
    LED_ADDR_DEC_DDR |= _BV(7);
}
