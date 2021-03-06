/**
 * \file
 *
 * \brief User Interface.
 *
 * Copyright (c) 2012-2015 Atmel Corporation. All rights reserved.
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
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include "ui.h"

//
#define UI_IDLE_TIME         (156u)

// SAM4L status structure for the board monitor
volatile sam4l_status_t sam4l_status = {
	.power_scaling = POWER_SCALING_PS0,
	.sleep_mode = SLEEP_MODE_RUN,
	.cpu_freq = 12000000,
	.cpu_src = CPU_SRC_RC4M,
};

extern volatile uint32_t event_qtouch_sensors_idle_count;

/**
 * \brief Set MCU power saving information used by the UI.
 *
 * \param power_scaling Power scaling.
 * \param sleep_mode Sleep mode.
 * \param cpu_freq CPU frequency.
 * \param cpu_src CPU source clock.
 */
void ui_set_mcu_status(power_scaling_t power_scaling,
	sleep_mode_t sleep_mode, uint32_t cpu_freq, cpu_src_t cpu_src)
{
	sam4l_status.power_scaling = power_scaling;
	sam4l_status.sleep_mode = sleep_mode;
	sam4l_status.cpu_freq = cpu_freq;
	sam4l_status.cpu_src = cpu_src;
}

/**
 * \brief Get MCU Power Scaling Status.
 */
power_scaling_t ui_get_power_scaling_mcu_status(void)
{
	return sam4l_status.power_scaling;
}

/**
 * \brief Set MCU Power Scaling Status.
 * \param power_scaling Power scaling.
 */
void ui_set_power_scaling_mcu_status(power_scaling_t power_scaling)
{
	sam4l_status.power_scaling = power_scaling;
}

/**
 * \brief Get MCU Sleep Mode Status.
 */
sleep_mode_t ui_get_sleep_mode_mcu_status(void)
{
	return sam4l_status.sleep_mode;
}

/**
 * \brief Set MCU Sleep Mode Status.
 * \param sleep_mode Sleep Mode.
 */
void ui_set_sleep_mode_mcu_status(sleep_mode_t sleep_mode)
{
	sam4l_status.sleep_mode = sleep_mode;
}
/**
 * \brief User Interface - Board Monitor Initialization :
 *  and send SAM4L status.
 */
void ui_bm_init(void)
{
	/*
	 * Initialize Board Monitor and send first status
	 */
	sysclk_enable_peripheral_clock(BM_USART_USART);
	bm_init();
	sysclk_disable_peripheral_clock(BM_USART_USART);
	ui_bm_send_mcu_status();
}


/**
 * \brief User Interface Board Monitor send SAM4L status.
 */
void ui_bm_send_mcu_status(void)
{
	uint32_t power_scaling, sleep_mode, cpu_freq, cpu_src;
	sysclk_enable_peripheral_clock(BM_USART_USART);
	power_scaling = sam4l_status.power_scaling;
	sleep_mode = sam4l_status.sleep_mode;
	cpu_freq = sam4l_status.cpu_freq;
	cpu_src = sam4l_status.cpu_src;
	bm_send_mcu_status(power_scaling, sleep_mode, cpu_freq, cpu_src);
	sysclk_disable_peripheral_clock(BM_USART_USART);
}

/**
 * \brief User Interface - LCD Initialization.
 */
void ui_lcd_init(void)
{
	uint8_t const scrolling_str[] = "SAM4L-EK DEMO";

	/*
	 * LCDCA Controller Initialization and display SAM4L-EK DEMO texts on
	 * segment LCD
	 */

	// Initialize the C42364A LCD glass component.
	c42364a_init();

	// Start autonomous animation.
	c42364a_circular_animation_start(C42364A_CSR_RIGHT, 7, 0x03);
	// Show ARM Icon.
	c42364a_show_icon(C42364A_ICON_ARM);
	// Start scrolling text.
	c42364a_text_scrolling_start(scrolling_str,
			strlen((char const *)scrolling_str));

	while(event_qtouch_sensors_idle_count<UI_IDLE_TIME){}
	event_qtouch_sensors_idle_count = 0;

	// Stop scrolling text.
	c42364a_text_scrolling_stop();

	ui_lcd_refresh_txt();
}
/**
 * \brief User Interface LCD Refresh Alphanumeric area.
 * \param ui_lcd_refresh boolean to refresh or not Alphanumeric area.
 * \param event_qtouch_slider_position set slider position in Alphanumeric area.
 */
void ui_lcd_refresh_alphanum(bool ui_lcd_refresh,
	int32_t event_qtouch_slider_position)
{
	char  string_info[8];
	if (ui_lcd_refresh == true ) {
		sprintf(string_info, "%d", (int)event_qtouch_slider_position);
		if (event_qtouch_slider_position < 100) {
			string_info[2] = ' ';
			string_info[3] = ' ';
			string_info[4] = '\0';
		}
		// display slider position on segment LCD
		c42364a_write_num_packet((uint8_t const*)&string_info);
	} else {
		// Clear digit area
		string_info[0] = ' ';
		string_info[1] = ' ';
		string_info[2] = ' ';
		string_info[3] = ' ';
		string_info[4] = '\0';
		// display slider position on segment LCD
		c42364a_write_num_packet((uint8_t const*)&string_info);
	}
}
/**
 * \brief User Interface LCD Refresh Text area.
 */
void ui_lcd_refresh_txt(void)
{
	char  string_info[8];
	power_scaling_t ps_status = ui_get_power_scaling_mcu_status();

	// Display Power Scaling mode on segment LCD
	if (ps_status == POWER_SCALING_PS1){
		sprintf((char*)string_info, "RUN PS1");
	} else if (ps_status == POWER_SCALING_PS2) {
		sprintf((char*)string_info, "RUN PS2");
	} else {
		sprintf((char*)string_info, "RUN PS0");
	}
	c42364a_show_text((const uint8_t *)string_info);
}
