/**
 * \file
 *
 * \brief API driver for HX8347A TFT display component.
 *
 * Copyright (c) 2011-2015 Atmel Corporation. All rights reserved.
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

#ifndef HX8347A_H_INCLUDED
#define HX8347A_H_INCLUDED

#include "compiler.h"
#include "board.h"
#include "conf_hx8347a.h"

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
 extern "C" {
#endif
/**INDENT-ON**/
/// @endcond

/* HX8347A screen size */
#define HX8347A_LCD_WIDTH  240
#define HX8347A_LCD_HEIGHT 320

/* HX8347A ID code */
#define HX8347A_DEVICE_CODE (0x47u)

/* HX8347A LCD Registers */

/* -------- HX8347A_DISP_MODE_CTRL : (Offset: 0x01) Display Mode Control -------- */
#define HX8347A_DISP_MODE_CTRL (0x01u)
#define HX8347A_DISP_MODE_CTRL_PTLON (0x1u << 0)
#define HX8347A_DISP_MODE_CTRL_NORON (0x1u << 1)
#define HX8347A_DISP_MODE_CTRL_INVON (0x1u << 2)
#define HX8347A_DISP_MODE_CTRL_IDMON (0x1u << 3)
/* -------- HX8347A_COL_ADDR_START2 : (Offset: 0x02) Column Address Start 2 -------- */
#define HX8347A_COL_ADDR_START2 (0x02u)
#define HX8347A_COL_ADDR_START2_SC_POS 0
#define HX8347A_COL_ADDR_START2_SC_MSK (0xffu << HX8347A_COL_ADDR_START2_SC_POS)
#define HX8347A_COL_ADDR_START2_SC(value) ((HX8347A_COL_ADDR_START2_SC_MSK & ((value) << HX8347A_COL_ADDR_START2_SC_POS)))
/* -------- HX8347A_COL_ADDR_START1 : (Offset: 0x03) Column Address Start 1 -------- */
#define HX8347A_COL_ADDR_START1 (0x03u)
#define HX8347A_COL_ADDR_START1_SC_POS 0
#define HX8347A_COL_ADDR_START1_SC_MSK (0xffu << HX8347A_COL_ADDR_START1_SC_POS)
#define HX8347A_COL_ADDR_START1_SC(value) ((HX8347A_COL_ADDR_START1_SC_MSK & ((value) << HX8347A_COL_ADDR_START1_SC_POS)))
/* -------- HX8347A_COL_ADDR_END2 : (Offset: 0x04) Column Address End 2 -------- */
#define HX8347A_COL_ADDR_END2 (0x04u)
#define HX8347A_COL_ADDR_END2_EC_POS 0
#define HX8347A_COL_ADDR_END2_EC_MSK (0xffu << HX8347A_COL_ADDR_END2_EC_POS)
#define HX8347A_COL_ADDR_END2_EC(value) ((HX8347A_COL_ADDR_END2_EC_MSK & ((value) << HX8347A_COL_ADDR_END2_EC_POS)))
/* -------- HX8347A_COL_ADDR_END1 : (Offset: 0x05) Column Address End 1 -------- */
#define HX8347A_COL_ADDR_END1 (0x05u)
#define HX8347A_COL_ADDR_END1_EC_POS 0
#define HX8347A_COL_ADDR_END1_EC_MSK (0xffu << HX8347A_COL_ADDR_END1_EC_POS)
#define HX8347A_COL_ADDR_END1_EC(value) ((HX8347A_COL_ADDR_END1_EC_MSK & ((value) << HX8347A_COL_ADDR_END1_EC_POS)))
/* -------- HX8347A_ROW_ADDR_START2 : (Offset: 0x06) Row Address Start 2 -------- */
#define HX8347A_ROW_ADDR_START2 (0x06u)
#define HX8347A_ROW_ADDR_START2_SP_POS 0
#define HX8347A_ROW_ADDR_START2_SP_MSK (0xffu << HX8347A_ROW_ADDR_START2_SP_POS)
#define HX8347A_ROW_ADDR_START2_SP(value) ((HX8347A_ROW_ADDR_START2_SP_MSK & ((value) << HX8347A_ROW_ADDR_START2_SP_POS)))
/* -------- HX8347A_ROW_ADDR_START1 : (Offset: 0x07) Row Address Start 1 -------- */
#define HX8347A_ROW_ADDR_START1 (0x07u)
#define HX8347A_ROW_ADDR_START1_SP_POS 0
#define HX8347A_ROW_ADDR_START1_SP_MSK (0xffu << HX8347A_ROW_ADDR_START1_SP_POS)
#define HX8347A_ROW_ADDR_START1_SP(value) ((HX8347A_ROW_ADDR_START1_SP_MSK & ((value) << HX8347A_ROW_ADDR_START1_SP_POS)))
/* -------- HX8347A_ROW_ADDR_END2 : (Offset: 0x08) Row address End 2 -------- */
#define HX8347A_ROW_ADDR_END2 (0x08u)
#define HX8347A_ROW_ADDR_END2_EP_POS 0
#define HX8347A_ROW_ADDR_END2_EP_MSK (0xffu << HX8347A_ROW_ADDR_END2_EP_POS)
#define HX8347A_ROW_ADDR_END2_EP(value) ((HX8347A_ROW_ADDR_END2_EP_MSK & ((value) << HX8347A_ROW_ADDR_END2_EP_POS)))
/* -------- HX8347A_ROW_ADDR_END1 : (Offset: 0x09) Row address End 1 -------- */
#define HX8347A_ROW_ADDR_END1 (0x09u)
#define HX8347A_ROW_ADDR_END1_EP_POS 0
#define HX8347A_ROW_ADDR_END1_EP_MSK (0xffu << HX8347A_ROW_ADDR_END1_EP_POS)
#define HX8347A_ROW_ADDR_END1_EP(value) ((HX8347A_ROW_ADDR_END1_EP_MSK & ((value) << HX8347A_ROW_ADDR_END1_EP_POS)))
/* -------- HX8347A_PARTIAL_AREA_START_ROW2 : (Offset: 0x0A) Partial Area Start Row 2 -------- */
#define HX8347A_PARTIAL_AREA_START_ROW2 (0x0Au)
#define HX8347A_PARTIAL_AREA_START_ROW2_PSL_POS 0
#define HX8347A_PARTIAL_AREA_START_ROW2_PSL_MSK (0xffu << HX8347A_PARTIAL_AREA_START_ROW2_PSL_POS)
#define HX8347A_PARTIAL_AREA_START_ROW2_PSL(value) ((HX8347A_PARTIAL_AREA_START_ROW2_PSL_MSK & ((value) << HX8347A_PARTIAL_AREA_START_ROW2_PSL_POS)))
/* -------- HX8347A_PARTIAL_AREA_START_ROW1 : (Offset: 0x0B) Partial Area Start Row 1 -------- */
#define HX8347A_PARTIAL_AREA_START_ROW1 (0x0Bu)
#define HX8347A_PARTIAL_AREA_START_ROW1_PSL_POS 0
#define HX8347A_PARTIAL_AREA_START_ROW1_PSL_MSK (0xffu << HX8347A_PARTIAL_AREA_START_ROW1_PSL_POS)
#define HX8347A_PARTIAL_AREA_START_ROW1_PSL(value) ((HX8347A_PARTIAL_AREA_START_ROW1_PSL_MSK & ((value) << HX8347A_PARTIAL_AREA_START_ROW1_PSL_POS)))
/* -------- HX8347A_PARTIAL_AREA_END_ROW2 : (Offset: 0x0C) Partial Area End Row 2 -------- */
#define HX8347A_PARTIAL_AREA_END_ROW2 (0x0Cu)
#define HX8347A_PARTIAL_AREA_END_ROW2_PEL_POS 0
#define HX8347A_PARTIAL_AREA_END_ROW2_PEL_MSK (0xffu << HX8347A_PARTIAL_AREA_END_ROW2_PEL_POS)
#define HX8347A_PARTIAL_AREA_END_ROW2_PEL(value) ((HX8347A_PARTIAL_AREA_END_ROW2_PEL_MSK & ((value) << HX8347A_PARTIAL_AREA_END_ROW2_PEL_POS)))
/* -------- HX8347A_PARTIAL_AREA_END_ROW1 : (Offset: 0x0D) Partial Area End Row 1 -------- */
#define HX8347A_PARTIAL_AREA_END_ROW1 (0x0Du)
#define HX8347A_PARTIAL_AREA_END_ROW1_PEL_POS 0
#define HX8347A_PARTIAL_AREA_END_ROW1_PEL_MSK (0xffu << HX8347A_PARTIAL_AREA_END_ROW1_PEL_POS)
#define HX8347A_PARTIAL_AREA_END_ROW1_PEL(value) ((HX8347A_PARTIAL_AREA_END_ROW1_PEL_MSK & ((value) << HX8347A_PARTIAL_AREA_END_ROW1_PEL_POS)))
/* -------- HX8347A_VERTICAL_SCROLL_TOP_FIXED_AREA2 : (Offset: 0x0E) Vertical Scroll Top Fixed Area 2 -------- */
#define HX8347A_VERTICAL_SCROLL_TOP_FIXED_AREA2 (0x0Eu)
#define HX8347A_VERTICAL_SCROLL_TOP_FIXED_AREA2_TFA_POS 0
#define HX8347A_VERTICAL_SCROLL_TOP_FIXED_AREA2_TFA_MSK (0xffu << HX8347A_VERTICAL_SCROLL_TOP_FIXED_AREA2_TFA_POS)
#define HX8347A_VERTICAL_SCROLL_TOP_FIXED_AREA2_TFA(value) ((HX8347A_VERTICAL_SCROLL_TOP_FIXED_AREA2_TFA_MSK & ((value) << HX8347A_VERTICAL_SCROLL_TOP_FIXED_AREA2_TFA_POS)))
/* -------- HX8347A_VERTICAL_SCROLL_TOP_FIXED_AREA1 : (Offset: 0x0F) Vertical Scroll Top Fixed Area 1 -------- */
#define HX8347A_VERTICAL_SCROLL_TOP_FIXED_AREA1 (0x0Fu)
#define HX8347A_VERTICAL_SCROLL_TOP_FIXED_AREA1_TFA_POS 0
#define HX8347A_VERTICAL_SCROLL_TOP_FIXED_AREA1_TFA_MSK (0xffu << HX8347A_VERTICAL_SCROLL_TOP_FIXED_AREA1_TFA_POS)
#define HX8347A_VERTICAL_SCROLL_TOP_FIXED_AREA1_TFA(value) ((HX8347A_VERTICAL_SCROLL_TOP_FIXED_AREA1_TFA_MSK & ((value) << HX8347A_VERTICAL_SCROLL_TOP_FIXED_AREA1_TFA_POS)))
/* -------- HX8347A_VERTICAL_SCROLL_HEIGHT_AREA2 : (Offset: 0x10) Vertical Scroll Height Area 2 -------- */
#define HX8347A_VERTICAL_SCROLL_HEIGHT_AREA2 (0x10u)
#define HX8347A_VERTICAL_SCROLL_HEIGHT_AREA2_VSA_POS 0
#define HX8347A_VERTICAL_SCROLL_HEIGHT_AREA2_VSA_MSK (0xffu << HX8347A_VERTICAL_SCROLL_HEIGHT_AREA2_VSA_POS)
#define HX8347A_VERTICAL_SCROLL_HEIGHT_AREA2_VSA(value) ((HX8347A_VERTICAL_SCROLL_HEIGHT_AREA2_VSA_MSK & ((value) << HX8347A_VERTICAL_SCROLL_HEIGHT_AREA2_VSA_POS)))
/* -------- HX8347A_VERTICAL_SCROLL_HEIGHT_AREA1 : (Offset: 0x11) Vertical Scroll Height Area 1 -------- */
#define HX8347A_VERTICAL_SCROLL_HEIGHT_AREA1 (0x11u)
#define HX8347A_VERTICAL_SCROLL_HEIGHT_AREA1_VSA_POS 0
#define HX8347A_VERTICAL_SCROLL_HEIGHT_AREA1_VSA_MSK (0xffu << HX8347A_VERTICAL_SCROLL_HEIGHT_AREA1_VSA_POS)
#define HX8347A_VERTICAL_SCROLL_HEIGHT_AREA1_VSA(value) ((HX8347A_VERTICAL_SCROLL_HEIGHT_AREA1_VSA_MSK & ((value) << HX8347A_VERTICAL_SCROLL_HEIGHT_AREA1_VSA_POS)))
/* -------- HX8347A_VERTICAL_SCROLL_BUTTON_AREA2 : (Offset: 0x12) Vertical Scroll Button Area 2 -------- */
#define HX8347A_VERTICAL_SCROLL_BUTTON_AREA2 (0x12u)
#define HX8347A_VERTICAL_SCROLL_BUTTON_AREA2_BFA_POS 0
#define HX8347A_VERTICAL_SCROLL_BUTTON_AREA2_BFA_MSK (0xffu << HX8347A_VERTICAL_SCROLL_BUTTON_AREA2_BFA_POS)
#define HX8347A_VERTICAL_SCROLL_BUTTON_AREA2_BFA(value) ((HX8347A_VERTICAL_SCROLL_BUTTON_AREA2_BFA_MSK & ((value) << HX8347A_VERTICAL_SCROLL_BUTTON_AREA2_BFA_POS)))
/* -------- HX8347A_VERTICAL_SCROLL_BUTTON_AREA1 : (Offset: 0x13) Vertical Scroll Button Area 1 -------- */
#define HX8347A_VERTICAL_SCROLL_BUTTON_AREA1 (0x13u)
#define HX8347A_VERTICAL_SCROLL_BUTTON_AREA1_BFA_POS 0
#define HX8347A_VERTICAL_SCROLL_BUTTON_AREA1_BFA_MSK (0xffu << HX8347A_VERTICAL_SCROLL_BUTTON_AREA1_BFA_POS)
#define HX8347A_VERTICAL_SCROLL_BUTTON_AREA1_BFA(value) ((HX8347A_VERTICAL_SCROLL_BUTTON_AREA1_BFA_MSK & ((value) << HX8347A_VERTICAL_SCROLL_BUTTON_AREA1_BFA_POS)))
/* -------- HX8347A_VERTICAL_SCROLL_START_ADDR2 : (Offset: 0x14) Vertical Scroll Start Address 2 -------- */
#define HX8347A_VERTICAL_SCROLL_START_ADDR2 (0x14u)
#define HX8347A_VERTICAL_SCROLL_START_ADDR2_VSP_POS 0
#define HX8347A_VERTICAL_SCROLL_START_ADDR2_VSP_MSK (0xffu << HX8347A_VERTICAL_SCROLL_START_ADDR2_VSP_POS)
#define HX8347A_VERTICAL_SCROLL_START_ADDR2_VSP(value) ((HX8347A_VERTICAL_SCROLL_START_ADDR2_VSP_MSK & ((value) << HX8347A_VERTICAL_SCROLL_START_ADDR2_VSP_POS)))
/* -------- HX8347A_VERTICAL_SCROLL_START_ADDR1 : (Offset: 0x15) Vertical Scroll Start Address 1 -------- */
#define HX8347A_VERTICAL_SCROLL_START_ADDR1 (0x15u)
#define HX8347A_VERTICAL_SCROLL_START_ADDR1_VSP_POS 0
#define HX8347A_VERTICAL_SCROLL_START_ADDR1_VSP_MSK (0xffu << HX8347A_VERTICAL_SCROLL_START_ADDR1_VSP_POS)
#define HX8347A_VERTICAL_SCROLL_START_ADDR1_VSP(value) ((HX8347A_VERTICAL_SCROLL_START_ADDR1_VSP_MSK & ((value) << HX8347A_VERTICAL_SCROLL_START_ADDR1_VSP_POS)))
/* -------- HX8347A_MEMORY_ACCESS_CTRL : (Offset: 0x16) Memory Access Control -------- */
#define HX8347A_MEMORY_ACCESS_CTRL (0x16u)
#define HX8347A_MEMORY_ACCESS_CTRL_BGR (0x1u << 3)
#define HX8347A_MEMORY_ACCESS_CTRL_MV (0x1u << 5)
#define HX8347A_MEMORY_ACCESS_CTRL_MX (0x1u << 6)
#define HX8347A_MEMORY_ACCESS_CTRL_MY (0x1u << 7)
/* -------- HX8347A_GATE_SCAN_CTRL : (Offset: 0x18) Gate Scan Control -------- */
#define HX8347A_GATE_SCAN_CTRL (0x18u)
#define HX8347A_GATE_SCAN_CTRL_SM (0x1u << 0)
#define HX8347A_GATE_SCAN_CTRL_SCROLL_ON (0x1u << 1)
/* -------- HX8347A_OSC_CTRL1 : (Offset: 0x19) OSC Control 1 -------- */
#define HX8347A_OSC_CTRL1 (0x19u)
#define HX8347A_OSC_CTRL1_OSC_EN (0x1u << 0)
#define HX8347A_OSC_CTRL1_CUADJ_POS 1
#define HX8347A_OSC_CTRL1_CUADJ_MSK (0x7u << HX8347A_OSC_CTRL1_CUADJ_POS)
#define HX8347A_OSC_CTRL1_CUADJ(value) ((HX8347A_OSC_CTRL1_CUADJ_MSK & ((value) << HX8347A_OSC_CTRL1_CUADJ_POS)))
#define HX8347A_OSC_CTRL1_CADJ_POS 4
#define HX8347A_OSC_CTRL1_CADJ_MSK (0xfu << HX8347A_OSC_CTRL1_CADJ_POS)
#define HX8347A_OSC_CTRL1_CADJ(value) ((HX8347A_OSC_CTRL1_CADJ_MSK & ((value) << HX8347A_OSC_CTRL1_CADJ_POS)))
/* -------- HX8347A_OSC_CTRL2 : (Offset: 0x1A) OSC Control 2 -------- */
#define HX8347A_OSC_CTRL2 (0x1Au)
#define HX8347A_OSC_CTRL2_OSC_TEST (0x1u << 0)
/* -------- HX8347A_POWER_CTRL1 : (Offset: 0x1B) Power Control 1 -------- */
#define HX8347A_POWER_CTRL1 (0x1Bu)
#define HX8347A_POWER_CTRL1_STB (0x1u << 0)
#define HX8347A_POWER_CTRL1_VLCD_TRI (0x1u << 1)
#define HX8347A_POWER_CTRL1_XDK (0x1u << 2)
#define HX8347A_POWER_CTRL1_DK (0x1u << 3)
#define HX8347A_POWER_CTRL1_PON (0x1u << 4)
#define HX8347A_POWER_CTRL1_GASENB (0x1u << 7)
/* -------- HX8347A_POWER_CTRL2 : (Offset: 0x1C) Power Control 2 -------- */
#define HX8347A_POWER_CTRL2 (0x1Cu)
#define HX8347A_POWER_CTRL2_AP_POS 0
#define HX8347A_POWER_CTRL2_AP_MSK (0x7u << HX8347A_POWER_CTRL2_AP_POS)
#define HX8347A_POWER_CTRL2_AP(value) ((HX8347A_POWER_CTRL2_AP_MSK & ((value) << HX8347A_POWER_CTRL2_AP_POS)))
/* -------- HX8347A_POWER_CTRL3 : (Offset: 0x1D) Power Control 3 -------- */
#define HX8347A_POWER_CTRL3 (0x1Du)
#define HX8347A_POWER_CTRL3_VC1_POS 0
#define HX8347A_POWER_CTRL3_VC1_MSK (0x7u << HX8347A_POWER_CTRL3_VC1_POS)
#define HX8347A_POWER_CTRL3_VC1(value) ((HX8347A_POWER_CTRL3_VC1_MSK & ((value) << HX8347A_POWER_CTRL3_VC1_POS)))
/* -------- HX8347A_POWER_CTRL4 : (Offset: 0x1E) Power Control 4 -------- */
#define HX8347A_POWER_CTRL4 (0x1Eu)
#define HX8347A_POWER_CTRL4_VC3_POS 0
#define HX8347A_POWER_CTRL4_VC3_MSK (0x7u << HX8347A_POWER_CTRL4_VC3_POS)
#define HX8347A_POWER_CTRL4_VC3(value) ((HX8347A_POWER_CTRL4_VC3_MSK & ((value) << HX8347A_POWER_CTRL4_VC3_POS)))
/* -------- HX8347A_POWER_CTRL5 : (Offset: 0x1F) Power Control 5 -------- */
#define HX8347A_POWER_CTRL5 (0x1Fu)
#define HX8347A_POWER_CTRL5_VRH_POS 0
#define HX8347A_POWER_CTRL5_VRH_MSK (0xfu << HX8347A_POWER_CTRL5_VRH_POS)
#define HX8347A_POWER_CTRL5_VRH(value) ((HX8347A_POWER_CTRL5_VRH_MSK & ((value) << HX8347A_POWER_CTRL5_VRH_POS)))
/* -------- HX8347A_POWER_CTRL6 : (Offset: 0x20) Power Control 6 -------- */
#define HX8347A_POWER_CTRL6 (0x20u)
#define HX8347A_POWER_CTRL6_BT_POS 4
#define HX8347A_POWER_CTRL6_BT_MSK (0xfu << HX8347A_POWER_CTRL6_BT_POS)
#define HX8347A_POWER_CTRL6_BT(value) ((HX8347A_POWER_CTRL6_BT_MSK & ((value) << HX8347A_POWER_CTRL6_BT_POS)))
/* -------- HX8347A_POWER_CTRL7 : (Offset: 0x21) Power Control 7 -------- */
#define HX8347A_POWER_CTRL7 (0x21u)
#define HX8347A_POWER_CTRL7_FS0_POS 0
#define HX8347A_POWER_CTRL7_FS0_MSK (0x3u << HX8347A_POWER_CTRL7_FS0_POS)
#define HX8347A_POWER_CTRL7_FS0(value) ((HX8347A_POWER_CTRL7_FS0_MSK & ((value) << HX8347A_POWER_CTRL7_FS0_POS)))
#define HX8347A_POWER_CTRL7_FS1_POS 4
#define HX8347A_POWER_CTRL7_FS1_MSK (0x3u << HX8347A_POWER_CTRL7_FS1_POS)
#define HX8347A_POWER_CTRL7_FS1(value) ((HX8347A_POWER_CTRL7_FS1_MSK & ((value) << HX8347A_POWER_CTRL7_FS1_POS)))
/* -------- HX8347A_SRAM_WRITE_CTRL : (Offset: 0x22) SRAM Write Control -------- */
#define HX8347A_SRAM_WRITE_CTRL (0x22u)
/* -------- HX8347A_CYCLE_CTRL_1 : (Offset: 0x23) Cycle Control 1 -------- */
#define HX8347A_CYCLE_CTRL_1 (0x23u)
#define HX8347A_CYCLE_CTRL_1_N_DC_POS 0
#define HX8347A_CYCLE_CTRL_1_N_DC_MSK (0xffu << HX8347A_CYCLE_CTRL_1_N_DC_POS)
#define HX8347A_CYCLE_CTRL_1_N_DC(value) ((HX8347A_CYCLE_CTRL_1_N_DC_MSK & ((value) << HX8347A_CYCLE_CTRL_1_N_DC_POS)))
/* -------- HX8347A_CYCLE_CTRL_2 : (Offset: 0x24) Cycle Control 2 -------- */
#define HX8347A_CYCLE_CTRL_2 (0x24u)
#define HX8347A_CYCLE_CTRL_2_PI_DC_POS 0
#define HX8347A_CYCLE_CTRL_2_PI_DC_MSK (0xffu << HX8347A_CYCLE_CTRL_2_PI_DC_POS)
#define HX8347A_CYCLE_CTRL_2_PI_DC(value) ((HX8347A_CYCLE_CTRL_2_PI_DC_MSK & ((value) << HX8347A_CYCLE_CTRL_2_PI_DC_POS)))
/* -------- HX8347A_CYCLE_CTRL_3 : (Offset: 0x25) Cycle Control 3 -------- */
#define HX8347A_CYCLE_CTRL_3 (0x25u)
#define HX8347A_CYCLE_CTRL_3_I_DC_POS 0
#define HX8347A_CYCLE_CTRL_3_I_DC_MSK (0xffu << HX8347A_CYCLE_CTRL_3_I_DC_POS)
#define HX8347A_CYCLE_CTRL_3_I_DC(value) ((HX8347A_CYCLE_CTRL_3_I_DC_MSK & ((value) << HX8347A_CYCLE_CTRL_3_I_DC_POS)))
/* -------- HX8347A_DISP_CTRL1 : (Offset: 0x26) Display Control 1 -------- */
#define HX8347A_DISP_CTRL1 (0x26u)
#define HX8347A_DISP_CTRL1_D_POS 2
#define HX8347A_DISP_CTRL1_D_MSK (0x3u << HX8347A_DISP_CTRL1_D_POS)
#define HX8347A_DISP_CTRL1_D(value) ((HX8347A_DISP_CTRL1_D_MSK & ((value) << HX8347A_DISP_CTRL1_D_POS)))
#define HX8347A_DISP_CTRL1_DTE (0x1u << 4)
#define HX8347A_DISP_CTRL1_GON (0x1u << 5)
#define HX8347A_DISP_CTRL1_PT_POS 6
#define HX8347A_DISP_CTRL1_PT_MSK (0x3u << HX8347A_DISP_CTRL1_PT_POS)
#define HX8347A_DISP_CTRL1_PT(value) ((HX8347A_DISP_CTRL1_PT_MSK & ((value) << HX8347A_DISP_CTRL1_PT_POS)))
/* -------- HX8347A_DISP_CTRL2 : (Offset: 0x27) Display Control 2 -------- */
#define HX8347A_DISP_CTRL2 (0x27u)
#define HX8347A_DISP_CTRL2_N_BP_POS 0
#define HX8347A_DISP_CTRL2_N_BP_MSK (0xfu << HX8347A_DISP_CTRL2_N_BP_POS)
#define HX8347A_DISP_CTRL2_N_BP(value) ((HX8347A_DISP_CTRL2_N_BP_MSK & ((value) << HX8347A_DISP_CTRL2_N_BP_POS)))
/* -------- HX8347A_DISP_CTRL3 : (Offset: 0x28) Display Control 3 -------- */
#define HX8347A_DISP_CTRL3 (0x28u)
#define HX8347A_DISP_CTRL3_N_FP_POS 0
#define HX8347A_DISP_CTRL3_N_FP_MSK (0xfu << HX8347A_DISP_CTRL3_N_FP_POS)
#define HX8347A_DISP_CTRL3_N_FP(value) ((HX8347A_DISP_CTRL3_N_FP_MSK & ((value) << HX8347A_DISP_CTRL3_N_FP_POS)))
/* -------- HX8347A_DISP_CTRL4 : (Offset: 0x29) Display Control 4 -------- */
#define HX8347A_DISP_CTRL4 (0x29u)
#define HX8347A_DISP_CTRL4_PI_BP_POS 0
#define HX8347A_DISP_CTRL4_PI_BP_MSK (0xfu << HX8347A_DISP_CTRL4_PI_BP_POS)
#define HX8347A_DISP_CTRL4_PI_BP(value) ((HX8347A_DISP_CTRL4_PI_BP_MSK & ((value) << HX8347A_DISP_CTRL4_PI_BP_POS)))
/* -------- HX8347A_DISP_CTRL5 : (Offset: 0x2A) Display Control 5 -------- */
#define HX8347A_DISP_CTRL5 (0x2Au)
#define HX8347A_DISP_CTRL5_PI_FP_POS 0
#define HX8347A_DISP_CTRL5_PI_FP_MSK (0xfu << HX8347A_DISP_CTRL5_PI_FP_POS)
#define HX8347A_DISP_CTRL5_PI_FP(value) ((HX8347A_DISP_CTRL5_PI_FP_MSK & ((value) << HX8347A_DISP_CTRL5_PI_FP_POS)))
/* -------- HX8347A_POWER_CTRL11 : (Offset: 0x2B) Power Control 11 -------- */
#define HX8347A_POWER_CTRL11 (0x2Bu)
#define HX8347A_POWER_CTRL11_BLANK_DIV_POS 0
#define HX8347A_POWER_CTRL11_BLANK_DIV_MSK (0xfu << HX8347A_POWER_CTRL11_BLANK_DIV_POS)
#define HX8347A_POWER_CTRL11_BLANK_DIV(value) ((HX8347A_POWER_CTRL11_BLANK_DIV_MSK & ((value) << HX8347A_POWER_CTRL11_BLANK_DIV_POS)))
#define HX8347A_POWER_CTRL11_PI_PRE_REFRESH_POS 4
#define HX8347A_POWER_CTRL11_PI_PRE_REFRESH_MSK (0x3u << HX8347A_POWER_CTRL11_PI_PRE_REFRESH_POS)
#define HX8347A_POWER_CTRL11_PI_PRE_REFRESH(value) ((HX8347A_POWER_CTRL11_PI_PRE_REFRESH_MSK & ((value) << HX8347A_POWER_CTRL11_PI_PRE_REFRESH_POS)))
/* -------- HX8347A_DISP_CTRL6 : (Offset: 0x2C) Display Control 6 -------- */
#define HX8347A_DISP_CTRL6 (0x2Cu)
#define HX8347A_DISP_CTRL6_I_BP_POS 0
#define HX8347A_DISP_CTRL6_I_BP_MSK (0xfu << HX8347A_DISP_CTRL6_I_BP_POS)
#define HX8347A_DISP_CTRL6_I_BP(value) ((HX8347A_DISP_CTRL6_I_BP_MSK & ((value) << HX8347A_DISP_CTRL6_I_BP_POS)))
/* -------- HX8347A_DISP_CTRL7 : (Offset: 0x2D) Display Control 7 -------- */
#define HX8347A_DISP_CTRL7 (0x2Du)
#define HX8347A_DISP_CTRL7_I_FP_POS 0
#define HX8347A_DISP_CTRL7_I_FP_MSK (0xfu << HX8347A_DISP_CTRL7_I_FP_POS)
#define HX8347A_DISP_CTRL7_I_FP(value) ((HX8347A_DISP_CTRL7_I_FP_MSK & ((value) << HX8347A_DISP_CTRL7_I_FP_POS)))
/* -------- HX8347A_DISP_CTRL9 : (Offset: 0x35) Display Control 9 -------- */
#define HX8347A_DISP_CTRL9 (0x35u)
#define HX8347A_DISP_CTRL9_EQS_POS 0
#define HX8347A_DISP_CTRL9_EQS_MSK (0xffu << HX8347A_DISP_CTRL9_EQS_POS)
#define HX8347A_DISP_CTRL9_EQS(value) ((HX8347A_DISP_CTRL9_EQS_MSK & ((value) << HX8347A_DISP_CTRL9_EQS_POS)))
/* -------- HX8347A_DISP_CTRL10 : (Offset: 0x36) Display Control 10 -------- */
#define HX8347A_DISP_CTRL10 (0x36u)
#define HX8347A_DISP_CTRL10_EQP_POS 0
#define HX8347A_DISP_CTRL10_EQP_MSK (0xffu << HX8347A_DISP_CTRL10_EQP_POS)
#define HX8347A_DISP_CTRL10_EQP(value) ((HX8347A_DISP_CTRL10_EQP_MSK & ((value) << HX8347A_DISP_CTRL10_EQP_POS)))
/* -------- HX8347A_DISP_CTRL12 : (Offset: 0x37) Display Control 12 -------- */
#define HX8347A_DISP_CTRL12 (0x37u)
#define HX8347A_DISP_CTRL12_ISC_POS 0
#define HX8347A_DISP_CTRL12_ISC_MSK (0xfu << HX8347A_DISP_CTRL12_ISC_POS)
#define HX8347A_DISP_CTRL12_ISC(value) ((HX8347A_DISP_CTRL12_ISC_MSK & ((value) << HX8347A_DISP_CTRL12_ISC_POS)))
#define HX8347A_DISP_CTRL12_PTG_POS 4
#define HX8347A_DISP_CTRL12_PTG_MSK (0x3u << HX8347A_DISP_CTRL12_PTG_POS)
#define HX8347A_DISP_CTRL12_PTG(value) ((HX8347A_DISP_CTRL12_PTG_MSK & ((value) << HX8347A_DISP_CTRL12_PTG_POS)))
/* -------- HX8347A_RGB_INTERFACE_CTRL1 : (Offset: 0x38) RGB Interface Control 1 -------- */
#define HX8347A_RGB_INTERFACE_CTRL1 (0x38u)
#define HX8347A_RGB_INTERFACE_CTRL1_EPL (0x1u << 0)
#define HX8347A_RGB_INTERFACE_CTRL1_VSPL (0x1u << 1)
#define HX8347A_RGB_INTERFACE_CTRL1_HSPL (0x1u << 2)
#define HX8347A_RGB_INTERFACE_CTRL1_DPL (0x1u << 3)
#define HX8347A_RGB_INTERFACE_CTRL1_RGB_EN (0x1u << 4)
/* -------- HX8347A_RGB_INTERFACE_CTRL2 : (Offset: 0x39) RGB Interface Control 2 -------- */
#define HX8347A_RGB_INTERFACE_CTRL2 (0x39u)
#define HX8347A_RGB_INTERFACE_CTRL2_DOTCLK_DIV_POS 0
#define HX8347A_RGB_INTERFACE_CTRL2_DOTCLK_DIV_MSK (0xffu << HX8347A_RGB_INTERFACE_CTRL2_DOTCLK_DIV_POS)
#define HX8347A_RGB_INTERFACE_CTRL2_DOTCLK_DIV(value) ((HX8347A_RGB_INTERFACE_CTRL2_DOTCLK_DIV_MSK & ((value) << HX8347A_RGB_INTERFACE_CTRL2_DOTCLK_DIV_POS)))
/* -------- HX8347A_CYCLE_CTRL1 : (Offset: 0x3A) Cycle Control 1 -------- */
#define HX8347A_CYCLE_CTRL1 (0x3Au)
#define HX8347A_CYCLE_CTRL1_N_NW_POS 0
#define HX8347A_CYCLE_CTRL1_N_NW_MSK (0x7u << HX8347A_CYCLE_CTRL1_N_NW_POS)
#define HX8347A_CYCLE_CTRL1_N_NW(value) ((HX8347A_CYCLE_CTRL1_N_NW_MSK & ((value) << HX8347A_CYCLE_CTRL1_N_NW_POS)))
#define HX8347A_CYCLE_CTRL1_N_RTN_POS 4
#define HX8347A_CYCLE_CTRL1_N_RTN_MSK (0xfu << HX8347A_CYCLE_CTRL1_N_RTN_POS)
#define HX8347A_CYCLE_CTRL1_N_RTN(value) ((HX8347A_CYCLE_CTRL1_N_RTN_MSK & ((value) << HX8347A_CYCLE_CTRL1_N_RTN_POS)))
/* -------- HX8347A_CYCLE_CTRL2 : (Offset: 0x3B) Cycle Control 2 -------- */
#define HX8347A_CYCLE_CTRL2 (0x3Bu)
#define HX8347A_CYCLE_CTRL2_PI_NW_POS 0
#define HX8347A_CYCLE_CTRL2_PI_NW_MSK (0x7u << HX8347A_CYCLE_CTRL2_PI_NW_POS)
#define HX8347A_CYCLE_CTRL2_PI_NW(value) ((HX8347A_CYCLE_CTRL2_PI_NW_MSK & ((value) << HX8347A_CYCLE_CTRL2_PI_NW_POS)))
#define HX8347A_CYCLE_CTRL2_PI_RTN_POS 4
#define HX8347A_CYCLE_CTRL2_PI_RTN_MSK (0xfu << HX8347A_CYCLE_CTRL2_PI_RTN_POS)
#define HX8347A_CYCLE_CTRL2_PI_RTN(value) ((HX8347A_CYCLE_CTRL2_PI_RTN_MSK & ((value) << HX8347A_CYCLE_CTRL2_PI_RTN_POS)))
/* -------- HX8347A_CYCLE_CTRL3 : (Offset: 0x3C) Cycle Control 3 -------- */
#define HX8347A_CYCLE_CTRL3 (0x3Cu)
#define HX8347A_CYCLE_CTRL3_I_NW_POS 0
#define HX8347A_CYCLE_CTRL3_I_NW_MSK (0x7u << HX8347A_CYCLE_CTRL3_I_NW_POS)
#define HX8347A_CYCLE_CTRL3_I_NW(value) ((HX8347A_CYCLE_CTRL3_I_NW_MSK & ((value) << HX8347A_CYCLE_CTRL3_I_NW_POS)))
#define HX8347A_CYCLE_CTRL3_I_RTN_POS 4
#define HX8347A_CYCLE_CTRL3_I_RTN_MSK (0xfu << HX8347A_CYCLE_CTRL3_I_RTN_POS)
#define HX8347A_CYCLE_CTRL3_I_RTN(value) ((HX8347A_CYCLE_CTRL3_I_RTN_MSK & ((value) << HX8347A_CYCLE_CTRL3_I_RTN_POS)))
/* -------- HX8347A_CYCLE_CTRL4 : (Offset: 0x3D) Cycle Control 4 -------- */
#define HX8347A_CYCLE_CTRL4 (0x3Du)
#define HX8347A_CYCLE_CTRL4_DIV_N_POS 0
#define HX8347A_CYCLE_CTRL4_DIV_N_MSK (0x3u << HX8347A_CYCLE_CTRL4_DIV_N_POS)
#define HX8347A_CYCLE_CTRL4_DIV_N(value) ((HX8347A_CYCLE_CTRL4_DIV_N_MSK & ((value) << HX8347A_CYCLE_CTRL4_DIV_N_POS)))
#define HX8347A_CYCLE_CTRL4_DIV_PI_POS 2
#define HX8347A_CYCLE_CTRL4_DIV_PI_MSK (0x3u << HX8347A_CYCLE_CTRL4_DIV_PI_POS)
#define HX8347A_CYCLE_CTRL4_DIV_PI(value) ((HX8347A_CYCLE_CTRL4_DIV_PI_MSK & ((value) << HX8347A_CYCLE_CTRL4_DIV_PI_POS)))
#define HX8347A_CYCLE_CTRL4_DIV_I_POS 4
#define HX8347A_CYCLE_CTRL4_DIV_I_MSK (0x3u << HX8347A_CYCLE_CTRL4_DIV_I_POS)
#define HX8347A_CYCLE_CTRL4_DIV_I(value) ((HX8347A_CYCLE_CTRL4_DIV_I_MSK & ((value) << HX8347A_CYCLE_CTRL4_DIV_I_POS)))
/* -------- HX8347A_CYCLE_CTRL5 : (Offset: 0x3E) Cycle Control 5 -------- */
#define HX8347A_CYCLE_CTRL5 (0x3Eu)
#define HX8347A_CYCLE_CTRL5_SON_POS 0
#define HX8347A_CYCLE_CTRL5_SON_MSK (0xffu << HX8347A_CYCLE_CTRL5_SON_POS)
#define HX8347A_CYCLE_CTRL5_SON(value) ((HX8347A_CYCLE_CTRL5_SON_MSK & ((value) << HX8347A_CYCLE_CTRL5_SON_POS)))
/* -------- HX8347A_CYCLE_CTRL6 : (Offset: 0x40) Cycle Control 6 -------- */
#define HX8347A_CYCLE_CTRL6 (0x40u)
#define HX8347A_CYCLE_CTRL6_GDON_POS 0
#define HX8347A_CYCLE_CTRL6_GDON_MSK (0xffu << HX8347A_CYCLE_CTRL6_GDON_POS)
#define HX8347A_CYCLE_CTRL6_GDON(value) ((HX8347A_CYCLE_CTRL6_GDON_MSK & ((value) << HX8347A_CYCLE_CTRL6_GDON_POS)))
/* -------- HX8347A_CYCLE_CTRL7 : (Offset: 0x41) Cycle Control 7 -------- */
#define HX8347A_CYCLE_CTRL7 (0x41u)
#define HX8347A_CYCLE_CTRL7_GDOF_POS 0
#define HX8347A_CYCLE_CTRL7_GDOF_MSK (0xffu << HX8347A_CYCLE_CTRL7_GDOF_POS)
#define HX8347A_CYCLE_CTRL7_GDOF(value) ((HX8347A_CYCLE_CTRL7_GDOF_MSK & ((value) << HX8347A_CYCLE_CTRL7_GDOF_POS)))
/* -------- HX8347A_BGP_CTRL : (Offset: 0x42) BGP Control -------- */
#define HX8347A_BGP_CTRL (0x42u)
#define HX8347A_BGP_CTRL_BGP_POS 0
#define HX8347A_BGP_CTRL_BGP_MSK (0xfu << HX8347A_BGP_CTRL_BGP_POS)
#define HX8347A_BGP_CTRL_BGP(value) ((HX8347A_BGP_CTRL_BGP_MSK & ((value) << HX8347A_BGP_CTRL_BGP_POS)))
#define HX8347A_BGP_CTRL_VBGP_OE (0x1u << 4)
/* -------- HX8347A_VCOM_CTRL1 : (Offset: 0x43) VCOM Control 1 -------- */
#define HX8347A_VCOM_CTRL1 (0x43u)
#define HX8347A_VCOM_CTRL1_VCOMG (0x1u << 7)
/* -------- HX8347A_VCOM_CTRL2 : (Offset: 0x44) VCOM Control 2 -------- */
#define HX8347A_VCOM_CTRL2 (0x44u)
#define HX8347A_VCOM_CTRL2_VCM_POS 0
#define HX8347A_VCOM_CTRL2_VCM_MSK (0x7fu << HX8347A_VCOM_CTRL2_VCM_POS)
#define HX8347A_VCOM_CTRL2_VCM(value) ((HX8347A_VCOM_CTRL2_VCM_MSK & ((value) << HX8347A_VCOM_CTRL2_VCM_POS)))
/* -------- HX8347A_VCOM_CTRL3 : (Offset: 0x45) VCOM Control 3 -------- */
#define HX8347A_VCOM_CTRL3 (0x45u)
#define HX8347A_VCOM_CTRL3_VDV_POS 0
#define HX8347A_VCOM_CTRL3_VDV_MSK (0x1fu << HX8347A_VCOM_CTRL3_VDV_POS)
#define HX8347A_VCOM_CTRL3_VDV(value) ((HX8347A_VCOM_CTRL3_VDV_MSK & ((value) << HX8347A_VCOM_CTRL3_VDV_POS)))
/* -------- HX8347A_R_CTRL1 : (Offset: 0x42) r Control 1 -------- */
#define HX8347A_R_CTRL1 (0x46u)
#define HX8347A_R_CTRL1_CP00 (0x1u << 0)
#define HX8347A_R_CTRL1_CP01 (0x1u << 1)
#define HX8347A_R_CTRL1_CP02 (0x1u << 2)
#define HX8347A_R_CTRL1_CP10 (0x1u << 4)
#define HX8347A_R_CTRL1_CP11 (0x1u << 5)
#define HX8347A_R_CTRL1_CP12 (0x1u << 6)
#define HX8347A_R_CTRL1_GSEL (0x1u << 7)
/* -------- HX8347A_R_CTRL2 : (Offset: 0x47) r Control 2 -------- */
#define HX8347A_R_CTRL2 (0x47u)
#define HX8347A_R_CTRL2_CN00 (0x1u << 0)
#define HX8347A_R_CTRL2_CN01 (0x1u << 1)
#define HX8347A_R_CTRL2_CN02 (0x1u << 2)
#define HX8347A_R_CTRL2_CN10 (0x1u << 4)
#define HX8347A_R_CTRL2_CN11 (0x1u << 5)
#define HX8347A_R_CTRL2_CN12 (0x1u << 6)
/* -------- HX8347A_R_CTRL3 : (Offset: 0x48) r Control 3 -------- */
#define HX8347A_R_CTRL3 (0x48u)
#define HX8347A_R_CTRL3_NP00 (0x1u << 0)
#define HX8347A_R_CTRL3_NP01 (0x1u << 1)
#define HX8347A_R_CTRL3_NP02 (0x1u << 2)
#define HX8347A_R_CTRL3_NP10 (0x1u << 4)
#define HX8347A_R_CTRL3_NP11 (0x1u << 5)
#define HX8347A_R_CTRL3_NP12 (0x1u << 6)
/* -------- HX8347A_R_CTRL4 : (Offset: 0x49) r Control 4 -------- */
#define HX8347A_R_CTRL4 (0x49u)
#define HX8347A_R_CTRL4_NP20 (0x1u << 0)
#define HX8347A_R_CTRL4_NP21 (0x1u << 1)
#define HX8347A_R_CTRL4_NP22 (0x1u << 2)
#define HX8347A_R_CTRL4_NP30 (0x1u << 4)
#define HX8347A_R_CTRL4_NP31 (0x1u << 5)
#define HX8347A_R_CTRL4_NP32 (0x1u << 6)
/* -------- HX8347A_R_CTRL5 : (Offset: 0x4A) r Control 5 -------- */
#define HX8347A_R_CTRL5 (0x4Au)
#define HX8347A_R_CTRL5_NP40 (0x1u << 0)
#define HX8347A_R_CTRL5_NP41 (0x1u << 1)
#define HX8347A_R_CTRL5_NP42 (0x1u << 2)
#define HX8347A_R_CTRL5_NP50 (0x1u << 4)
#define HX8347A_R_CTRL5_NP51 (0x1u << 5)
#define HX8347A_R_CTRL5_NP52 (0x1u << 6)
/* -------- HX8347A_R_CTRL6 : (Offset: 0x4B) r Control 6 -------- */
#define HX8347A_R_CTRL6 (0x4Bu)
#define HX8347A_R_CTRL6_NN00 (0x1u << 0)
#define HX8347A_R_CTRL6_NN01 (0x1u << 1)
#define HX8347A_R_CTRL6_NN02 (0x1u << 2)
#define HX8347A_R_CTRL6_NN10 (0x1u << 4)
#define HX8347A_R_CTRL6_NN11 (0x1u << 5)
#define HX8347A_R_CTRL6_NN12 (0x1u << 6)
/* -------- HX8347A_R_CTRL7 : (Offset: 0x4C) r Control 7 -------- */
#define HX8347A_R_CTRL7 (0x4Cu)
#define HX8347A_R_CTRL7_NN20 (0x1u << 0)
#define HX8347A_R_CTRL7_NN21 (0x1u << 1)
#define HX8347A_R_CTRL7_NN22 (0x1u << 2)
#define HX8347A_R_CTRL7_NN30 (0x1u << 4)
#define HX8347A_R_CTRL7_NN31 (0x1u << 5)
#define HX8347A_R_CTRL7_NN32 (0x1u << 6)
/* -------- HX8347A_R_CTRL8 : (Offset: 0x4D) r Control 8 -------- */
#define HX8347A_R_CTRL8 (0x4Du)
#define HX8347A_R_CTRL8_NN40 (0x1u << 0)
#define HX8347A_R_CTRL8_NN41 (0x1u << 1)
#define HX8347A_R_CTRL8_NN42 (0x1u << 2)
#define HX8347A_R_CTRL8_NN50 (0x1u << 4)
#define HX8347A_R_CTRL8_NN51 (0x1u << 5)
#define HX8347A_R_CTRL8_NN52 (0x1u << 6)
/* -------- HX8347A_R_CTRL9 : (Offset: 0x4E) r Control 9 -------- */
#define HX8347A_R_CTRL9 (0x4Eu)
#define HX8347A_R_CTRL9_OP00 (0x1u << 0)
#define HX8347A_R_CTRL9_OP01 (0x1u << 1)
#define HX8347A_R_CTRL9_OP02 (0x1u << 2)
#define HX8347A_R_CTRL9_OP03 (0x1u << 3)
#define HX8347A_R_CTRL9_CGMP00 (0x1u << 4)
#define HX8347A_R_CTRL9_CGMP01 (0x1u << 5)
#define HX8347A_R_CTRL9_CGMP10 (0x1u << 6)
#define HX8347A_R_CTRL9_CGMP11 (0x1u << 7)
/* -------- HX8347A_R_CTRL10 : (Offset: 0x4F) r Control 10 -------- */
#define HX8347A_R_CTRL10 (0x4Fu)
#define HX8347A_R_CTRL10_OP10 (0x1u << 0)
#define HX8347A_R_CTRL10_OP11 (0x1u << 1)
#define HX8347A_R_CTRL10_OP12 (0x1u << 2)
#define HX8347A_R_CTRL10_OP13 (0x1u << 3)
#define HX8347A_R_CTRL10_OP14 (0x1u << 4)
#define HX8347A_R_CTRL10_CGMP2 (0x1u << 6)
#define HX8347A_R_CTRL10_CGMP3 (0x1u << 7)
/* -------- HX8347A_R_CTRL11 : (Offset: 0x50) r Control 11 -------- */
#define HX8347A_R_CTRL11 (0x50u)
#define HX8347A_R_CTRL11_ON00 (0x1u << 0)
#define HX8347A_R_CTRL11_ON01 (0x1u << 1)
#define HX8347A_R_CTRL11_ON02 (0x1u << 2)
#define HX8347A_R_CTRL11_ON03 (0x1u << 3)
#define HX8347A_R_CTRL11_CGMN00 (0x1u << 4)
#define HX8347A_R_CTRL11_CGMN01 (0x1u << 5)
#define HX8347A_R_CTRL11_CGMN10 (0x1u << 6)
#define HX8347A_R_CTRL11_CGMN11 (0x1u << 7)
/* -------- HX8347A_R_CTRL12 : (Offset: 0x51) r Control 12 -------- */
#define HX8347A_R_CTRL12 (0x51u)
#define HX8347A_R_CTRL12_ON10 (0x1u << 0)
#define HX8347A_R_CTRL12_ON11 (0x1u << 1)
#define HX8347A_R_CTRL12_ON12 (0x1u << 2)
#define HX8347A_R_CTRL12_ON13 (0x1u << 3)
#define HX8347A_R_CTRL12_ON14 (0x1u << 4)
#define HX8347A_R_CTRL12_CGMN2 (0x1u << 6)
#define HX8347A_R_CTRL12_CGMN3 (0x1u << 7)
/* -------- HX8347A_OTP_CTRL1 : (Offset: 0x52) OTP Control 1 -------- */
#define HX8347A_OTP_CTRL1 (0x52u)
#define HX8347A_OTP_CTRL1_OTP_MASK_POS 0
#define HX8347A_OTP_CTRL1_OTP_MASK_MSK (0xffu << HX8347A_OTP_CTRL1_OTP_MASK_POS)
#define HX8347A_OTP_CTRL1_OTP_MASK(value) ((HX8347A_OTP_CTRL1_OTP_MASK_MSK & ((value) << HX8347A_OTP_CTRL1_OTP_MASK_POS)))
/* -------- HX8347A_OTP_CTRL2 : (Offset: 0x53) OTP Control 2 -------- */
#define HX8347A_OTP_CTRL2 (0x53u)
#define HX8347A_OTP_CTRL2_OTP_INDEX_POS 0
#define HX8347A_OTP_CTRL2_OTP_INDEX_MSK (0xffu << HX8347A_OTP_CTRL2_OTP_INDEX_POS)
#define HX8347A_OTP_CTRL2_OTP_INDEX(value) ((HX8347A_OTP_CTRL2_OTP_INDEX_MSK & ((value) << HX8347A_OTP_CTRL2_OTP_INDEX_POS)))
/* -------- HX8347A_OTP_CTRL3 : (Offset: 0x54) OTP Control 3 -------- */
#define HX8347A_OTP_CTRL3 (0x54u)
#define HX8347A_OTP_CTRL3_OTP_PROG (0x1u << 0)
#define HX8347A_OTP_CTRL3_VPP_SEL (0x1u << 1)
#define HX8347A_OTP_CTRL3_OTP_PTM (0x1u << 3)
#define HX8347A_OTP_CTRL3_OTP_PWE (0x1u << 4)
#define HX8347A_OTP_CTRL3_OTP_POR (0x1u << 5)
#define HX8347A_OTP_CTRL3_DCCLK_DISABLE (0x1u << 6)
#define HX8347A_OTP_CTRL3_OTP_LOAD_DISABLE (0x1u << 7)
/* -------- HX8347A_HIMAX_ID_CODE : (Offset: 0x67) Himax ID code -------- */
#define HX8347A_HIMAX_ID_CODE (0x67u)
/* -------- HX8347A_DATA_CTRL : (Offset: 0x72) Data Control -------- */
#define HX8347A_DATA_CTRL (0x72u)
#define HX8347A_DATA_CTRL_TRI_POS 0
#define HX8347A_DATA_CTRL_TRI_MSK (0x3u << HX8347A_DATA_CTRL_TRI_POS)
#define HX8347A_DATA_CTRL_TRI(value) ((HX8347A_DATA_CTRL_TRI_MSK & ((value) << HX8347A_DATA_CTRL_TRI_POS)))
#define HX8347A_DATA_CTRL_DFM_POS 4
#define HX8347A_DATA_CTRL_DFM_MSK (0x3u << HX8347A_DATA_CTRL_DFM_POS)
#define HX8347A_DATA_CTRL_DFM(value) ((HX8347A_DATA_CTRL_DFM_MSK & ((value) << HX8347A_DATA_CTRL_DFM_POS)))
/* -------- HX8347A_DISP_CTRL8 : (Offset: 0x90) Display Control 8 -------- */
#define HX8347A_DISP_CTRL8 (0x90u)
#define HX8347A_DISP_CTRL8_SAP_POS 0
#define HX8347A_DISP_CTRL8_SAP_MSK (0xffu << HX8347A_DISP_CTRL8_SAP_POS)
#define HX8347A_DISP_CTRL8_SAP(value) ((HX8347A_DISP_CTRL8_SAP_MSK & ((value) << HX8347A_DISP_CTRL8_SAP_POS)))
/* -------- HX8347A_DISP_CTRL11 : (Offset: 0x91) Display Control 11 -------- */
#define HX8347A_DISP_CTRL11 (0x91u)
#define HX8347A_DISP_CTRL11_GEN_OFF_POS 0
#define HX8347A_DISP_CTRL11_GEN_OFF_MSK (0xffu << HX8347A_DISP_CTRL11_GEN_OFF_POS)
#define HX8347A_DISP_CTRL11_GEN_OFF(value) ((HX8347A_DISP_CTRL11_GEN_OFF_MSK & ((value) << HX8347A_DISP_CTRL11_GEN_OFF_POS)))
/* -------- HX8347A_OSC_CTRL3 : (Offset: 0x93) OSC Control 3 -------- */
#define HX8347A_OSC_CTRL3 (0x93u)
#define HX8347A_OSC_CTRL3_RADJ_POS 0
#define HX8347A_OSC_CTRL3_RADJ_MSK (0xfu << HX8347A_OSC_CTRL3_RADJ_POS)
#define HX8347A_OSC_CTRL3_RADJ(value) ((HX8347A_OSC_CTRL3_RADJ_MSK & ((value) << HX8347A_OSC_CTRL3_RADJ_POS)))
/* -------- HX8347A_SAP_IDLE_MODE : (Offset: 0x94) SAP Idle Mode -------- */
#define HX8347A_SAP_IDLE_MODE (0x94u)
#define HX8347A_SAP_IDLE_MODE_SAP_I_POS 0
#define HX8347A_SAP_IDLE_MODE_SAP_I_MSK (0xffu << HX8347A_SAP_IDLE_MODE_SAP_I_POS)
#define HX8347A_SAP_IDLE_MODE_SAP_I(value) (HX8347A_SAP_IDLE_MODE_SAP_I_MSK & ((value) << HX8347A_SAP_IDLE_MODE_SAP_I_POS)))
/* -------- HX8347A_DCCLK_SYNC_TO_CL1 : (Offset: 0x95) DCCLK SYNC TO CL1 -------- */
#define HX8347A_DCCLK_SYNC_TO_CL1 (0x95u)
#define HX8347A_DCCLK_SYNC_TO_CL1_DCCLK_SYNC (0x1u << 0)

/* Define EBI access for HX8347A 16-bit System Interface.*/
#if defined(BOARD_HX8347A_ADDR) && defined (BOARD_HX8347A_RS)
	static inline void LCD_IR(uint16_t lcd_index)
	{
		*((volatile uint16_t *)(BOARD_HX8347A_ADDR)) = lcd_index; /* HX8347A index register address */
	}
	static inline void LCD_WD(uint16_t lcd_data)
	{
		*((volatile uint16_t *)((BOARD_HX8347A_ADDR) | (BOARD_HX8347A_RS))) = lcd_data;
	}
	static inline uint16_t LCD_RD(void)
	{
		return *((volatile uint16_t *)((BOARD_HX8347A_ADDR) | (BOARD_HX8347A_RS)));
	}
#else
	#error "Missing module configuration for HX8347A!"
#endif

/* RGB 24-bits color table definition (RGB888). */
#define COLOR_BLACK          (0x000000u)
#define COLOR_WHITE          (0xFFFFFFu)
#define COLOR_BLUE           (0x0000FFu)
#define COLOR_GREEN          (0x00FF00u)
#define COLOR_RED            (0xFF0000u)
#define COLOR_NAVY           (0x000080u)
#define COLOR_DARKBLUE       (0x00008Bu)
#define COLOR_DARKGREEN      (0x006400u)
#define COLOR_DARKCYAN       (0x008B8Bu)
#define COLOR_CYAN           (0x00FFFFu)
#define COLOR_TURQUOISE      (0x40E0D0u)
#define COLOR_INDIGO         (0x4B0082u)
#define COLOR_DARKRED        (0x800000u)
#define COLOR_OLIVE          (0x808000u)
#define COLOR_GRAY           (0x808080u)
#define COLOR_SKYBLUE        (0x87CEEBu)
#define COLOR_BLUEVIOLET     (0x8A2BE2u)
#define COLOR_LIGHTGREEN     (0x90EE90u)
#define COLOR_DARKVIOLET     (0x9400D3u)
#define COLOR_YELLOWGREEN    (0x9ACD32u)
#define COLOR_BROWN          (0xA52A2Au)
#define COLOR_DARKGRAY       (0xA9A9A9u)
#define COLOR_SIENNA         (0xA0522Du)
#define COLOR_LIGHTBLUE      (0xADD8E6u)
#define COLOR_GREENYELLOW    (0xADFF2Fu)
#define COLOR_SILVER         (0xC0C0C0u)
#define COLOR_LIGHTGREY      (0xD3D3D3u)
#define COLOR_LIGHTCYAN      (0xE0FFFFu)
#define COLOR_VIOLET         (0xEE82EEu)
#define COLOR_AZUR           (0xF0FFFFu)
#define COLOR_BEIGE          (0xF5F5DCu)
#define COLOR_MAGENTA        (0xFF00FFu)
#define COLOR_TOMATO         (0xFF6347u)
#define COLOR_GOLD           (0xFFD700u)
#define COLOR_ORANGE         (0xFFA500u)
#define COLOR_SNOW           (0xFFFAFAu)
#define COLOR_YELLOW         (0xFFFF00u)

/**
 * Data type for HX8347A color (RGB565)
 */
typedef uint16_t hx8347a_color_t;

/**
 * Input parameters when initializing HX8347A driver.
 */
struct hx8347a_opt_t {
	uint32_t ul_width;                  //!< lcd width in pixel
	uint32_t ul_height;                 //!< lcd height in pixel
	hx8347a_color_t foreground_color;   //!< lcd foreground color
	hx8347a_color_t background_color;   //!< lcd background color
};

/**
 * Font structure
 */
struct font {
	/* Font width in pixels. */
	uint8_t width;
	/* Font height in pixels. */
	uint8_t height;
};

/**
 * Display direction option
 */
typedef enum display_direction {
	LANDSCAPE  = 0,
	PORTRAIT   = 1
} display_direction_t;

uint32_t hx8347a_init(struct hx8347a_opt_t *p_opt);
void hx8347a_display_on(void);
void hx8347a_display_off(void);
void hx8347a_set_foreground_color(hx8347a_color_t us_color);
void hx8347a_fill(hx8347a_color_t us_color);
void hx8347a_set_window(uint32_t ul_x, uint32_t ul_y, uint32_t ul_width,
		uint32_t ul_height);
void hx8347a_set_cursor_position(uint16_t us_x, uint16_t us_y);
void hx8347a_scroll(int32_t ul_lines);
void hx8347a_enable_scroll(void);
void hx8347a_disable_scroll(void);
void hx8347a_set_display_direction(display_direction_t dd);
uint32_t hx8347a_draw_pixel(uint32_t ul_x, uint32_t ul_y);
hx8347a_color_t hx8347a_get_pixel(uint32_t ul_x, uint32_t ul_y);
void hx8347a_draw_line(uint32_t ul_x1, uint32_t ul_y1,
		uint32_t ul_x2, uint32_t ul_y2);
void hx8347a_draw_rectangle(uint32_t ul_x1, uint32_t ul_y1,
		uint32_t ul_x2, uint32_t ul_y2);
void hx8347a_draw_filled_rectangle(uint32_t ul_x1, uint32_t ul_y1,
		uint32_t ul_x2, uint32_t ul_y2);
uint32_t hx8347a_draw_circle(uint32_t ul_x, uint32_t ul_y, uint32_t ul_r);
uint32_t hx8347a_draw_filled_circle(uint32_t ul_x, uint32_t ul_y, uint32_t ul_r);
void hx8347a_draw_string(uint32_t ul_x, uint32_t ul_y, const uint8_t *p_str);
void hx8347a_draw_pixmap(uint32_t ul_x, uint32_t ul_y, uint32_t ul_width,
		uint32_t ul_height, const hx8347a_color_t *p_ul_pixmap);

/// @cond 0
/**INDENT-OFF**/
#ifdef __cplusplus
}
#endif
/**INDENT-ON**/
/// @endcond

#endif /* HX8347A_H_INCLUDED */
