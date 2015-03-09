/**
 * \file
 *
 * \brief SAM D21 TCC - Timer Counter for Control Driver
 *
 * Copyright (C) 2013-2014 Atmel Corporation. All rights reserved.
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

#ifndef TCC_H_INCLUDED
#define TCC_H_INCLUDED

/**
 * \defgroup asfdoc_sam0_tcc_group SAM D21 Timer/Counter for Control Driver (TCC)
 *
 * This driver for SAM D21 devices provides an interface for the configuration
 * and management of the timer modules within the device, for waveform
 * generation and timing operations. It also provide extended options for
 * control applications.
 *
 * The following driver API modes are covered
 * by this manual:
 *
 *  - Polled APIs
 * \if TCC_CALLBACK_MODE
 *  - Callback APIs
 * \endif
 *
 * The following peripherals are used by this module:
 *
 *  - TCC (Timer/Counter for Control Applications)
 *
 * The outline of this documentation is as follows:
 *  - \ref asfdoc_sam0_tcc_prerequisites
 *  - \ref asfdoc_sam0_tcc_module_overview
 *  - \ref asfdoc_sam0_tcc_special_considerations
 *  - \ref asfdoc_sam0_tcc_extra_info
 *  - \ref asfdoc_sam0_tcc_examples
 *  - \ref asfdoc_sam0_tcc_api_overview
 *
 * \section asfdoc_sam0_tcc_prerequisites Prerequisites
 *
 * There are no prerequisites for this module.
 *
 * \section asfdoc_sam0_tcc_module_overview Module Overview
 *
 * The Timer/Counter for Control Applications (TCC) module provides a set of
 * timing and counting related functionality, such as the generation of periodic
 * waveforms, the capturing of a periodic waveform's frequency/duty cycle,
 * software timekeeping for periodic operations, waveform extension control and
 * fault detection, etc.
 * The counter size of TCC modules can be 16- or 24-bit max.
 *
 * This TCC module for the SAM D21 is capable of the following functions:
 *
 * - Generation of PWM signals
 * - Generation of timestamps for events
 * - General time counting
 * - Waveform period capture
 * - Waveform frequency capture
 * - Additional control for generated waveform outputs
 * - Fault protection for waveform generation
 *
 * \ref asfdoc_sam0_tcc_block_diagram "The diagram below" shows the overview
 * of the TCC module design.
 *
 * \anchor asfdoc_sam0_tcc_block_diagram
 * \image html overview.svg "Overview of the TCC module"
 *
 * \subsection asfdoc_sam0_tcc_module_overview_parts Functional Description
 * Independent of the event, DMA and interrupt system, the TCC module consists
 * of following three parts:
 * - Base Counter
 * - Compare/Capture channels, with waveform generation
 * - Waveform extended control and fault detection
 *
 * The base counter counts input signals, such as connected generic clock (GCLK)
 * channel, or event signals (TCEx, with event action configured to counting).
 * Then the counter value is used by compare/capture channel which can be set up
 * in one of two different modes: capture and compare.
 *
 * In capture mode, the counter value is stored when a configurable event
 * occurs. This mode can be used to generate timestamps used in event capture,
 * or it can be used for the measurement of a periodic input signal's
 * frequency/duty cycle.
 *
 * In compare mode, the counter value is compared against one or more of the
 * configured channel compare values. When the counter value coincides with a
 * compare value an action can be taken automatically by the module, such as
 * generating an output event or toggling a pin when used for frequency or PWM
 * signal generation.
 *
 * \note The connection of events between modules requires the use of the
 *       \ref asfdoc_sam0_events_group "SAM D2x Event System Driver (EVENTS)"
 *       to route output event of one module to the the input event of another.
 *       For more information on event routing, refer to the event driver
 *       documentation.
 *
 * In compare mode, when output signal is generated. Extended waveform controls
 * are available, to reform the compare outputs to specific format. E.g., output
 * matrix can change the path of channel outputs to output pins, pattern will
 * overwrite the output signal line to special state, fault protection can
 * automatically restart/freeze counter or put the output lines to pre-defined
 * state, to disable and/or shut down the external drivers.
 *
 * \subsection asfdoc_sam0_tcc_module_overview_tc Base Timer/Counter
 *
 * \subsubsection asfdoc_sam0_tcc_module_overview_tc_size Timer/Counter Size
 * Each timer module has been defined in one of following different counter
 * sizes: 16-, and 24-bits. The size of the counter determines the maximum value
 * it can count to before an overflow occurs and the count is reset back to
 * zero. \ref asfdoc_sam0_tcc_count_size_vs_top "The table below" shows the
 * maximum values for each of the possible counter sizes.
 *
 * \anchor asfdoc_sam0_tcc_count_size_vs_top
 * <table>
 *  <caption>Timer counter sizes and their maximum count values</caption>
 *  <tr>
 *    <th>Counter Size</th>
 *    <th>Max (Hexadecimal)</th>
 *    <th>Max (Decimal)</th>
 *  </tr>
 *  <tr>
 *    <td>16-bit</td>
 *    <td>0xFFFF</td>
 *    <td>65,535</td>
 *  </tr>
 *  <tr>
 *    <td>24-bit</td>
 *    <td>0xFFFFFF</td>
 *    <td>16,777,215</td>
 *  </tr>
 * </table>
 *
 * Besides the counter size, the period/top value of the counter can be set, to
 * define counting period. This way, the overflow occurs when counter achieve
 * the assigned period/top value.
 *
 * \subsubsection asfdoc_sam0_tcc_module_overview_tc_clk Timer/Counter Clock and Prescaler
 * Each TCC peripheral is clocked asynchronously to the system clock by a GCLK
 * (Generic Clock) channel. The GCLK channel connects to any of the GCLK
 * generators. The GCLK generators are configured to use one of the available
 * clock sources on the system such as internal oscillator, external crystals
 * etc. - see the \ref asfdoc_sam0_system_clock_group "Generic Clock driver" for
 * more information.
 *
 * Each TCC module in the SAM D21 has its own individual clock prescaler, which
 * can be used to divide the input clock frequency used in the counter. This
 * prescaler only scales the clock used to provide clock pulses for the counter
 * to count, and does not affect the digital register interface portion of
 * the module, thus the timer registers will synchronized to the raw GCLK
 * frequency input to the module.
 *
 * As a result of this, when selecting a GCLK frequency and timer prescaler
 * value the user application should consider both the timer resolution
 * required and the synchronization frequency, to avoid lengthy
 * synchronization times of the module if a very slow GCLK frequency is fed
 * into the TCC module. It is preferable to use a higher module GCLK frequency
 * as the input to the timer and prescale this down as much as possible to
 * obtain a suitable counter frequency in latency-sensitive applications.
 *
 * \subsubsection asfdoc_sam0_tcc_module_overview_tc_ctrl Timer/Counter Control Inputs (Events)
 *
 * Except the GCLK, timer/counter accepts events input. The input events can be
 * used to control the counter action or drive the counter counting instead of
 * GCLK, or generate fault condition, depending on the event actions, described
 * in \ref asfdoc_sam0_tcc_module_event_act "events action settings".
 *
 * \anchor asfdoc_sam0_tcc_module_event_act
 * <table>
 *   <caption>TCC module events actions</caption>
 *   <tr>
 *     <th>Event Action</th>
 *     <th>Description</th>
 *     <th>Applied Event#</th>
 *   </tr>
 *   <tr>
 *     <td>\ref TCC_EVENT_ACTION_OFF</td>
 *     <td>No action on the event input</td>
 *     <td>All</td>
 *   </tr>
 *   <tr>
 *     <td>\ref TCC_EVENT_ACTION_RETRIGGER</td>
 *     <td>Re-trigger Counter on event</td>
 *     <td>All</td>
 *   </tr>
 *   <tr>
 *     <td>\ref TCC_EVENT_ACTION_NON_RECOVERABLE_FAULT</td>
 *     <td>Generate Non-Recoverable Fault on event</td>
 *     <td>All</td>
 *   </tr>
 *   <tr>
 *     <td>\ref TCC_EVENT_ACTION_START</td>
 *     <td>Counter start on event</td>
 *     <td>TCE0</td>
 *   </tr>
 *   <tr>
 *     <td>\ref TCC_EVENT_ACTION_DIR_CONTROL</td>
 *     <td>Counter direction control (event line low: up; high: down)</td>
 *     <td>TCE0</td>
 *   </tr>
 *   <tr>
 *     <td>\ref TCC_EVENT_ACTION_DECREMENT</td>
 *     <td>Counter decrement on event</td>
 *     <td>TCE0</td>
 *   </tr>
 *   <tr>
 *     <td>\ref TCC_EVENT_ACTION_PERIOD_PULSE_WIDTH_CAPTURE</td>
 *     <td>Capture pulse period and pulse width</td>
 *     <td>TCE0</td>
 *   </tr>
 *   <tr>
 *     <td>\ref TCC_EVENT_ACTION_PULSE_WIDTH_PERIOD_CAPTURE</td>
 *     <td>Capture pulse width and pulse period</td>
 *     <td>TCE0</td>
 *   </tr>
 *   <tr>
 *     <td>\ref TCC_EVENT_ACTION_STOP</td>
 *     <td>Counter stop on event</td>
 *     <td>TCE1</td>
 *   </tr>
 *   <tr>
 *     <td>\ref TCC_EVENT_ACTION_COUNT_EVENT</td>
 *     <td>Counter count on event</td>
 *     <td>TCE1</td>
 *   </tr>
 *   <tr>
 *     <td>\ref TCC_EVENT_ACTION_INCREMENT</td>
 *     <td>Counter increment on event</td>
 *     <td>TCE1</td>
 *   </tr>
 *   <tr>
 *     <td>\ref TCC_EVENT_ACTION_COUNT_DURING_ACTIVE</td>
 *     <td>Counter count during active state of asynchronous event</td>
 *     <td>TCE1</td>
 *   </tr>
 * </table>
 *
 * \subsubsection asfdoc_sam0_tcc_module_overview_tc_reload Timer/Counter Reloading
 *
 * Timer modules also contain a configurable reload action, used when a
 * re-trigger event occurs. Examples of a re-trigger event are the counter
 * reaching the max value when counting up, or when an event from the event
 * system tells the counter to re-trigger. The reload action determines if the
 * prescaler should be reset, and when this should happen. The counter will
 * always be reloaded with the value it is set to start counting from. The user
 * can choose between three different reload actions, described in
 * \ref asfdoc_sam0_tcc_module_reload_act "the table below".
 *
 * \anchor asfdoc_sam0_tcc_module_reload_act
 * <table>
 *   <caption>TCC module reload actions</caption>
 *   <tr>
 *     <th>Reload Action</th>
 *     <th>Description</th>
 *   </tr>
 *   <tr>
 *     <td>\ref TCC_RELOAD_ACTION_GCLK </td>
 *     <td>Reload TCC counter value on next GCLK cycle. Leave prescaler
 *         as-is.</td>
 *   </tr>
 *   <tr>
 *     <td>\ref TCC_RELOAD_ACTION_PRESC </td>
 *     <td>Reloads TCC counter value on next prescaler clock. Leave prescaler
 *         as-is.</td>
 *   </tr>
 *  <tr>
 *    <td> \ref TCC_RELOAD_ACTION_RESYNC </td>
 *    <td>Reload TCC counter value on next GCLK cycle. Clear prescaler to
 *        zero.</td>
 *  </tr>
 * </table>
 *
 * The reload action to use will depend on the specific application being
 * implemented. One example is when an external trigger for a reload occurs; if
 * the TCC uses the prescaler, the counter in the prescaler should not have a
 * value between zero and the division factor. The TC counter and the counter
 * in the prescaler should both start at zero. When the counter is set to
 * re-trigger when it reaches the max value on the other hand, this is not the
 * right option to use. In such a case it would be better if the prescaler is
 * left unaltered when the re-trigger happens, letting the counter reset on the
 * next GCLK cycle.
 *
 * \subsubsection asfdoc_sam0_tcc_module_overview_tc_oneshot One-shot Mode
 *
 * TCC modules can be configured into a one-shot mode. When configured in this
 * manner, starting the timer will cause it to count until the next overflow
 * or underflow condition before automatically halting, waiting to be manually
 * triggered by the user application software or an event signal from the event
 * system.
 *
 * \subsection asfdoc_sam0_tcc_module_overview_capt Capture Operations
 *
 * In capture operations, any event from the event system or a pin change can
 * trigger a capture of the counter value. This captured counter value can be
 * used as timestamps for the events, or it can be used in frequency and pulse
 * width capture.
 *
 * \subsubsection asfdoc_sam0_tcc_module_overview_capt_ev Capture Operations - Event
 *
 * Event capture is a simple use of the capture functionality,
 * designed to create timestamps for specific events. When the TCC
 * module's input capture pin is externally toggled, the current timer
 * count value is copied into a buffered register which can then be
 * read out by the user application.
 *
 * Note that when performing any capture operation, there is a risk that the
 * counter reaches its top value (MAX) when counting up, or the bottom value
 * (zero) when counting down, before the capture event occurs. This can distort
 * the result, making event timestamps to appear shorter than reality; the
 * user application should check for timer overflow when reading a capture
 * result in order to detect this situation and perform an appropriate
 * adjustment.
 *
 * Before checking for a new capture, \ref TCC_STATUS_COUNT_OVERFLOW
 * should be checked. The response to an overflow error is left to the user
 * application, however it may be necessary to clear both the capture overflow
 * flag and the capture flag upon each capture reading.
 *
 * \subsubsection asfdoc_sam0_tcc_module_overview_capt_pulse Capture Operations - Pulse Width
 *
 * Pulse Width Capture mode makes it possible to measure the pulse width and
 * period of PWM signals. This mode uses two capture channels of the counter.
 * This means that the counter module used for Pulse Width Capture can not be
 * used for any other purpose. There are two modes for pulse width capture;
 * Pulse Width Period (PWP) and Period Pulse Width (PPW). In PWP mode, capture
 * channel 0 is used for storing the pulse width and capture channel 1 stores
 * the observed period. While in PPW mode, the roles of the two capture channels
 * is reversed.
 *
 * As in the above example it is necessary to poll on interrupt flags to see
 * if a new capture has happened and check that a capture overflow error has
 * not occurred.
 *
 * Refer to \ref asfdoc_sam0_tcc_module_overview_tc_ctrl to set up the input
 * event to perform pulse width capture.
 *
 * \subsection asfdoc_sam0_tcc_module_overview_mc Match Compare Operations
 *
 * In compare match operation, Compare/Capture registers are used in comparison
 * with the counter value. When the timer's count value matches the value of a
 * compare channel, a user defined action can be taken.
 *
 * \subsubsection asfdoc_sam0_tcc_module_overview_mc_timer Basic Timer
 *
 * A Basic Timer is a simple application where compare match operations is used
 * to determine when a specific period has elapsed. In Basic Timer operations,
 * one or more values in the module's Compare/Capture registers are used to
 * specify the time (as a number of prescaled GCLK cycles, or input events) when
 * an action should be taken by the microcontroller. This can be an Interrupt
 * Service Routine (ISR), event generator via the event system, or a software
 * flag that is polled via the user application.
 *
 * \subsubsection asfdoc_sam0_tcc_module_overview_mc_wave Waveform Generation
 *
 * Waveform generation enables the TCC module to generate square waves, or if
 * combined with an external passive low-pass filter, analog waveforms.
 *
 * \subsubsection asfdoc_sam0_tcc_module_overview_mc_wave_pwm Waveform Generation - PWM
 *
 * Pulse width modulation is a form of waveform generation and a signalling
 * technique that can be useful in many situations. When PWM mode is used,
 * a digital pulse train with a configurable frequency and duty cycle can be
 * generated by the TCC module and output to a GPIO pin of the device.
 *
 * Often PWM is used to communicate a control or information parameter to an
 * external circuit or component. Differing impedances of the source generator
 * and sink receiver circuits is less of an issue when using PWM compared to
 * using an analog voltage value, as noise will not generally affect the
 * signal's integrity to a meaningful extent.
 *
 * \ref asfdoc_sam0_tcc_module_pwm_single_diag "The figure below" illustrates
 * operations and different states of the counter and its output when running
 * the counter in PWM Single-Slope mode. As can be seen, the TOP/PERIOD value is
 * unchanged and is set to MAX. The compare match value is changed at several
 * points to illustrate the resulting waveform output changes. The PWM output is
 * set to normal (i.e. non-inverted) output mode.
 *
 * \anchor asfdoc_sam0_tcc_module_pwm_single_diag
 * \image html pwm_single_ex.svg "Example of PWM in Single-Slope mode, and different counter operations"
 *
 * There are some other PWM modes supported by TCC module, refer to data sheet
 * for the PWM waveform generation.
 *
 * \subsubsection asfdoc_sam0_tcc_module_overview_mc_wave_freq Waveform Generation - Frequency
 *
 * Frequency Generation normal mode is in many ways identical to PWM generation.
 * However, in Frequency Generation a toggle only occurs on the output when a
 * match on a compare channels occurs.
 *
 * When the match mode is used, the timer value is reset on match condition,
 * resulting in a variable frequency square wave with a fixed 50% duty cycle.
 *
 * \subsection asfdoc_sam0_tcc_module_overview_ext Waveform Extended Controls
 *
 * \subsubsection asfdoc_sam0_tcc_module_overview_ext_pat Pattern Generation
 *
 * Pattern insertion allows TCC module to change the actual pin output level
 * without modifying the compare/match settings. As follow:
 *
 * \anchor asfdoc_sam0_tcc_module_pattern_gen
 * <table>
 *   <caption>TCC module Output Pattern Generation</caption>
 *   <tr>
 *     <th>Pattern</th>
 *     <th>Description</th>
 *   </tr>
 *   <tr>
 *     <td>\ref TCC_OUTPUT_PATTERN_DISABLE </td>
 *     <td>Pattern disabled, generate output as is</td>
 *   </tr>
 *   <tr>
 *     <td>\ref TCC_OUTPUT_PATTERN_0 </td>
 *     <td>Generate pattern 0 on output (keep output to low level)</td>
 *   </tr>
 *  <tr>
 *    <td> \ref TCC_OUTPUT_PATTERN_1 </td>
 *    <td>Generate pattern 1 on output (keep output to high level)</td>
 *  </tr>
 * </table>
 *
 * \subsubsection asfdoc_sam0_tcc_module_overview_ext_r_fault Recoverable Faults
 *
 * The recoverable faults can restart or halt the TCC timer/counter. So that the
 * final output wave is kept in a defined state. When the fault state is removed
 * it's possible to recover the counter and waveform generation.
 *
 * In TCC module, only first of two compare channels (CC0 and CC1) can work with
 * recoverable fault inputs. The corresponding event inputs (MCE0 and MCE1) are
 * then used as fault inputs respectively. The faults are called Fault A and
 * Fault B.
 *
 * The recoverable fault can be filtered. On fault condition there are many
 * actions can be chosen. Also it's possible to get the fault time stamp. Please
 * refer to data sheet for more details about the recoverable fault operations.
 *
 * \subsubsection asfdoc_sam0_tcc_module_overview_ext_n_fault Non-Recoverable Faults
 *
 * The non-recoverable faults force all the TCC output pins to a pre-defined
 * level (can be force 0 or 1). The input control signal of non-recoverable
 * fault is from timer/counter event (TCEx). To enable non-recoverable fault,
 * corresponding TCEx event action must be set to non-recoverable fault action (
 * \ref TCC_EVENT_ACTION_NON_RECOVERABLE_FAULT).
 * Refer to \ref asfdoc_sam0_tcc_module_overview_tc_ctrl to the set up of
 * timer/counter event input action.
 *
 * \subsection asfdoc_sam0_tcc_module_overview_tc_sleep Sleep Mode
 *
 * TCC modules can be configured to operate in any sleep mode, with its "run
 * in standby" function enabled. It can wakeup the device using interrupts or
 * perform internal actions through the event system.
 *
 * \section asfdoc_sam0_tcc_special_considerations Special Considerations
 *
 * \subsection asfdoc_sam0_tc_special_considerations_tcc_feature Module Features
 *
 * The feature of TCC, such as timer/counter size, number of compare capture
 * channels, number of outputs, are dependent on the specific SAM D21 device
 * being used, and the module used.
 *
 * \subsubsection asfdoc_sam0_tc_special_considerations_tcc_d21 SAM D21 TCC Feature List
 * For SAM D21, the TCC features are as follow:
 * \anchor asfdoc_sam0_tcc_features_d21
 * <table>
 *   <caption>TCC module features for SAM D21</caption>
 *   <tr>
 *     <th>TCC#</th>
 *     <th>Match/Capture Channels</th>
 *     <th>Wave outputs</th>
 *     <th>Counter Size (bits)</th>
 *     <th>Fault</th>
 *     <th>Dithering</th>
 *     <th>Output Matrix</th>
 *     <th>Dead-Time Insertion</th>
 *     <th>SWAP</th>
 *     <th>Pattern</th>
 *   </tr>
 *   <tr>
 *     <td>0</td>
 *     <td>4</td>
 *     <td>8</td>
 *     <td>24</td>
 *     <td>Y</td>
 *     <td>Y</td>
 *     <td>Y</td>
 *     <td>Y</td>
 *     <td>Y</td>
 *     <td>Y</td>
 *   </tr>
 *   <tr>
 *     <td>1</td>
 *     <td>2</td>
 *     <td>4</td>
 *     <td>24</td>
 *     <td>Y</td>
 *     <td>Y</td>
 *     <td></td>
 *     <td></td>
 *     <td></td>
 *     <td>Y</td>
 *   </tr>
 *   <tr>
 *     <td>2</td>
 *     <td>2</td>
 *     <td>2</td>
 *     <td>16</td>
 *     <td>Y</td>
 *     <td></td>
 *     <td></td>
 *     <td></td>
 *     <td></td>
 *     <td></td>
 *   </tr>
 * </table>
 *
 * \subsection asfdoc_sam0_tc_special_considerations_tcc_pin Channels VS. Pin outs
 *
 * As the TCC module may have more waveform output pins than the number of
 * compare/capture channels, the free pins (with number higher than number of
 * channels) will reuse the waveform generated by channels subsequently. E.g.,
 * if the number of channels is 4 and number of wave output pins is 8, channel
 * 0 output is able to provide signal for wave out pin 0 and 4, channel 1 output
 * is able to feed wave out pin 1 and 5, and so on.
 *
 * \section asfdoc_sam0_tcc_extra_info Extra Information
 *
 * For extra information see \ref asfdoc_sam0_tcc_extra. This includes:
 *  - \ref asfdoc_sam0_tcc_extra_acronyms
 *  - \ref asfdoc_sam0_tcc_extra_dependencies
 *  - \ref asfdoc_sam0_tcc_extra_errata
 *  - \ref asfdoc_sam0_tcc_extra_history
 *
 *
 * \section asfdoc_sam0_tcc_examples Examples
 *
 * For a list of examples related to this driver, see
 * \ref asfdoc_sam0_tcc_exqsg.
 *
 * \section asfdoc_sam0_tcc_api_overview API Overview
 * @{
 */

#include <compiler.h>
#include <clock.h>
#include <gclk.h>
#include <pinmux.h>

/** Max number of channels supported by the driver
 *  (Channel index from 0 to \c TCC_NUM_CHANNELS - 1).
 */
#define TCC_NUM_CHANNELS           4

/** Max number of wave outputs lines supported by the driver
 *  (Output line index from 0 to \c TCC_NUM_WAVE_OUTPUTS - 1).
 */
#define TCC_NUM_WAVE_OUTPUTS       8

/** Max number of faults supported by the driver. */
#define TCC_NUM_FAULTS             2

#if TCC_ASYNC == true
#  include <system_interrupt.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Generates a table enum list entry for a given type
   and index (e.g. "TCC_CALLBACK_MC_CHANNEL_0,"). */
#define _TCC_ENUM(n, type) TCC_##type##_##n,

/* Generates table enum list entries for all channels of a
   given type and channel number on TCC module. */
#define _TCC_CHANNEL_ENUM_LIST(type) \
		MREPEAT(TCC_NUM_CHANNELS, _TCC_ENUM, type##_CHANNEL)
/* Generates table enum list entries for all output of a
   given type and waveform output number on TCC module. */
#define _TCC_WO_ENUM_LIST(type) \
		MREPEAT(TCC_NUM_WAVE_OUTPUTS, _TCC_ENUM, type)


#if TCC_ASYNC == true
/** Enum for the possible callback types for the TCC module. */
enum tcc_callback {
	/** Callback for TCC overflow. */
	TCC_CALLBACK_OVERFLOW,
	/** Callback for TCC Retrigger. */
	TCC_CALLBACK_RETRIGGER,
	/** Callback for TCC counter event. */
	TCC_CALLBACK_COUNTER_EVENT,
	/** Callback for capture overflow error. */
	TCC_CALLBACK_ERROR,
	/** Callback for Recoverable Fault A. */
	TCC_CALLBACK_FAULTA,
	/** Callback for Recoverable Fault B. */
	TCC_CALLBACK_FAULTB,
	/** Callback for Non-Recoverable Fault. 0 */
	TCC_CALLBACK_FAULT0,
	/** Callback for Non-Recoverable Fault. 1 */
	TCC_CALLBACK_FAULT1,

#  if defined(__DOXYGEN__)
	/** Channel callback type table for TCC
	 *
	 *  Each TCC module may contain several callback types for channels; each
	 *  channel will have its own callback type in the table, with the channel
	 *  index number substituted for "n" in the channel callback type
	 *  (e.g. \c TCC_MATCH_CAPTURE_CHANNEL_0).
	 */
	TCC_CALLBACK_CHANNEL_n = n,
#  else
	/** Callbacks for Match/Capture channels, e.g., TCC_CALLBACK_CHANNEL_0. */
	_TCC_CHANNEL_ENUM_LIST(CALLBACK)
#  endif

#  if !defined(__DOXYGEN__)
	/** Number of available callbacks. */
	TCC_CALLBACK_N
#  endif
};
#endif /* #if TCC_ASYNC == true */

/**
 * \name Module status flags
 *
 * TCC status flags, returned by \ref tcc_get_status() and cleared by
 * \ref tcc_clear_status().
 *
 * @{
 */

/** Timer channel \c ch (0 ~ 7) has matched against its compare value,
 * or has captured a new value.
 */
#define TCC_STATUS_CHANNEL_MATCH_CAPTURE(ch)        (1UL << (ch))
/** Timer channel \c ch (0 ~ 7) match/compare output value. */
#define TCC_STATUS_CHANNEL_OUTPUT(ch)               (1UL << ((ch)+8))
/** A Non-Recoverable Fault \c x (0 ~ 1) has occured */
#define TCC_STATUS_NON_RECOVERABLE_FAULT_OCCUR(x)   (1UL << ((x)+16))
/** A Recoverable Fault \c n (0 ~ 1 representing A ~ B) has occured */
#define TCC_STATUS_RECOVERABLE_FAULT_OCCUR(n)       (1UL << ((n)+18))
/** The Non-Recoverable Fault \c x (0 ~ 1) input is present */
#define TCC_STATUS_NON_RECOVERABLE_FAULT_PRESENT(x) (1UL << ((x)+20))
/** A Recoverable Fault \c n (0 ~ 1 representing A ~ B) is present */
#define TCC_STATUS_RECOVERABLE_FAULT_PRESENT(n)     (1UL << ((n)+22))
/** Timer registers synchronization has completed, and the synchronized count
 *  value may be read.
 */
#define TCC_STATUS_SYNC_READY                       (1UL << 23)
/** A new value was captured before the previous value was read, resulting in
 *  lost data.
 */
#define TCC_STATUS_CAPTURE_OVERFLOW                 (1UL << 24)
/** A counter event occurs */
#define TCC_STATUS_COUNTER_EVENT                    (1UL << 25)
/** A counter retrigger occurs */
#define TCC_STATUS_COUNTER_RETRIGGERED              (1UL << 26)
/** The timer count value has overflowed from its maximum value to its minimum
 *  when counting upward, or from its minimum value to its maximum when
 *  counting downward.
 */
#define TCC_STATUS_COUNT_OVERFLOW                   (1UL << 27)
/** Ramp period cycle index.
 *  In ramp operation, each two period cycles are marked as cycle A and B,
 *  the index 0 represents cycle A and 1 represents cycle B. */
#define TCC_STATUS_RAMP_CYCLE_INDEX                 (1UL << 28)
/** The counter has been stopped (due to disable, stop command or one-shot) */
#define TCC_STATUS_STOPPED                          (1UL << 29)

/** @} */

/**
 * \brief Index of the match capture channels
 *
 * This enum is used to specify which capture/match channel to do
 * operations on.
 */
enum tcc_match_capture_channel {
#  if defined(__DOXYGEN__)
	/** Match capture channel index table for TCC
	 *
	 *  Each TCC module may contain several match capture channels; each channel
	 *  will have its own index in the table, with the index number substituted
	 *  for "n" in the index name (e.g. \c TCC_MATCH_CAPTURE_CHANNEL_0).
	 */
	TCC_MATCH_CAPTURE_CHANNEL_n = n,
#  else
	/** Indexes of match capture channels, e.g., TCC_MATCH_CAPTURE_CHANNEL_0. */
	_TCC_CHANNEL_ENUM_LIST(MATCH_CAPTURE)
#  endif
#  if !defined(__DOXYGEN__)
	/** Number of supported channels */
	TCC_MATCH_CAPTURE_CHANNEL_N
#  endif
};

/**
 * \brief Index of the wave outputs
 *
 * This enum is used to specify which wave output to do
 * operations on.
 */
enum tcc_wave_output {
#  if defined(__DOXYGEN__)
	/** Waveform output index table for TCC
	 *
	 *  Each TCC module may contain several wave outputs; each output
	 *  will have its own index in the table, with the index number substituted
	 *  for "n" in the index name (e.g. \c TCC_WAVE_OUTPUT_0).
	 */
	TCC_WAVE_OUTPUT_n = n,
#  else
	/** Indexes of match capture channels, e.g., TCC_WAVEFORM_OUTPUT_0. */
	_TCC_WO_ENUM_LIST(WAVE_OUTPUT)
#  endif
#  if !defined(__DOXYGEN__)
	/** Number of supported channels */
	TCC_WAVE_OUTPUT_N
#  endif
};

/**
 * \brief TCC wave generation mode enum
 *
 * This enum is used to select which mode to run the wave
 * generation in.
 *
 */
enum tcc_wave_generation {
	/** Normal Frequency: Top is the PER register, output toggled on each
	 *  compare match. */
	TCC_WAVE_GENERATION_NORMAL_FREQ = 0,
	/** Match Frequency: Top is CC0 register, output toggles on each update
	 *  condition. */
	TCC_WAVE_GENERATION_MATCH_FREQ = 1,
	/** Single-Slope PWM: Top is the PER register, CCx controls duty cycle (
	 *  output active when count is greater than CCx). */
	TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM = 2,
	/** Double-slope (count up and down), critical: Top is the PER register,
	 *  CC[x] for counting up and CC[x+N/2] for counting down. */
	TCC_WAVE_GENERATION_DOUBLE_SLOPE_CRITICAL = 4,
	/** Double-slope (count up and down), interrupt/event at Bottom (Top is the
	 *  PER register, output active when count is greater than CCx). */
	TCC_WAVE_GENERATION_DOUBLE_SLOPE_BOTTOM = 5,
	/** Double-slope (count up and down), interrupt/event at Bottom and Top: (Top is the
	 *  PER register, output active when count is lower than CCx). */
	TCC_WAVE_GENERATION_DOUBLE_SLOPE_BOTH = 6,
	/** Double-slope (count up and down), interrupt/event at Top (Top is the
	 *  PER register, output active when count is greater than CCx). */
	TCC_WAVE_GENERATION_DOUBLE_SLOPE_TOP = 7,
};

/**
 * \brief Polarity of TCC wave generation on channels
 */
enum tcc_wave_polarity {
	/** Wave output initialized to ~DIR */
	TCC_WAVE_POLARITY_0,
	/** Wave output initialized to DIR */
	TCC_WAVE_POLARITY_1
};

/**
 * \brief TCC pattern generator for outputs
 */
enum tcc_output_pattern {
	/** SWAP Output pattern is not used */
	TCC_OUTPUT_PATTERN_DISABLE,
	/** Pattern 0 is applied to SWAP output */
	TCC_OUTPUT_PATTERN_0,
	/** Pattern 1 is applied to SWAP output */
	TCC_OUTPUT_PATTERN_1
};

/**
 * \brief Ramp Operations of TCC wave generation
 *  In ramp operation, each two period cycles are marked as cycle A and B,
 *  the index 0 represents cycle A and 1 represents cycle B.
 */
enum tcc_ramp {
	/** Default timer/counter PWM operation. */
	TCC_RAMP_RAMP1 = 0,
	/** CC0 compare outputs are for channel 1 and 0.
	 *  Two consecutive cycles A and B are for channel 1 and 0. */
	TCC_RAMP_RAMP2A,
	/** CC0 and CC1 compare outputs are involved.
	 *  Two consecutive cycles A and B are for channel 1 and 0. */
	TCC_RAMP_RAMP2
};

/**
 * \brief Ramp Index of TCC wave generation
 */
enum tcc_ramp_index {
	/** Default, cycle index toggles. */
	TCC_RAMP_INDEX_DEFAULT,
	/** Force next cycle to be cycle B (set to 1). */
	TCC_RAMP_INDEX_FORCE_B,
	/** Force next cycle to be cycle A (clear to 0). */
	TCC_RAMP_INDEX_FORCE_A,
	/** Force next cycle keeping the same as current. */
	TCC_RAMP_INDEX_FORCE_KEEP
};

/**
 * \brief TCC output invertion
 */
enum tcc_output_invertion {
	/** output of WO[x] is disabled */
	TCC_OUTPUT_INVERTION_DISABLE,
	/** output of WO[x] is enabled */
	TCC_OUTPUT_INVERTION_ENABLE
};

/**
 * \brief TCC Counter reload action enum
 *
 * This enum specify how the counter and prescaler should reload.
 */
enum tcc_reload_action {
	/** The counter is reloaded/reset on the next GCLK and starts
	 * counting on the prescaler clock.
	 */
	TCC_RELOAD_ACTION_GCLK,
	/** The counter is reloaded/reset on the next prescaler clock
	 */
	TCC_RELOAD_ACTION_PRESC,
	/** The counter is reloaded/reset on the next GCLK, and the
	 * prescaler is restarted as well.
	 */
	TCC_RELOAD_ACTION_RESYNC
};


/**
 * \brief TCC clock prescaler values
 *
 * This enum is used to choose the clock prescaler
 * configuration. The prescaler divides the clock frequency of the TCC
 * module to make the counter count slower.
 */
enum tcc_clock_prescaler {
	/** Divide clock by 1 */
	TCC_CLOCK_PRESCALER_DIV1,
	/** Divide clock by 2 */
	TCC_CLOCK_PRESCALER_DIV2,
	/** Divide clock by 4 */
	TCC_CLOCK_PRESCALER_DIV4,
	/** Divide clock by 8 */
	TCC_CLOCK_PRESCALER_DIV8,
	/** Divide clock by 16 */
	TCC_CLOCK_PRESCALER_DIV16,
	/** Divide clock by 64 */
	TCC_CLOCK_PRESCALER_DIV64,
	/** Divide clock by 256 */
	TCC_CLOCK_PRESCALER_DIV256,
	/** Divide clock by 1024 */
	TCC_CLOCK_PRESCALER_DIV1024
};

/**
 * \brief TCC module count direction.
 *
 * Timer/Counter count direction.
 */
enum tcc_count_direction {
	/** Timer should count upward from zero to MAX. */
	TCC_COUNT_DIRECTION_UP,
	/** Timer should count downward to zero from MAX. */
	TCC_COUNT_DIRECTION_DOWN,
};

/**
 * \brief Action to perform when the TCC module is triggered by events.
 *
 * Event action to perform when the module is triggered by events.
 */
enum tcc_event_action {
	/** No event action. */
	TCC_EVENT_ACTION_OFF,
	/** Stop counting, the counter will maintain its current value, waveforms
	 *  are set to a defined Non-Recoverable State output
	 *  (\ref tcc_non_recoverable_state_output). */
	TCC_EVENT_ACTION_STOP,
	/** Re-trigger counter on event, may generate an event if the re-trigger
	 *  event output is enabled.
	 *  \note When re-trigger event action is enabled, enabling the counter
	 *        will not start the counter but waiting incoming event. */
	TCC_EVENT_ACTION_RETRIGGER,
	/** Start counter when previously stopped. */
	TCC_EVENT_ACTION_START,
	/** Count events (increment or decrement, dependint on DIR). */
	TCC_EVENT_ACTION_COUNT_EVENT,
	/** The event source must be an asynchronous event, input value will
	 *  overrides the direction settings (input low: counting up, input high
	 *  counting down). */
	TCC_EVENT_ACTION_DIR_CONTROL,
	/** Increment or decrement counter on event, depending on direction. */
	TCC_EVENT_ACTION_INCREMENT,
	/** Decrement or increment counter on event, depending on direction. */
	TCC_EVENT_ACTION_DECREMENT,
	/** Count during active state of asynchronous event. */
	TCC_EVENT_ACTION_COUNT_DURING_ACTIVE,

	/** Store period in capture register 0, pulse width in capture
	 *  register 1.
	 */
	TCC_EVENT_ACTION_PERIOD_PULSE_WIDTH_CAPTURE,
	/** Store pulse width in capture register 0, period in capture
	 *  register 1.
	 */
	TCC_EVENT_ACTION_PULSE_WIDTH_PERIOD_CAPTURE,

	/** Generate Non-Recoverable Fault on event. */
	TCC_EVENT_ACTION_NON_RECOVERABLE_FAULT,
};


/**
 * \brief Action to perform when the TCC module is triggered by event0.
 *
 * Event action to perform when the module is triggered by event0.
 */
enum tcc_event0_action {
	/** No event action. */
	TCC_EVENT0_ACTION_OFF                   = TCC_EVENT_ACTION_OFF,
	/** Re-trigger Counter on event. */
	TCC_EVENT0_ACTION_RETRIGGER             = TCC_EVENT_ACTION_RETRIGGER,
	/** Count events (increment or decrement, dependint on DIR). */
	TCC_EVENT0_ACTION_COUNT_EVENT           = TCC_EVENT_ACTION_COUNT_EVENT,
	/** Start counter on event. */
	TCC_EVENT0_ACTION_START                 = TCC_EVENT_ACTION_START,
	/** Increment counter on event. */
	TCC_EVENT0_ACTION_INCREMENT             = TCC_EVENT_ACTION_INCREMENT,
	/** Count during active state of asynchronous event. */
	TCC_EVENT0_ACTION_COUNT_DURING_ACTIVE   = TCC_EVENT_ACTION_COUNT_DURING_ACTIVE,

	/** Generate Non-Recoverable Fault on event. */
	TCC_EVENT0_ACTION_NON_RECOVERABLE_FAULT = TCC_EVENT_ACTION_NON_RECOVERABLE_FAULT
};

/**
 * \brief Action to perform when the TCC module is triggered by event1.
 *
 * Event action to perform when the module is triggered by event1.
 */
enum tcc_event1_action {
	/** No event action. */
	TCC_EVENT1_ACTION_OFF                   = TCC_EVENT_ACTION_OFF,
	/** Re-trigger Counter on event. */
	TCC_EVENT1_ACTION_RETRIGGER             = TCC_EVENT_ACTION_RETRIGGER,
	/** The event source must be an asynchronous event, input value will
	 *  overrides the direction settings (input low: counting up, input high
	 *  counting down). */
	TCC_EVENT1_ACTION_DIR_CONTROL           = TCC_EVENT_ACTION_DIR_CONTROL,
	/** Stop counter on event. */
	TCC_EVENT1_ACTION_STOP                  = TCC_EVENT_ACTION_STOP,
	/** Decrement on event */
	TCC_EVENT1_ACTION_DECREMENT             = TCC_EVENT_ACTION_DECREMENT,

	/** Store period in capture register 0, pulse width in capture
	 *  register 1.
	 */
	TCC_EVENT1_ACTION_PERIOD_PULSE_WIDTH_CAPTURE  = TCC_EVENT_ACTION_PERIOD_PULSE_WIDTH_CAPTURE,
	/** Store pulse width in capture register 0, period in capture
	 *  register 1.
	 */
	TCC_EVENT1_ACTION_PULSE_WIDTH_PERIOD_CAPTURE  = TCC_EVENT_ACTION_PULSE_WIDTH_PERIOD_CAPTURE,

	/** Generate Non-Recoverable Fault on event. */
	TCC_EVENT1_ACTION_NON_RECOVERABLE_FAULT = TCC_EVENT_ACTION_NON_RECOVERABLE_FAULT
};

/**
 * \brief On which part of the counter cycle the counter event output is generated
 */
enum tcc_event_generation_selection {
	/** Counter Event is generated when a new counter cycle starts */
	TCC_EVENT_GENERATION_SELECTION_START,
	/** Counter Event is generated when a counter cycle ends */
	TCC_EVENT_GENERATION_SELECTION_END,
	/** Counter Event is generated when a counter cycle ends, except for the
	 *  first and last cycles */
	TCC_EVENT_GENERATION_SELECTION_BETWEEN,
	/** Counter Event is generated when a new counter cycle starts or ends */
	TCC_EVENT_GENERATION_SELECTION_BOUNDARY
};

/**
 * \brief TCC channel operation modes
 */
enum tcc_channel_function {
	/** TCC channel performs compare operation. */
	TCC_CHANNEL_FUNCTION_COMPARE,
	/** TCC channel performs capture operation. */
	TCC_CHANNEL_FUNCTION_CAPTURE
};

/**
 * \brief TCC input event enable/disable/configure structure.
 */
struct tcc_input_event_config {
	/** Event action on incoming event. */
	enum tcc_event_action action;
	/** Modify event action */
	bool modify_action;
	/** Invert incoming event input line. */
	bool invert;
};

/**
 * \brief TCC output event enable/disable/configure structure.
 */
struct tcc_output_event_config {
	/** Event output action for counter event generation. */
	enum tcc_event_generation_selection generation_selection;
	/** Modify output action */
	bool modify_generation_selection;
};

/**
 * \brief TCC event enable/disable structure.
 *
 * Event flags for the \ref tcc_enable_events() and \ref tcc_disable_events().
 */
struct tcc_events {
	/** Input events configuration */
	struct tcc_input_event_config input_config[2];
	/** Output event configuration */
	struct tcc_output_event_config output_config;

	/** Perform the configured event action when an incoming event is
	 *  signalled. */
	bool on_input_event_perform_action[2];

	/** Perform the configured event action when an incoming channel event is
	 *  signalled */
	bool on_event_perform_channel_action[TCC_NUM_CHANNELS];

	/** Generate an output event on a channel capture/match. */
	bool generate_event_on_channel[TCC_NUM_CHANNELS];

	/** Generate an output event on counter overflow/underflow. */
	bool generate_event_on_counter_overflow;
	/** Generate an output event on counter retrigger */
	bool generate_event_on_counter_retrigger;
	/** Generate an output event on counter boundary.
	 *  See \ref tcc_event_output_action */
	bool generate_event_on_counter_event;
};

/**
 * \brief Configuration struct for TCC module base counter
 */
struct tcc_counter_config {
	/** Value to initialize the count register */
	uint32_t count;
	/** Period/top and period/top buffer values for counter */
	uint32_t period;

	/** When \c true, one-shot will stop the TC on next hardware or software
	 *  re-trigger event or overflow/underflow.
	 */
	bool oneshot;

	/** Specifies the direction for the TC to count. */
	enum tcc_count_direction direction;

	/** GCLK generator used to clock the peripheral. */
	enum gclk_generator clock_source;
	/** Specifies the prescaler value for GCLK_TCC. */
	enum tcc_clock_prescaler clock_prescaler;
	/** Specifies the reload or reset time of the counter and prescaler
	 *  resynchronization on a re-trigger event for the TCC.
	 */
	enum tcc_reload_action reload_action;
};

/**
 * \brief Configuration struct for TCC module capture
 */
struct tcc_capture_config {
	/** Channel functions selection (capture/match) */
	enum tcc_channel_function channel_function[TCC_NUM_CHANNELS];
};

/**
 * \brief Configuration struct for TCC module match/wave generation
 */
struct tcc_match_wave_config {
	/** Channel functions selection (capture/match) */
	enum tcc_channel_function channel_function[TCC_NUM_CHANNELS];

	/** Specifies polarity for match output waveform generation. */
	enum tcc_wave_polarity wave_polarity[TCC_NUM_CHANNELS];
	/** Specifies which waveform generation mode to use. */
	enum tcc_wave_generation wave_generation;
	/** Specifies Ramp mode for waveform generation. */
	enum tcc_ramp wave_ramp;

	/** Value to be used for compare match on each channel. */
	uint32_t match[TCC_NUM_CHANNELS];
};

/**
 * \brief Configuration struct for TCC module waveform extension processes
 */
struct tcc_wave_extension_config {
	/** Invert waveform final outputs lines. */
	bool invert[TCC_NUM_WAVE_OUTPUTS];
};

/**
 * \brief Configuration struct for TCC module input/output pins
 */
struct tcc_pins_config {
	/** Specifies pin output for each channel. */
	uint32_t wave_out_pin[TCC_NUM_WAVE_OUTPUTS];
	/** Specifies MUX setting for each output channel pin. */
	uint32_t wave_out_pin_mux[TCC_NUM_WAVE_OUTPUTS];
	/** When \c true, PWM output pin for the given channel is enabled. */
	bool enable_wave_out_pin[TCC_NUM_WAVE_OUTPUTS];
};

/**
 * \brief TCC configuration structure.
 *
 * Configuration struct for a TCC instance. This structure should be
 * initialized by the \ref tcc_get_config_defaults function before being
 * modified by the user application.
 */
struct tcc_config {

	/** TCC base timer/counter configurations. */
	struct tcc_counter_config counter;

	/** TCC match/capture configurations. */
	union {
		/** TCC capture configurations */
		struct tcc_capture_config capture;
		/** TCC compare/wave generation configurations */
		struct tcc_match_wave_config compare;
		/** TCC compare/wave generation configurations */
		struct tcc_match_wave_config wave;
	};

	/** TCC waveform extension configurations. */
	struct tcc_wave_extension_config wave_ext;

	/** TCC output pins configuration. */
	struct tcc_pins_config pins;

	/** When \c true the module is enabled during standby. */
	bool run_in_standby;
};

#if TCC_ASYNC == true
/* Forward Declaration for the device instance */
struct tcc_module;

/* Type of the callback functions */
typedef void (*tcc_callback_t)(struct tcc_module *const module);
#endif

/**
 * \brief TCC software device instance structure.
 *
 * TCC software instance structure, used to retain software state information
 * of an associated hardware module instance.
 *
 * \note The fields of this structure should not be altered by the user
 *       application; they are reserved for module-internal use only.
 */
struct tcc_module {
	/** Hardware module pointer of the associated Timer/Counter peripheral. */
	Tcc *hw;

#  if TCC_ASYNC == true
	/** Array of callbacks */
	tcc_callback_t callback[TCC_CALLBACK_N];
	/** Bit mask for callbacks registered */
	uint32_t register_callback_mask;
	/** Bit mask for callbacks enabled */
	uint32_t enable_callback_mask;
#  endif
};

#if !defined(__DOXYGEN__)
uint8_t _tcc_get_inst_index(
		Tcc *const hw);
#endif

/**
 * \name Driver Initialization and Configuration
 * @{
 */

/**
 * \brief Determines if the hardware module(s) are currently synchronizing to the bus.
 *
 * Checks to see if the underlying hardware peripheral module(s) are currently
 * synchronizing across multiple clock domains to the hardware bus, This
 * function can be used to delay further operations on a module until such time
 * that it is ready, to prevent blocking delays for synchronization in the
 * user application.
 *
 * \param[in]  module_inst   Pointer to the software module instance struct
 *
 * \return Synchronization status of the underlying hardware module(s).
 *
 * \retval true if the module has completed synchronization
 * \retval false if the module synchronization is ongoing
 */
static inline bool tcc_is_syncing(
		const struct tcc_module *const module_inst)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);

	return (module_inst->hw->SYNCBUSY.reg > 0);
}


void tcc_get_config_defaults(
		struct tcc_config *const config,
		Tcc *const hw);

enum status_code tcc_init(
		struct tcc_module *const module_inst,
		Tcc *const hw,
		const struct tcc_config *const config);

/** @} */

/**
 * \name Event Management
 * @{
 */

enum status_code tcc_enable_events(
		struct tcc_module *const module_inst,
		struct tcc_events *const events);

void tcc_disable_events(
		struct tcc_module *const module_inst,
		struct tcc_events *const events);

/** @} */

/**
 * \name Enable/Disable/Reset
 * @{
 */

/**
 * \brief Enable the TCC module.
 *
 * Enables a TCC module that has been previously initialized. The counter will
 * start when the counter is enabled.
 *
 * \note When the counter is configured to re-trigger on an event, the counter
 *       will not start until the start function is used.
 *
 * \param[in]  module_inst   Pointer to the software module instance struct
 */
static inline void tcc_enable(
		const struct tcc_module *const module_inst)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);

	/* Get a pointer to the module's hardware instance */
	Tcc *const tcc_module = module_inst->hw;

	while (tcc_module->SYNCBUSY.reg & TCC_SYNCBUSY_ENABLE) {
		/* Wait for sync */
	}

	/* Enable TCC module */
	tcc_module->CTRLA.reg |= TCC_CTRLA_ENABLE;
}

/**
 * \brief Disables the TCC module.
 *
 * Disables a TCC module and stops the counter.
 *
 * \param[in]  module_inst   Pointer to the software module instance struct
 */
static inline void tcc_disable(
		const struct tcc_module *const module_inst)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);

	/* Get a pointer to the module's hardware instance */
	Tcc *const tcc_module = module_inst->hw;

	while (tcc_module->SYNCBUSY.reg & TCC_SYNCBUSY_ENABLE) {
		/* Wait for sync */
	}

	/* Disable TCC module */
	tcc_module->CTRLA.reg  &= ~TC_CTRLA_ENABLE;
}

/**
 * \brief Resets the TCC module.
 *
 * Resets the TCC module, restoring all hardware module registers to their
 * default values and disabling the module. The TCC module will not be
 * accessible while the reset is being performed.
 *
 * \note When resetting a 32-bit counter only the master TCC module's instance
 *       structure should be passed to the function.
 *
 * \param[in]  module_inst    Pointer to the software module instance struct
 *
 */
static inline void tcc_reset(
		const struct tcc_module *const module_inst)
{
	/* Sanity check arguments  */
	Assert(module_inst);
	Assert(module_inst->hw);

	/* Get a pointer to the module hardware instance */
	Tcc *const tcc_module = module_inst->hw;

	/* Disable this module if it is running */
	if (tcc_module->CTRLA.reg & TCC_CTRLA_ENABLE) {
		tcc_disable(module_inst);
		while (tcc_is_syncing(module_inst)) {
			/* wait while module is disabling */
		}
	}

	/* Reset this TC module */
	tcc_module->CTRLA.reg  |= TCC_CTRLA_SWRST;
}

/** @} */


/**
 * \name Set/Toggle Count Direction
 * @{
 */

/**
 * \brief Sets TCC module count direction.
 *
 * Sets the current timer count direction of a initialized TCC module. The
 * specified TCC module may be started or stopped.
 *
 * \param[in] module_inst  Pointer to the software module instance struct
 * \param[in] dir          New timer count direction to set
 */
static inline void tcc_set_count_direction(
		const struct tcc_module *const module_inst,
		enum tcc_count_direction dir)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);

	/* Get a pointer to the module's hardware instance */
	Tcc *const tcc_module = module_inst->hw;

	while (tcc_module->SYNCBUSY.bit.CTRLB) {
		/* Wait for sync */
	}

	/* Set count direction */
	if (TCC_COUNT_DIRECTION_DOWN == dir) {
		tcc_module->CTRLBSET.reg = TCC_CTRLBSET_DIR;
		return;
	}
	tcc_module->CTRLBCLR.reg = TCC_CTRLBCLR_DIR;
}

/**
 * \brief Toggles TCC module count direction.
 *
 * Toggles the current timer count direction of a initialized TCC module. The
 * specified TCC module may be started or stopped.
 *
 * \param[in] module_inst  Pointer to the software module instance struct
 */
static inline void tcc_toggle_count_direction(
		const struct tcc_module *const module_inst)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);

	/* Get a pointer to the module's hardware instance */
	Tcc *const tcc_module = module_inst->hw;

	while (tcc_module->SYNCBUSY.bit.CTRLB) {
		/* Wait for sync */
	}
	bool dir_value_1 = tcc_module->CTRLBSET.bit.DIR;
	if (dir_value_1) {
		tcc_module->CTRLBCLR.reg = TCC_CTRLBCLR_DIR;
	} else {
		tcc_module->CTRLBSET.reg = TCC_CTRLBSET_DIR;
	}
}

/** @} */

/**
 * \name Get/Set Count Value
 * @{
 */

uint32_t tcc_get_count_value(
		const struct tcc_module *const module_inst);

enum status_code tcc_set_count_value(
		const struct tcc_module *const module_inst,
		const uint32_t count);

/** @} */

/**
 * \name Start/Stop Counter
 * @{
 */

/**
 * \brief Stops the counter.
 *
 * This function will stop the counter. When the counter is stopped
 * the value in the count value is set to 0 if the counter was
 * counting up, or max or the top value if the counter was counting
 * down when stopped.
 *
 * \param[in]  module_inst   Pointer to the software module instance struct
 */
static inline void tcc_stop_counter(
		const struct tcc_module *const module_inst)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);

	/* Get a pointer to the module's hardware instance */
	Tcc *const tcc_module = module_inst->hw;
	uint32_t last_cmd;

	/* Wait last command done */
	do {
		while (tcc_module->SYNCBUSY.bit.CTRLB) {
			/* Wait for sync */
		}
		last_cmd = tcc_module->CTRLBSET.reg & TCC_CTRLBSET_CMD_Msk;
		if (TCC_CTRLBSET_CMD_NONE == last_cmd) {
			break;
		} else if (TCC_CTRLBSET_CMD_STOP == last_cmd) {
			/* Command have been issued */
			return;
		} else if (TCC_CTRLBSET_CMD_RETRIGGER == last_cmd) {
			/* Cancel RETRIGGER command and issue STOP */
			tcc_module->CTRLBCLR.reg = TCC_CTRLBCLR_CMD_Msk;
		}
	} while (1);

	/* Write command to execute */
	tcc_module->CTRLBSET.reg = TCC_CTRLBSET_CMD_STOP;
}

/**
 * \brief Starts the counter from beginning.
 *
 * Restarts an initialized TC module's counter.
 *
 * \param[in]  module_inst   Pointer to the software module instance struct
 */
static inline void tcc_restart_counter(
		const struct tcc_module *const module_inst)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);

	/* Get a pointer to the module's hardware instance */
	Tcc *const tcc_module = module_inst->hw;
	uint32_t last_cmd;

	/* Wait last command done */
	do {
		while (tcc_module->SYNCBUSY.bit.CTRLB) {
			/* Wait for sync */
		}
		last_cmd = tcc_module->CTRLBSET.reg & TCC_CTRLBSET_CMD_Msk;
		if (TCC_CTRLBSET_CMD_NONE == last_cmd) {
			break;
		} else if (TCC_CTRLBSET_CMD_RETRIGGER == last_cmd) {
			/* Command have been issued */
			return;
		} else if (TCC_CTRLBSET_CMD_STOP == last_cmd) {
			/* Cancel STOP command and issue RETRIGGER */
			tcc_module->CTRLBCLR.reg = TCC_CTRLBCLR_CMD_Msk;
		}
	} while (1);

	/* Write command to execute */
	tcc_module->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER;
}

/** @} */

/**
 * \name Get Capture Set Compare
 * @{
 */

uint32_t tcc_get_capture_value(
		const struct tcc_module *const module_inst,
		const enum tcc_match_capture_channel channel_index);

enum status_code tcc_set_compare_value(
		const struct tcc_module *const module_inst,
		const enum tcc_match_capture_channel channel_index,
		const uint32_t compare);

/** @} */

/**
 * \name Set Top Value
 * @{
 */

/**
 * \brief Set the timer TOP/PERIOD value.
 *
 * This function writes the top value.
 *
 * When using MFRQ, the top value is defined by the CC0 register value, for all
 * other waveforms operation the top value is defined by PER register value.
 *
 * \param[in]  module_inst   Pointer to the software module instance struct
 * \param[in]  top_value     New timer TOP value to set
 *
 * \return Status of the TOP set procedure.
 *
 * \retval STATUS_OK              The timer TOP value was updated successfully
 * \retval STATUS_ERR_INVALID_ARG The configured TC module counter size in the
 *                                module instance is invalid.
 */
enum status_code tcc_set_top_value(
		const struct tcc_module *const module_inst,
		const uint32_t top_value);

/** @} */


/**
 * \name Set Output Pattern
 * @{
 */

enum status_code tcc_set_pattern(
		const struct tcc_module *const module_inst,
		const uint32_t line_index,
		const enum tcc_output_pattern pattern);

/** @} */


/**
 * \name Set Ramp Index
 * @{
 */

/**
 * \brief Sets TCC module ramp index on next cycle
 *
 * Force cycle A and cycle B changes in RAMP2 and RAMP2A operation.
 * See \ref tcc_ramp.
 *
 * \param[in]  module_inst Pointer to the software module instance struct
 * \param[in]  ramp_index  Ramp index (\ref tcc_ramp_index) of the next cycle
 */
static inline void tcc_set_ramp_index(
		const struct tcc_module *const module_inst,
		const enum tcc_ramp_index ramp_index)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);

	/* Get a pointer to the module's hardware instance */
	Tcc *const tcc_module = module_inst->hw;
	uint32_t last_cmd;

	/* Wait last command done */
	do {
		while (tcc_module->SYNCBUSY.bit.CTRLB) {
			/* Wait for sync */
		}
		if (TCC_RAMP_INDEX_DEFAULT == ramp_index) {
			/* Cancel pending command */
			tcc_module->CTRLBCLR.reg = TCC_CTRLBSET_IDXCMD_DISABLE;
			return;
		}
		last_cmd = tcc_module->CTRLBSET.reg & TCC_CTRLBSET_IDXCMD_Msk;
		if (TCC_CTRLBSET_IDXCMD_DISABLE == last_cmd) {
			break;
		} else if (TCC_CTRLBSET_CMD(ramp_index) == last_cmd) {
			/* Command have been issued */
			return;
		}
	} while (1);

	/* Write command to execute */
	tcc_module->CTRLBSET.reg = TCC_CTRLBSET_CMD(ramp_index);
}

/** @} */

/**
 * \name Status Management
 * @{
 */

/**
 * \brief Checks if the timer/counter is running.
 *
 * \param[in] module_inst  Pointer to the TCC software instance struct
 *
 * \return Module running status.
 *
 * \retval true The timer/counter is running.
 * \retval false The timer/counter is stopped.
 */
static inline bool tcc_is_running(
		struct tcc_module *const module_inst)
{
	/* Sanity check arguments */
	Assert(module_inst);
	Assert(module_inst->hw);

	return !module_inst->hw->STATUS.bit.STOP;
}

uint32_t tcc_get_status(
		struct tcc_module *const module_inst);

void tcc_clear_status(
		struct tcc_module *const module_inst,
		const uint32_t status_flags);

/** @} */

/** @} */

#ifdef __cplusplus
}
#endif

/**
 * \page asfdoc_sam0_tcc_extra Extra Information for TCC Driver
 *
 * \section asfdoc_sam0_tcc_extra_acronyms Acronyms
 * The table below presents the acronyms used in this module:
 *
 * <table>
 *  <tr>
 *      <th>Acronym</th>
 *      <th>Description</th>
 *  </tr>
  * <tr>
 *      <td>DMA</td>
 *      <td>Direct Memory Access</td>
 *  </tr>
 *  <tr>
 *      <td>TC</td>
 *      <td>Timer Counter</td>
 *  </tr>
 *  <tr>
 *      <td>TCC</td>
 *      <td>Timer Counter for Control Applications</td>
 *  </tr>
 *  <tr>
 *      <td>PWM</td>
 *      <td>Pulse Width Modulation</td>
 *  </tr>
 *  <tr>
 *      <td>PWP</td>
 *      <td>Pulse Width Period</td>
 *  </tr>
 *  <tr>
 *      <td>PPW</td>
 *      <td>Period Pulse Width</td>
 *  </tr>
 * </table>
 *
 *
 * \section asfdoc_sam0_tcc_extra_dependencies Dependencies
 * This driver has the following dependencies:
 *
 *  - \ref asfdoc_sam0_system_pinmux_group "System Pin Multiplexer Driver"
 *
 *
 * \section asfdoc_sam0_tcc_extra_errata Errata
 * There are no errata related to this driver.
 *
 *
 * \section asfdoc_sam0_tcc_extra_history Module History
 * An overview of the module history is presented in the table below, with
 * details on the enhancements and fixes made to the module since its first
 * release. The current version of this corresponds to the newest version in
 * the table.
 *
 * <table>
 *  <tr>
 *      <th>Changelog</th>
 *  </tr>
 *  <tr>
 *      <td>Initial Release</td>
 *  </tr>
 * </table>
 */

/**
 * \page asfdoc_sam0_tcc_exqsg Examples for TCC Driver
 *
 * This is a list of the available Quick Start guides (QSGs) and example
 * applications for \ref asfdoc_sam0_tcc_group. QSGs are simple examples with
 * step-by-step instructions to configure and use this driver in a selection of
 * use cases. Note that QSGs can be compiled as a standalone application or be
 * added to the user application.
 *
 *  - \subpage asfdoc_sam0_tcc_basic_use_case
 * \if TCC_CALLBACK_MODE
 *  - \subpage asfdoc_sam0_tcc_callback_use_case
 * \endif
 *  - \subpage asfdoc_sam0_tcc_dma_use_case
 *
 * \page asfdoc_sam0_tcc_document_revision_history Document Revision History
 *
 * <table>
 *  <tr>
 *      <th>Doc. Rev.</td>
 *      <th>Date</td>
 *      <th>Comments</td>
 *  </tr>
 *  <tr>
 *      <td>A</td>
 *      <td>01/2014</td>
 *      <td>Initial release</td>
 *  </tr>
 * </table>
 */

#endif /* TCC_H_INCLUDED */
