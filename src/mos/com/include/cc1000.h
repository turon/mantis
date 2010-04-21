//  This file is part of MOS, the MANTIS Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (c) 2002 - 2007 University of Colorado, Boulder
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//       * Redistributions of source code must retain the above copyright
//         notice, this list of conditions and the following disclaimer.
//       * Redistributions in binary form must reproduce the above
//         copyright notice, this list of conditions and the following
//         disclaimer in the documentation and/or other materials provided
//         with the distribution. 
//       * Neither the name of the MANTIS Project nor the names of its
//         contributors may be used to endorse or promote products derived
//         from this software without specific prior written permission.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//   POSSIBILITY OF SUCH DAMAGE.

/*********************************************************************************************** */
/* File:    cc1000.h                                                                             */
/* This file contains the  register name definitions for used in the cc1000 control module.      */
/* Part of the constant register definition is adapted from the CC1000Const.h used in the tinyOS */
/* which you could find at the Berkeley tinyOS website                                           */
/*                                                                                               */
/* Author:         huid@colorado.edu                                                             */
/*************************************************************************************************/

/** @file cc1000.h
 * @brief This module provides the functionality of the CC1000 radio
 * 
 * @author Adapted from the cc1000 source code provided by chipcon company.
 * @date 04/27/200?
 */

#ifndef _CC1000_H_
#define _CC1000_H_

#include "mos.h"
//#include "config.h"
#include "stdarg.h"
#include "com.h"

#ifndef PLATFORM_LINUX
// The registers defined in the CC1000 data sheet

#define CC1000_MAIN            0x00
#define CC1000_FREQ_2A         0x01
#define CC1000_FREQ_1A         0x02
#define CC1000_FREQ_0A         0x03
#define CC1000_FREQ_2B         0x04
#define CC1000_FREQ_1B         0x05
#define CC1000_FREQ_0B         0x06
#define CC1000_FSEP1           0x07
#define CC1000_FSEP0           0x08
#define CC1000_CURRENT         0x09
#define CC1000_FRONT_END       0x0A
#define CC1000_PA_POW          0x0B
#define CC1000_PLL             0x0C
#define CC1000_LOCK            0x0D
#define CC1000_CAL             0x0E
#define CC1000_MODEM2          0x0F
#define CC1000_MODEM1          0x10
#define CC1000_MODEM0          0x11
#define CC1000_MATCH           0x12
#define CC1000_FSCTRL          0x13
#define CC1000_FSHAPE7         0x14
#define CC1000_FSHAPE6         0x15
#define CC1000_FSHAPE5         0x16
#define CC1000_FSHAPE4         0x17
#define CC1000_FSHAPE3         0x18
#define CC1000_FSHAPE2         0x19
#define CC1000_FSHAPE1         0x1A
#define CC1000_FSDELAY         0x1B
#define CC1000_PRESCALER       0x1C
#define CC1000_TEST6           0x40
#define CC1000_TEST5           0x41
#define CC1000_TEST4           0x42
#define CC1000_TEST3           0x43
#define CC1000_TEST2           0x44
#define CC1000_TEST1           0x45
#define CC1000_TEST0           0x46

/* Define each pin for the CC1000 */

// The bit definitions in the MAIN register
#define CC1000_RXTX		7
#define CC1000_F_REG		6
#define CC1000_RX_PD		5
#define CC1000_TX_PD		4
#define CC1000_FS_PD		3
#define CC1000_CORE_PD		2
#define CC1000_BIAS_PD		1
#define CC1000_RESET_N		0


// The bit definitions in the CURRENT register
//CURRENT[7-4],  control of current in VCO core for TX and RX
#define CC1000_VCO_CURRENT	4
//control of current in VCO buffer for LO drive
#define CC1000_LO_DRIVE		2
//control of current in VCO buffer for PA
#define CC1000_PA_DRIVE		0

// The bit definitions in the FRONT_END register
#define CC1000_BUF_CURRENT	5
#define CC1000_LNA_CURRENT	3
#define CC1000_IF_RSSI		1
#define CC1000_XOSC_BYPASS	0

// The bit definitions in the PA_POW register
#define CC1000_PA_HIGHPOWER	4
#define CC1000_PA_LOWPOWER	0

// The bit definitions in the PLL register
#define CC1000_EXT_FILTER	7
#define CC1000_REFDIV		3
#define CC1000_ALARM_DISABLE	2
#define CC1000_ALARM_H		1
#define CC1000_ALARM_L		0

// The Bit Definitions in the LOCK register
#define CC1000_LOCK_SELECT	4
#define CC1000_PLL_LOCK_ACCURACY	3
#define CC1000_PLL_LOCK_LENGTH	2
#define CC1000_LOCK_INSTANT	1
#define CC1000_LOCK_CONTINUOUS	0

// Bit Definitions in the CAL register 
#define CC1000_CAL_START	7
#define CC1000_CAL_DUAL		6
#define CC1000_CAL_WAIT		5
#define CC1000_CAL_CURRENT	4
#define CC1000_CAL_COMPLETE	3
#define CC1000_CAL_ITERATE	0  //3 bits

// Bit definitions in the MODEM2 register
#define CC1000_PEAKDETECT		7 
#define CC1000_PEAK_LEVEL_OFFSET	0 //7 bits

// Bit Definitions in MODEM1 Register 
#define CC1000_MLIMIT		5
#define CC1000_LOCK_AVG_IN	4
#define CC1000_LOCK_AVG_MODE	3
#define CC1000_SETTLING		1
#define CC1000_MODEM_RESET_N	0

// Bit Definitions in  MODEM0 Register 
#define CC1000_BAUDRATE		4
#define CC1000_DATA_FORMAT	2
#define CC1000_XOSC_FREQ	0

// Bit Definitions in MATCH Register 
#define CC1000_RX_MATCH		4
#define CC1000_TX_MATCH		0

// Bit Definitions in FSCTLR Register 
#define CC1000_DITHER1		3
#define CC1000_DITHER0		2
#define CC1000_SHAPE		1
#define CC1000_FS_RESET_N	0

// Bit Definitions in PRESCALER Register 
#define CC1000_PRE_SWING		6
#define CC1000_PRE_CURRENT	4
#define CC1000_IF_INPUT		3
#define CC1000_IF_FRONT		2

// Bit Definitions in TEST6 Register 
#define CC1000_LOOPFILTER_TP1	7
#define CC1000_LOOPFILTER_TP2	6
#define CC1000_CHP_OVERRIDE	5
#define CC1000_CHP_CO		0

// Bit Definitions in TEST5 Register 
#define CC1000_CHP_DISALBE	5
#define CC1000_VCO_OVERRIDE	4
#define CC1000_VCO_AO		0

// Bit Definitions in TEST3 Register 
#define CC1000_BREAK_LOOP		4
#define CC1000_CAL_DAC_OPEN	0

// Bit Definitons in TEST2 Register 
#define CC1000_CHP_CURRENT     0

// Bit Definitons in TEST1 Register 
#define CC1000_CAL_DAC        0

// Bit Definitons in TEST0 
#define CC1000_VCO_ARRAY 0

// The Number of the total frequencies available to use
#define FREQS_NUM 0x1e

// The Number of the values defined for each register
#define PARAM_NUM 38
#define SETTING_NUM 6

//the index of the registers in the array
#define MAIN_INDEX	0x00 
#define FREQ2A_INDEX	0x01
#define FREQ1A_INDEX	0x02
#define FREQ0A_INDEX	0x03
#define FREQ2B_INDEX	0x04
#define FREQ1B_INDEX	0x05
#define FREQ0B_INDEX	0x06
#define FSEP1_INDEX	0x07
#define FSEP0_INDEX	0x08
#define RXCURRENT_INDEX	0x09
#define FRONTEND_INDEX	0x0a
#define PAPOW_INDEX	0x0b
#define RXPLL_INDEX	0x0c
#define LOCK_INDEX	0x0d
#define CAL_INDEX	0x0e
#define MODEM2_INDEX	0x0f
#define MODEM1_INDEX	0x10
#define MODEM0_INDEX	0x11
#define MATCH_INDEX	0x12
#define FSCTRL_INDEX	0x13
#define FSHAPE7_INDEX	0x14
#define FSHAPE6_INDEX	0x15
#define FSHAPE5_INDEX	0x16
#define FSHAPE4_INDEX	0x17
#define FSHAPE3_INDEX	0x18
#define FSHAPE2_INDEX	0x19
#define FSHAPE1_INDEX	0x1a
#define FSDELAY_INDEX	0x1b
#define PRESCALER_INDEX	0x1c
#define TEST6_INDEX	0x1d
#define TEST5_INDEX	0x1e
#define TEST4_INDEX	0x1f	
#define TEST3_INDEX	0x20
#define TEST2_INDEX	0x21
#define TEST1_INDEX	0x22
#define TEST0_INDEX	0x23
#define TXCURRENT_INDEX	0x24
#define TXPLL_INDEX	0x25

// The index for each frequency 
#define FREQ_902_265_MHZ	0x00
#define FREQ_902_791_MHZ	0x01
#define FREQ_903_318_MHZ        0x02
#define FREQ_903_845_MHZ        0x03
#define FREQ_904_371_MHZ	0x04
#define FREQ_904_898_MHZ	0x05
#define FREQ_905_425_MHZ        0x06
#define FREQ_905_951_MHZ        0x07
#define FREQ_906_478_MHZ	0x08
#define FREQ_907_004_MHZ	0x09
#define FREQ_907_531_MHZ        0x0a
#define FREQ_908_058_MHZ        0x0b
#define FREQ_908_584_MHZ	0x0c
#define FREQ_909_111_MHZ	0x0d
#define FREQ_909_638_MHZ        0x0e
#define FREQ_910_164_MHZ        0x0f
#define FREQ_910_691_MHZ	0x10
#define FREQ_911_217_MHZ	0x11
#define FREQ_911_744_MHZ        0x12
#define FREQ_912_271_MHZ        0x13
#define FREQ_912_797_MHZ	0x14
#define FREQ_913_324_MHZ	0x15
#define FREQ_913_851_MHZ        0x16
#define FREQ_914_377_MHZ        0x17
#define FREQ_914_907_MHZ        0x18
#define FREQ_915_430_MHZ        0x19
#define FREQ_915_957_MHZ        0x1a
#define FREQ_916_484_MHZ        0x1b
#define FREQ_917_010_MHZ        0x1c
#define FREQ_917_537_MHZ        0x1d


// Define the transmission speed for the radios 
#define BAUD_0_6_K 0x07     //0.6 KBaud
#define BAUD_1_2_K 0x17     //1.2 KBaud
#define BAUD_2_4_K 0x27     //2.4 KBaud
#define BAUD_4_8_K 0x37
#define BAUD_9_6_K 0x47
#define BAUD_19_2_K 0x57
#define BAUD_38_4_K 0x57

#define BAUD_19_2_K_LOWER_TEST4 0x25


#if defined(PLATFORM_NYMPH)
#define CONFIG_PORT      PORTC
#define CONFIG_PORT_DIR  DDRC
#define CONFIG_PORT_PIN  PINC
#define PALE             3
#define PDATA            2
#define PCLK             1
#elif defined(PLATFORM_MICA2)
#define CONFIG_PORT      PORTD
#define CONFIG_PORT_DIR  DDRD
#define CONFIG_PORT_PIN  PIND
#define PALE             4
#define PDATA            7
#define PCLK             6
#elif defined(PLATFORM_MICA2DOT)
#define CONFIG_PORT      PORTD
#define CONFIG_PORT_DIR  DDRD
#define CONFIG_PORT_PIN  PIND
#define PALE             5
#define PDATA            7
#define PCLK             6
#else
#error "Radio port/pins undefined"
#endif

// Mode definitions
#define CC1000_MODE_RX    0x00
#define CC1000_MODE_TX    0x01
#define CC1000_MODE_PD    0x02
#define CC1000_MODE_SLEEP 0x03

/* CC1000 ioctl requests */
/* set transmit power, takes uint8_t argument */
#define CC1000_TX_POWER 0
#define CC1000_RSSI     1
#define CC1000_FREQ     2
#define CC1000_RTS      3
#define CC1000_NO_RTS   4
#define CC1000_GET_TX_POWER 5

//generic setting of radio power
#define RADIO_TX_POWER CC1000_TX_POWER

/** @brief Radio is on. */
#define MODE_ON 0 
/** @brief Radio is off.*/ 
#define MODE_OFF 1 

/*some constants*/
#define FREQ 0x03

/** @brief Number of preamble bytes you need to see. */
#define PREAMBLE_THRESH 6
/** @brief Number of preamble bytes you need to send. */ 
#define PREAMBLE_LEN 16
/** @brief Actual byte to send as preamble */
#define PREAMBLE_BYTE 0xAA

#define FLUSH_BYTE 0xff

// Assistant functions for delaying.
// Will be moved to the timer module in the future
/** @brief Minimum delay.
 */
void delay_null();		

// init the radio with specific frequency
/** @brief Init the cc1000 radio.
 * The initial procedure is in the cc1000 data sheet
 * @param freq Assigned frequency (FREQ_XXX_XXX_MHZ in cc1000.h)
 */
void cc1000_init(uint8_t freq);

// set the radio to specific frequency
/** @brief Set the radio to a specific frequency.
 * @param newFreq Frequency to set the radio to (FREQ_XXX_XXX_MHZ in cc1000.h)
 */
void cc1000_set(uint8_t newFreq);

/** @brief Set the radio to a specific frequency.
 * @param new_freq Frequency to set the radio to (FREQ_XXX_XXX_MHZ in cc1000.h)
 */
void cc1000_change_freq(uint8_t new_freq);

/** @brief Calibrates the CC1000, required
 *
 * Follows the steps listed in the cc1000 data sheet
 */
void cc1000_calibrate();

/** @brief Write a value to a single CC1000 register
 * @param addr Address of the register to write to
 * @param data Data to be written
 */
void cc1000_write(uint8_t addr, uint8_t data);

/** @brief Write a value to a single CC1000 register
 * @param addranddata Address of register and data to be written,
 *                     stuffed in the same variable.
 */
void cc1000_write_word(short addranddata);

/** @brief Reads from a single CC1000 register.
 * @param addr Address of the register to read from
 * @return Data read
 */
uint8_t cc1000_read(uint8_t addr);

/** @brief Get the current transmit mode of the radio
 */
inline uint8_t cc1000_get_mode (void);

void cc1000_set_power(uint8_t powerLevel);

/** @brief Puts the cc1000 into different modes.
 * @param mode Mode to set: CC1000_RX_MODE, CC1000_TX_MODE,
                            CC1000_PD_MODE, CC1000_SLEEP_MODE
 */
void cc1000_mode(uint8_t mode);

/** @brief Puts the CC1000 into RX mode.
 */

void cc1000_on();
void cc1000_off();

/** @brief Puts the CC1000 into sleep mode.
 */
void cc1000_sleep();

/** @brief Wakes the CC1000.
 */
void cc1000_wakeup();
void cc1000_reset();

/** @brief Turns on the cc1000 radio.
 */
void cc1000_rssi_on();
void cc1000_rssi_off();

/** @brief Get the current frequency.
 * @return Current frequency
 */
uint8_t cc1000_get_channel();

/** @brief Get the current power level
 * @return Current power level (0-255)
 */
uint8_t cc1000_get_power();

/** @brief Sets mode for this driver. */
uint8_t cc1000_cmode(uint8_t md);

/** @brief Generic io control for this driver. */
uint8_t cc1000_ioctl(uint8_t request, va_list ap);

extern uint16_t cc1000_crc_error_count;
extern uint16_t cc1000_mem_error_count;
extern uint16_t cc1000_sync_error_count;
extern uint16_t cc1000_size_error_count;

/** @brief get the number of FEC errors */
inline uint16_t cc1000_get_fec_errors();


/** @brief state machine to send and receive packet */
inline void cc1000_state_machine();

/** @brief actually start sending the data */
inline void cc1000_start_transmit(comBuf *sendPkt);

void cc1000_fec_on();
void cc1000_fec_off();

/** @brief default initializations for the cc1000 MAC layer */
void cc1000_default_init();

/** @brief Used to initialize error checking or error correction
 * depending on whether or not we're using FEC or CRC
 */
void cc1000_init_ec();

#endif /* _CC1000_H_ */

#endif /* PLATFORM LINUX doesn't need this */
