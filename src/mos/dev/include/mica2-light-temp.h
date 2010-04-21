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


/** @file mica2-light-temp.h
 * @brief Integrated light and temp sensor driver for the mica2.
 * 
 * This driver registers with the dev layer as two differnt devices, but they are 
 * intertwined in hardware so we implement it in one file.
 * @author Brian Shucker
 * @author Based on light.c and temp.c by Charles Gruenwald III
 * @date 04/14/2004
 */

//#define MICA2_PROTO 1
#include "plat_dep.h"

#if defined (PLATFORM_MICA2) || defined (PLATFORM_MICAZ) || defined(PLATFORM_AVRDEV) || defined(PLATFORM_MICA2DOT)


#ifdef MICA2_PROTO
#define LIGHT_PORT_DIRE DDRC
#define TEMP_PORT_DIRE DDRC
#define LIGHT_PORT PORTC
#define TEMP_PORT PORTC
#define LIGHT_PIN_MASK (1 << 1)
#define TEMP_PIN_MASK (1 << 2)
#define LIGHT_ADC_CHANNEL AVR_ADC_CH_6
#define TEMP_ADC_CHANNEL AVR_ADC_CH_5


/** @brief temperature conversion table*/
static int8_t temp_vals [] ARCH_PROGMEM = {
   -82, -66, -56, -49, -44, -39, -35, -31, -28, -25,
   -22, -20, -18, -15, -13, -11, -10,  -8,  -6,  -4,
   -3,   -1,   0,   0,   2,   3,   4,   5,   7,   8,
   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,
   19,   20,  21,  22,  23,  23,  24,  25,  26,  27,
   28,   28,  29,  30,  31,  31,  32,  33,  34,  34,
   35,   36,  37,  37,  38,  39,  39,  40,  41,  41,
   42,   43,  43,  44,  45,  45,  46,  47,  47,  48,
   48,   49,  50,  50,  51,  52,  52,  53,  53,  54,
   55,   55,  56,  56,  57,  58,  58,  59,  59,  60,
   61,   61,  62,  62,  63,  64,  64,  65,  65,  66,
   66,   67,  68,  68,  69,  69,  70,  71,  71,  72,
   72,   73,  74,  74,  75,  75,  76,  76,  77,  78, 
   78,   79,  79,  80,  81,  81,  82,  82,  83,  84,
   84,   85,  86,  86,  87,  87,  88,  89,  89,  90,
   91,   91,  92,  93,  93,  94,  95,  95,  96,  97,
   97,   98,  99,  99, 100, 101, 101, 102, 103, 104,
   104, 105, 106, 106, 107, 108, 109, 110, 110, 111,
   112, 113, 114, 114, 115, 116, 117, 118, 119, 119, 
   120, 121, 122, 123, 124, 125, 126, 127, 127, 127,
   127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
   127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
   127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
   127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
   127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
   127, 127, 127, 127, 127};

/* truncate the table so it will fit in an uint8_t, this is the full data */
/*
  128, 129, 130, 
  131, 132, 133, 134, 135, 137, 138, 139, 140, 141, 143, 
  144, 145, 147, 148, 149, 151, 152, 154, 155, 157, 159, 
  161, 162, 164, 166, 168, 170, 172, 174, 177, 179, 182, 
  184, 187, 190, 193, 196, 200, 204, 208, 212, 217, 222, 
  228, 235, 242, 251, 261, 273, 288, 309, 340, 398};
*/


#else
#define LIGHT_PORT_DIRE DDRE
#define TEMP_PORT_DIRE DDRE
#define LIGHT_PORT PORTE
#define TEMP_PORT PORTE
#define LIGHT_PIN_MASK 0x20
#define TEMP_PIN_MASK 0x40
#define LIGHT_ADC_CHANNEL AVR_ADC_CH_1
#define TEMP_ADC_CHANNEL AVR_ADC_CH_1



/*temperature conversion table*/
/* TODO: put this in the right place in memory */
static int8_t temp_vals [] ARCH_PROGMEM = {
   -82, -66, -56, -49, -44, -39, -35, -31, -28, -25,
   -22, -20, -18, -15, -13, -11, -10,  -8,  -6,  -4,
   -3,   -1,   0,   0,   2,   3,   4,   5,   7,   8,
   9,   10,  11,  12,  13,  14,  15,  16,  17,  18,
   19,   20,  21,  22,  23,  23,  24,  25,  26,  27,
   28,   28,  29,  30,  31,  31,  32,  33,  34,  34,
   35,   36,  37,  37,  38,  39,  39,  40,  41,  41,
   42,   43,  43,  44,  45,  45,  46,  47,  47,  48,
   48,   49,  50,  50,  51,  52,  52,  53,  53,  54,
   55,   55,  56,  56,  57,  58,  58,  59,  59,  60,
   61,   61,  62,  62,  63,  64,  64,  65,  65,  66,
   66,   67,  68,  68,  69,  69,  70,  71,  71,  72,
   72,   73,  74,  74,  75,  75,  76,  76,  77,  78, 
   78,   79,  79,  80,  81,  81,  82,  82,  83,  84,
   84,   85,  86,  86,  87,  87,  88,  89,  89,  90,
   91,   91,  92,  93,  93,  94,  95,  95,  96,  97,
   97,   98,  99,  99, 100, 101, 101, 102, 103, 104,
   104, 105, 106, 106, 107, 108, 109, 110, 110, 111,
   112, 113, 114, 114, 115, 116, 117, 118, 119, 119, 
   120, 121, 122, 123, 124, 125, 126, 127, 127, 127,
   127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
   127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
   127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
   127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
   127, 127, 127, 127, 127, 127, 127, 127, 127, 127,
   127, 127, 127, 127, 127};

/* truncate the table so it will fit in an uint8_t, this is the full data */
/*
  128, 129, 130, 
  131, 132, 133, 134, 135, 137, 138, 139, 140, 141, 143, 
  144, 145, 147, 148, 149, 151, 152, 154, 155, 157, 159, 
  161, 162, 164, 166, 168, 170, 172, 174, 177, 179, 182, 
  184, 187, 190, 193, 196, 200, 204, 208, 212, 217, 222, 
  228, 235, 242, 251, 261, 273, 288, 309, 340, 398};
*/


#endif

#define select_temp()						\
   LIGHT_PORT_DIRE &= ~LIGHT_PIN_MASK;				\
   LIGHT_PORT &= ~LIGHT_PIN_MASK;				\
   TEMP_PORT_DIRE |= TEMP_PIN_MASK;				\
   TEMP_PORT &= ~TEMP_PIN_MASK;					\
   TEMP_PORT |= TEMP_PIN_MASK;

#define select_light()							\
   TEMP_PORT_DIRE &= ~TEMP_PIN_MASK;					\
   TEMP_PORT &= ~TEMP_PIN_MASK;		\
   LIGHT_PORT_DIRE |= LIGHT_PIN_MASK;					\
   LIGHT_PORT &= ~LIGHT_PIN_MASK; \
   LIGHT_PORT |= LIGHT_PIN_MASK; 

#define select_off()						     \
   LIGHT_PORT &= ~LIGHT_PIN_MASK; \
   LIGHT_PORT_DIRE &= ~LIGHT_PIN_MASK;				     \
   TEMP_PORT &= ~TEMP_PIN_MASK; \
   TEMP_PORT_DIRE &= ~TEMP_PIN_MASK;				     \

#elif defined (PLATFORM_MICA2DOT)

#define LIGHT_PORT_DIRE DDRC
#define LIGHT_PORT PORTC
#define LIGHT_PIN_MASK (1 << 0)
#define LIGHT_ADC_CHANNEL AVR_ADC_CH_7

#define TEMP_PORT_DIRE DDRC
#define TEMP_PORT PORTC
#define TEMP_HI (1 << 7)
#define TEMP_LO (1 << 6)

#define select_temp() {						\
      TEMP_PORT_DIRE |= (TEMP_HI | TEMP_LO);			\
      /* turn on thermistor (int2) */				\
      TEMP_PORT |= (TEMP_HI);					\
      TEMP_PORT &= ~(TEMP_LO);					\
      LIGHT_PORT_DIRE &= ~LIGHT_PIN_MASK;			\
      /* turn off photoresistor (int1) */			\
      LIGHT_PORT &= ~LIGHT_PIN_MASK;				\
   }


#define select_light() {						\
      LIGHT_PORT_DIRE &= ~LIGHT_PIN_MASK;				\
      LIGHT_PORT |= LIGHT_PIN_MASK; /* turn on photo-resistor (int1)*/	\
      LIGHT_PORT_DIRE |= LIGHT_PIN_MASK;				\
      TEMP_PORT &= ~(TEMP_HI);						\
      TEMP_PORT |= (TEMP_LO);						\
      TEMP_PORT_DIRE &= ~(TEMP_HI | TEMP_LO);				\
   }


#define select_off() {							\
      LIGHT_PORT_DIRE |= LIGHT_PIN_MASK;				\
      LIGHT_PORT &= ~LIGHT_PIN_MASK;  /*turn off photoresistor (int1)*/	\
      TEMP_PORT &= ~(TEMP_HI);						\
      TEMP_PORT |= (TEMP_LO);						\
      TEMP_PORT_DIRE &= ~(TEMP_HI | TEMP_LO);				\
   }

#endif


/*functions*/

void mica2_light_temp_init(void);
