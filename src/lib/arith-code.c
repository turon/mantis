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

/*
  A module for arithmatic encoding compression 
*/

#include <inttypes.h>
#include "mos.h"

#ifdef PLATFORM_LINUX
#include <stdio.h>
#include <unistd.h>
#endif

#include "arith-code.h"

#define BUFFER_SIZE 256

static int32_t output_mask; // mask applied to output byte if output bit is 1
static int32_t input_bytes_left;
static int32_t input_bits_left;

static char *cur_out_buf;
static char *cur_in_buf;
static char *cur_out_ptr;
static char *cur_in_ptr;

static uint16_t code;  /* The present input code value       */
static uint16_t low;   /* Start of the current code range    */
static uint16_t high;  /* End of the current code range      */
static int32_t arith_code_underflow_bits = 0;  /* underflow bits pending   */

static int16_t storage[BUFFER_SIZE + 2];
static int16_t *totals;

static void arith_code_output_init (char *buf);
static void arith_code_input_init (char *buf);
static void arith_code_output_flush ();
static void output_bit (int16_t bit);
static int16_t input_bit () ;
static void arith_code_encode_init (void);
static void arith_code_decode_init ();
static void arith_code_encode_symbol (arith_code_symbol_t *symbol);
static void arith_code_encode_flush (void);
static int16_t arith_code_current_count (arith_code_symbol_t *symbol);
static void arith_code_remove_symbol (arith_code_symbol_t *symbol);
static void arith_code_model_init (void);
static void arith_code_update_model (int16_t symbol);
static int16_t arith_code_int_to_symbol (int16_t count,
					    arith_code_symbol_t *symbol);
static inline void arith_code_symbol_scale (arith_code_symbol_t *symbol);
static int16_t arith_code_symbol_to_int (int16_t count,
					 arith_code_symbol_t *symbol);
   
/** @brief Init the code output. */
static void arith_code_output_init (char *buf)
{
   cur_out_buf = buf;
   cur_out_ptr = cur_out_buf;
   *cur_out_ptr = 0;
   output_mask = 0x80;
}

/** @brief Init the code input. */
static void arith_code_input_init (char *buf)
{
   cur_in_buf = buf;
   cur_in_ptr = cur_in_buf;
   input_bytes_left = 1;
   input_bits_left = 8;
}

/** @brief Flush the code output buffer. */
static void arith_code_output_flush ()
{
   cur_out_ptr = cur_out_buf;
}

static void output_bit (int16_t bit)
{
   // if there is a bit, or it in
   if (bit)
      *cur_out_ptr |= output_mask;
   // move the output mask either way
   output_mask >>= 1;
   if (output_mask == 0) {
      // once the output mask is 0, reset it
      output_mask = 0x80;
      cur_out_ptr++;
      *cur_out_ptr = 0;
   }
}

/* pull bits out of a buffer one at a time */
static int16_t input_bit ()
{
   if (input_bits_left == 0) {
      // if there are no bits left in the byte, reset things
      cur_in_ptr++;
      input_bytes_left--;
      input_bits_left = 8;
   }
   input_bits_left--;
   // shift out and return the bit
   return ((*cur_in_ptr >> input_bits_left) & 1);
}

/** @brief Initialize the encoding variables. */
static void arith_code_encode_init (void)
{
   low = 0;
   high = 0xffff;
   arith_code_underflow_bits = 0;
}

/** @brief Initialize the decoding variables and read in 16 bits from input. */
static void arith_code_decode_init ()
{
   uint8_t i;

   code = 0;
   for (i = 0; i < 16; i++) {
      code <<= 1;
      code += input_bit ();
   }
   low = 0;
   high = 0xffff;
}


/** @brief Encode a symbol. */
static void arith_code_encode_symbol (arith_code_symbol_t *symbol)
{
   int32_t range;

   // rescale high and low for the new symbol
   range = (int32_t)(high - low) + 1;
   high = low + (int16_t)((range * symbol->high_count) /
			   symbol->scale - 1);
   low = low + (int16_t)((range * symbol->low_count) /
			  symbol->scale);

   // turn out new bits until high and low have stabilized
   for (;;) {
      if ((high & 0x8000) == (low & 0x8000)) {
	 // msdigits match, send to output
	 output_bit (high & 0x8000);
	 while (arith_code_underflow_bits > 0) {
	    output_bit (~high & 0x8000);
	    arith_code_underflow_bits--;
	 }
      } else if ((low & 0x4000) && !(high & 0x4000)) {
	 // numbers are in danger of underflow, msdigits don't match
	 // and the 2nd digits are just one apart
	 arith_code_underflow_bits += 1;
	 low &= 0x3fff;
	 high |= 0x4000;
      } else
	 return;
      low <<= 1;
      high <<= 1;
      high |= 1;
   }
}

/** @brief Output the significant bits left in the high and low registers
 * this is two bits, plus all the underflow bits.
 */
static void arith_code_encode_flush (void)
{
   output_bit (low & 0x4000);
   arith_code_underflow_bits++;
   while (arith_code_underflow_bits > 0) {
      output_bit (~low & 0x4000);
      arith_code_underflow_bits--;
   }
}

/** @brief Figures out which symbol is waiting to be decoded.
 * @param symbol Symbol to be decoded
 * @return Current count
 */
static int16_t arith_code_current_count (arith_code_symbol_t *symbol)
{
   int32_t range;
   int16_t count;

   range = (int32_t)(high - low) + 1;
   count = (int16_t)((((int32_t)(code - low) + 1) * symbol->scale - 1) /
		     range);

   return count;
}

/** @brief After a character is decoded this is called to remove it from
 * the input.
 * @param symbol Symbol decoded 
 */
static void arith_code_remove_symbol (arith_code_symbol_t *symbol)
{
   int32_t range;

   // expand the range to account for removal
   range = (int32_t)(high - low) + 1;
   high = low + (uint16_t)((range * symbol->high_count) /
			   symbol->scale - 1);
   low = low + (uint16_t)((range * symbol->low_count) /
			  symbol->scale);

   for (;;) {
      if ((high & 0x8000) == (low & 0x8000)) {
	 // if msdigits match, bits will be shifted out below
      } else if ((low & 0x4000) == 0x4000 && (high & 0x4000) == 0) {
	 //underflow is looming shift out the 2nd msdigit
	 code ^= 0x4000;
	 low &= 0x3fff;
	 high |= 0x4000;
      } else {
	 // nothing can be shifted out, return
	 return;
      }

      low <<= 1;
      high <<= 1;
      high |= 1;
      code <<= 1;
      code += input_bit ();
   }
}

/** @brief Init the arith code model. */
static void arith_code_model_init (void)
{
   int16_t i;

   totals = storage + 1;
   
   for (i = -1; i <= BUFFER_SIZE; i++)
      totals[i] = i + 1;
}

/** @brief  Update the model, incrementing every count from the high value
 * for the symbol to the total. 
 *
 * If the total has gone up to the max, rescale everything. 
 * @param symbol Current symbol
 */
static void arith_code_update_model (int16_t symbol)
{
   int16_t i;

   for (symbol++; symbol <= BUFFER_SIZE; symbol++)
      totals[symbol]++;
   if (totals[BUFFER_SIZE] == ARITH_CODE_MAXIMUM_SCALE) {
      for (i = 0; i <= BUFFER_SIZE; i++) {
	 totals[i] /= 2;
	 if (totals[i] <= totals[i - 1])
	    totals[i] = totals[i - 1] + 1;
      }
   }
}

/** @brief Convert an into to symbol. 
 * @param count Current count
 * @param symbol New symbol
 */
static int16_t arith_code_int_to_symbol (int16_t count,
				  arith_code_symbol_t *symbol)
{
   symbol->scale = totals[BUFFER_SIZE];
   symbol->low_count = totals[count];
   symbol->high_count = totals[count + 1];
   return 0;
}

/** @brief Get the scale for the current context 
 * @param symbol Current symbol
 */
static inline void arith_code_symbol_scale (arith_code_symbol_t *symbol)
{
   symbol->scale = totals[BUFFER_SIZE];
}

/** @brief Search through the table until we find a symbol straddling the
 * count parameter and return it. 
 *
 * Also set high and low count so the symbol can be properly 
 * removed from the encoded input. 
 * @param count Current count
 * @param symbol Current symbol
 */
static int16_t arith_code_symbol_to_int (int16_t count,
					 arith_code_symbol_t *symbol)
{
   int16_t c;

   for (c = BUFFER_SIZE - 1; count < totals[c]; c--)
      ;
   symbol->high_count = totals[c + 1];
   symbol->low_count = totals[c];
   return c;
}

void arith_code_encode (char *in_buf, uint8_t in_len,
			char *out_buf, uint8_t out_len)
{
   arith_code_symbol_t symbol;
   char c;
   int16_t i;

   arith_code_model_init ();
   arith_code_output_init (out_buf);
   arith_code_encode_init ();
   for (i = 0; ; i++) {
      c = in_buf[i];
      arith_code_int_to_symbol (c, &symbol);
      arith_code_encode_symbol (&symbol);
      if (i >= in_len)
	 break;
      arith_code_update_model (c);
   }
   arith_code_encode_flush ();
   arith_code_output_flush ();
}

void arith_code_decode (char *in_buf, uint8_t in_len,
			char *out_buf, uint8_t out_len)
{
   arith_code_symbol_t symbol;
   char c;
   int16_t i;
   int16_t count;
   
   arith_code_model_init ();
   arith_code_input_init (in_buf);
   arith_code_decode_init ();
   for (i = 0; ; i++) {
      arith_code_symbol_scale (&symbol);
      count = arith_code_current_count (&symbol);
      c = arith_code_symbol_to_int (count, &symbol);
      arith_code_remove_symbol (&symbol);
      if (i >= out_len)
	 break;
      out_buf[i] = c;
      arith_code_update_model (c);
   }
}

