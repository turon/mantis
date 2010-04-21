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

/** @file arith-code.h
 * @brief Symbol coding.
 *
 * largely based on: http://dogma.net/markn/articles/arith/
 */

/** @brief Max allowed frequency count. */
#define ARITH_CODE_MAXIMUM_SCALE   16383  
/** @brief Escape symbol. */
#define ARITH_CODE_ESCAPE          256  
/** @brief Output stream empty symbol. */
#define ARITH_CODE_DONE            -1    
/** @brief Symbol to flush the model. */
#define ARITH_CODE_FLUSH           -2   

/** @brief A symbol can be an int or a pair of counts on a scale.
   Here is the structure for the latter. */
typedef struct {
  /** @brief Overflow bits for start of code range.*/
   unsigned short int low_count; 
  /** @brief Overflow bits for end of code range. */
   unsigned short int high_count; 
  /** @brief Scale used */
   unsigned short int scale;
} arith_code_symbol_t;

/** @brief Encode a symbol
 * @param in_buf Input buffer
 * @param in_len Input length
 * @param out_buf Output buffer
 * @param out_len Output length
 */
void arith_code_encode (char *in_buf, uint8_t in_len,
			char *out_buf, uint8_t out_len);
/** @brief Decode a symbol
 * @param in_buf Input buffer
 * @param in_len Input length
 * @param out_buf Output buffer
 * @param out_len Output length
 */
void arith_code_decode (char *in_buf, uint8_t in_len,
			char *out_buf, uint8_t out_len);
