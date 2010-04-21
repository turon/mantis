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

/** @file fec.h
 * @brief Forward Error Corretion routines
 *
 * Note: this is adapted from rscode, http://rscode.sf.net
 */
#include "inttypes.h"
#include "mos.h"

/** @brief Maximum allowed number of parity bytes */
#define MAXIMUM_PARITY_BYTES 64
#define MAXIMUM_IMPLEMENTED_PARITY_BYTES 8

/** @brief Return value if fec_decode detects no errors */
#define FEC_NO_ERRORS 0

/** @brief Return value if fec_decode is able to fix all errors */
#define FEC_CORRECTED_ERRORS 1

/** @brief Return value if fec_decode cannot fix all errors */
#define FEC_UNCORRECTABLE_ERRORS 2

/** @brief Initalize the fec subsystem
 * to the specified number of parity bytes.
 * @param parity_bytes Number of parity bytes to be used in FEC
 */
void fec_init (uint8_t parity_bytes);

/** @brief Return the number of parity bytes the FEC subsystem is currently using.
    @return number of parity bytes
 */
uint8_t fec_get_parity_bytes ();

/** @brief Encode the parity bytes for a portion of memory
 * @param src Place in memory to start computing FEC parity bytes for
 * @param len how much memory (in bytes) to compute the FEC parity bytes for
 * @param parity_ptr Location in memory to store the computed parity bytes.
 */
void fec_encode (uint8_t *src, uint8_t len, uint8_t *parity_ptr);


/** @brief Use FEC to correct errors if possible
 *
 * Note: this function actually changes the passed in buffer
 * if it is possible to fix errors. Also note that for every
 * 2 parity bytes, one byte error can be fixed.
 * @param src location to start Forward Error Correction
 * @param len how much memory (in bytes) to preform FEC on
 * @param fec_ptr location of parity bytes.
 * @return one of FEC_NO_ERRORS, FEC_CORRECTED_ERRORS, FEC_UNCORRECTABLE_ERRORS
 */
uint8_t fec_decode (uint8_t *src, uint8_t len, uint8_t *fec_ptr);
