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

#include <inttypes.h>

#define RC5_ROUNDS      12
#define RC5_KEYSPACES   ((2 * RC5_ROUNDS) + 2)

/** @brief Data structure to save key information.
 */
typedef struct _rc5key_info {
  /** @brief Number of times the bits are shifted */
  int      rounds;
  /** @brief Expanded key */
  uint32_t vector[RC5_KEYSPACES];
} rc5key_info;

/** @brief Key Setup. Expends key.
 * @param key Initial key (16 bytes)
 * @param dummy   Size of the key (always use 16)
 * @param key_info The memory to save expended key 
 */
void rc5_key_setup(uint8_t *key, int dummy, rc5key_info *key_info);
/** @brief Standard RC5 block encryption.
 *
 * Encrypting 64-bit of data with expended key
 * @param in input plaintext. Pointer to 2 32-bit words
 * @param out output ciphertext. Pointer to 2 32-bit words
 * @param key_info expended key. Pointer to rc5key_info
 */
void rc5_encrypt(uint32_t *in, uint32_t *out, rc5key_info *key_info);
/** @brief Standard RC5 block decryption.
 *  
 * Decrypting 64-bit of data with expended key
 * @param in input ciphertext. Pointer to 2 32-bit words
 * @param out output decrypted text. Pointer to 2 32-bit words
 * @param key_info expended key. Pointer to rc5key_info
 */
void rc5_decrypt(const uint32_t *in, uint32_t *out, rc5key_info *key_info);
/** @brief CBC-mode block encryption using rc5_encrypt.
 * @param plt input plaintext. Pointer to a >= 8 bytes string
 * @param enc output ciphertext, Pointer to a >= 8 bytes string
 * @param size the size of input/output data, counted by bytes
 * @param iv initial vector. Pointer to an 8-bytes string
 * @param key_info expended key. Pointer to rc5key_info
 */
void block_encrypt(const uint8_t *plt, uint8_t *enc, int size,
                   uint8_t *iv, rc5key_info *key_info);
/** @brief CBC-mode block decryption using rc5_encrypt.
 * @param enc input encrypted text. Pointer to a >= 8 bytes string
 * @param dec output decrypted text, Pointer to a >= 8 bytes string
 * @param size the size of input/output data, counted by bytes
 * @param iv initial vector. Pointer to an 8-bytes string
 * @param key_info expended key. Pointer to rc5key_info
 */
void block_decrypt(const uint8_t *enc, uint8_t *dec, int size,
		   uint8_t *iv, rc5key_info *key_info);
/** @brief CBC-mode message authentication code generation.
 * @param plt input text. Pointer to a >= 8 bytes string
 * @param size the size of input data, counted by bytes
 * @param mac output MAC, Pointer to a 3-bytes string
 * @param mac_size size of MAC, should be <= 8, otherwise, mac won't contain MAC
 * @param iv initial vector. Pointer to an 8-bytes string
 * @param key_info expended key. Pointer to rc5key_info
 */
void block_mac(const uint8_t *plt, int size, uint8_t *mac, int mac_size,
                      uint8_t *iv, rc5key_info *key_info);

