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
  This file implements RC5 block cipher. It provides RC5 key setup,
  RC5-based encryption/decryption, CBC-mode encryption/decryption,
  and CBC-MAC generation.
*/

#include "mos.h"

#ifdef RC5
#ifndef PLATFORM_LINUX

#include <inttypes.h>
#include <string.h>

#include "CryptoPrimitives.h"
#include "rc5.h"

#define ROTL(x,s) ((x)<<(s) | (x)>>(32-(s)))
#define ROTR(x,s) ((x)>>(s) | (x)<<(32-(s)))

#define P               0xb7e15163L
#define Q               0x9e3779b9L
#define RC5_32_P	0xB7E15163L
#define RC5_32_Q	0x9E3779B9L
#define RC5_32_MASK	0xffffffffL

#define ANDVAL          31
#define rotl32(a, b) fastrol32((a), (b))
#define rotr32(a, b) fastror32((a), (b))

void rc5_key_setup(uint8_t *key, int dummy, rc5key_info *key_info) {
        
  uint32_t *L, l, A, B, *S, k;
  uint8_t  ii, jj, m;
  int8_t   i;
  uint8_t  tmp[16];
 
  S = key_info->vector;
  c2l(key, l);
  L = (uint32_t *) tmp;
  L[0] = l;
  key += 4;
  c2l(key, l);
  L[1] = l;

  key += 4;
  c2l(key, l);
  L[2] = l;
  key += 4;
  c2l(key, l);
  L[3] = l;

  S[0] = RC5_32_P;
  for (i = 1; i < RC5_KEYSPACES; i ++) {
    S[i] = (S[i - 1] + RC5_32_Q);
  }
  ii = jj = 0;
  A = B = 0;
  S = key_info->vector;
  for (i = 3 * ( RC5_KEYSPACES) - 1; i >= 0; i --) {
    k = (*S + A + B) & RC5_32_MASK;
    rotl32((k), (3));
    A = *S = k;
    S ++;
    m = ((char)(A+B)) & 0x1f;
    k = (*L + A + B) & RC5_32_MASK;
    rotl32((k), (m));
    B = *L = k;
    if (++ii >= RC5_KEYSPACES) {
      ii = 0;
      S = key_info->vector;
    }
    jj = (jj + 4) & 0x0f;
    L = (uint32_t *) (&tmp[jj]);
   }
}

void rc5_encrypt(uint32_t *in, uint32_t *out, rc5key_info *key_info)
{
   register uint32_t a, b;
   register uint32_t *s =key_info->vector;
   uint8_t i, tmp;

   a = *in;
   b = *(in+1);

   a += *s++;
   b += *s++;

   for (i = RC5_ROUNDS; i > 0; i --) {
     a ^= b; tmp = b; tmp &= 0x1f; rotl32(a, tmp); a += *s++;
     b ^= a; tmp = a; tmp &= 0x1f; rotl32(b, tmp); b += *s++;
   }

   *out = a;
   *(out+1) = b;
}

void rc5_decrypt(const uint32_t *in, uint32_t *out, rc5key_info *key_info)
{
  uint32_t a, b;
  uint8_t  tmp;
  uint32_t *de_key = key_info->vector;
  int i;

  a = *in;
  b = *(in+1);

  de_key += 26;

  for (i = 0; i < 12; i++) {
    b -= *--de_key;
    tmp = a & 0x1f;
    rotr32(b, tmp);
    b ^= a;

    a -= *--de_key;
    tmp = b & 0x1f;
    rotr32(a, tmp);
    a ^= b;
  }

  b -= *--de_key;
  a -= *--de_key;

  *out = a;
  *(out+1) = b;
}

void block_encrypt(const uint8_t *plt, 
                   uint8_t *enc, 
                   int size, 
                   uint8_t *iv, 
                   rc5key_info *key_info)
{
  int     i, j;
  uint8_t tmp_in[8];
  int     blk_count;
  int     remain_count;

  for (i = 0; i < 8; i ++)
    tmp_in[i] = iv[i];

  blk_count = size / 8;
  remain_count = size % 8;

  for (i = 0; i < blk_count; i ++) {
    for (j = 0; j < 8; j ++) {
      tmp_in[j] ^= plt[(i << 3) + j];
    }
    rc5_encrypt((uint32_t *)tmp_in, (uint32_t *)tmp_in, key_info);
    for (j = 0; j < 8; j ++) {
      enc[(i << 3) + j] = tmp_in[j];
    }
  }

  if (remain_count > 0) {
    for (j = 0; j < remain_count; j ++) {
      tmp_in[j] ^= plt[(blk_count << 3) + j];
    } // as pad 0 ?
    rc5_encrypt((uint32_t *)tmp_in, (uint32_t *)tmp_in, key_info);
    for (j = 0; j < remain_count; j ++) {
      enc[(blk_count << 3) + j] = enc[((blk_count - 1) << 3) + j];
    }
    for (j = 0; j < 8; j ++) {
      enc[((blk_count - 1) << 3) + j] = tmp_in[j];
    }
  }
}

void block_decrypt(const uint8_t *enc, 
                   uint8_t *dec, 
                   int size,
                   uint8_t *iv, 
                   rc5key_info *key_info)
{
  int     i, j;
  uint8_t tmp_iv[8];
  uint8_t tmp_in[8];
  uint8_t tmp_enc[8];
  int     blk_count;
  int     remain_count;

  for (i = 0; i < 8; i ++)
    tmp_iv[i] = iv[i];

  blk_count = size / 8;
  remain_count = size % 8;

  for (i = 0; i < blk_count; i ++) {
    if (i == 0) {
      for (j = 0; j < 8; j ++) {
        tmp_iv[j] = iv[j];
      }
    } else {
      for (j = 0; j < 8; j ++) {
        tmp_iv[j] = tmp_enc[j];
      }
    }
    for (j = 0; j < 8; j ++) {
      tmp_enc[j] = tmp_in[j] = enc[(i << 3) + j];
    }
    rc5_decrypt((uint32_t *)tmp_in, (uint32_t *)tmp_in, key_info);
    for (j = 0; j < 8; j ++) {
      dec[(i << 3) + j] = tmp_iv[j] ^ tmp_in[j]; 
    }
  }

  if (remain_count > 0) {
    for (j = 0; j < remain_count; j ++) {
      tmp_enc[j] = enc[(blk_count << 3) + j];
      dec[(blk_count << 3) + j ] = tmp_in[j] ^ enc[(blk_count << 3) + j];
    }
    for (j = 0; j < remain_count; j ++) {
      tmp_in[j] = tmp_enc[j];
    }
    rc5_decrypt((uint32_t *)tmp_in, (uint32_t *)tmp_in, key_info);
    for (j = 0; j < 8; j ++) {
      dec[((blk_count - 1) << 3) + j] = tmp_in[j] ^ tmp_iv[j];
    }
  } 
}

void block_mac(const uint8_t *plt, 
               int size, 
               uint8_t *mac, 
               int mac_size,
               uint8_t *iv, 
               rc5key_info *key_info)
{
  int     i, j;
  uint8_t tmp_in[8];
  int     blk_count;
  int     remain_count;

  for (i = 0; i < 8; i ++)
    tmp_in[i] = iv[i];

  blk_count = size / 8;
  remain_count = size % 8;

  for (i = 0; i < blk_count; i ++) {
    for (j = 0; j < 8; j ++) {
      tmp_in[j] ^= plt[(i << 3) + j];
    }
    rc5_encrypt((uint32_t *)tmp_in, (uint32_t *)tmp_in, key_info);
  }

  if (remain_count > 0) {
    for (j = 0; j < remain_count; j ++) {
      tmp_in[j] ^= plt[(blk_count << 3) + j];
    } // as pad 0 ?
    rc5_encrypt((uint32_t *)tmp_in, (uint32_t *)tmp_in, key_info);
  }
  if (mac_size <= 8) {
    for (i = 0; i < mac_size; i ++) {
      mac[i] = tmp_in[i];
    }
  }
}

#endif
#endif
