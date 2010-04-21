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
  A module providing forward error correction (reed-solomon)
*/

/** @file fec.c
 * @brief Forward Error Correction 
 */

#include <inttypes.h>
#include "mos.h"

#ifdef PLATFORM_LINUX
#include <stdio.h>
#include <unistd.h>
#endif

#if defined(PLATFORM_LINUX) || defined(ARCH_AVR)

#include "fec_table.h"
#include "printf.h"
#include "led.h"
#include "fec.h"

static uint8_t num_parity_bytes;
static uint8_t global_parity_bytes[MAXIMUM_IMPLEMENTED_PARITY_BYTES];
static uint8_t galois_syn[MAXIMUM_IMPLEMENTED_PARITY_BYTES];
static uint8_t num_errors = 0;
static uint8_t lambda[MAXIMUM_IMPLEMENTED_PARITY_BYTES * 2];
static uint8_t omega[MAXIMUM_IMPLEMENTED_PARITY_BYTES * 2];
static uint8_t error_locations[MAXIMUM_IMPLEMENTED_PARITY_BYTES * 2];

static inline uint8_t gmult (uint8_t a, uint8_t b);
static inline void zero_poly (uint8_t *poly);
static inline void mult_polys (uint8_t *dst, uint8_t *p1, uint8_t *p2);
static inline void copy_poly (uint8_t *dst, uint8_t *src);

static uint8_t tmp1[MAXIMUM_IMPLEMENTED_PARITY_BYTES * 4];
static uint8_t product[MAXIMUM_IMPLEMENTED_PARITY_BYTES * 4];
static uint8_t LFSR[MAXIMUM_IMPLEMENTED_PARITY_BYTES + 1];
static uint8_t gmult_i, gmult_j;
static uint8_t syn_i, syn_nz = 0;
static uint8_t decode_i, decode_j, decode_sum;
static uint8_t correct_r, correct_i, correct_j, correct_err;
static uint8_t correct_parity_2;
static uint8_t correct_num, correct_denom;
static uint16_t root_r;
static uint8_t root_sum, root_k;
static uint8_t *berk_psi2;
static uint8_t *berk_D;
static uint8_t *berk_gamma;
static uint8_t berk_n, berk_L, berk_L2, berk_d, berk_i;
static int8_t berk_k;
static uint8_t *berk_psi;
static uint8_t berk_parity_2;
static uint8_t discrep_i;
static uint8_t discrep_sum;
static uint8_t mul_z_i;
static uint8_t modified_i;
static uint8_t zero_i, copy_i;
static int8_t mult_i, mult_j;
static uint8_t mult_parity_2, mult_parity_4;

#ifdef PLATFORM_LINUX
static void galois_init (void)
{
   uint16_t i, z;
   uint8_t pinit, p1, p2, p3, p4, p5, p6, p7, p8;

   pinit = p2 = p3 = p4 = p5 = p6 = p7 = p8 = 0;
   p1 = 1;
   
   galois_exp[0] = 1;
   galois_exp[255] = 1;
   galois_log[0] = 0;

   for (i = 1; i < 256; i++) {
      pinit = p8;
      p8 = p7;
      p7 = p6;
      p6 = p5;
      p5 = p4 ^ pinit;
      p4 = p3 ^ pinit;
      p3 = p2 ^ pinit;
      p2 = p1;
      p1 = pinit;
      galois_exp[i] = p1 + p2 * 2 + p3 * 4 + p4 * 8 + p5 * 16 + p6 * 32
	 + p7 * 64 + p8 * 128;
      galois_exp[i + 255] = galois_exp[i];
   }

   for (i = 1; i < 256; i++) {
      for (z = 0; z < 256; z++) {
	 if (galois_exp[z] == i) {
	    galois_log[i] = z;
	    break;
	 }
      }
   }   
}

static void compute_generator_poly (void)
{
   uint8_t i;
   uint8_t tp[256], tp1[256];

   zero_poly (tp1);
   tp1[0] = 1;

   for (i = 1; i <= num_parity_bytes; i++) {
      zero_poly (tp);
#ifdef PLATFORM_MICA_ANY
      tp[0] = pgm_read_byte (&galois_exp[i]);
#else
      tp[0] = galois_exp[i];
#endif
      tp[1] = 1;

      mult_polys (galois_poly, tp, tp1);
      copy_poly (tp1, galois_poly);
   }
}
#endif

static inline void zero_poly (uint8_t *poly)
{
   for (zero_i = 0; zero_i < num_parity_bytes * 2; zero_i++) {
      poly[zero_i] = 0;
   }
}

static inline void copy_poly (uint8_t *dst, uint8_t *src)
{
   for (copy_i = 0; copy_i < num_parity_bytes * 2; copy_i++) {
      dst[copy_i] = src[copy_i];
   }
}

static inline void mult_polys (uint8_t *dst, uint8_t *p1, uint8_t *p2)
{
   mult_parity_2 = num_parity_bytes * 2;
   mult_parity_4 = num_parity_bytes * 4;

   for (mult_i = 0; mult_i < mult_parity_4; mult_i++)
      dst[mult_i] = 0;

   for (mult_i = 0; mult_i < mult_parity_2; mult_i++) {
      for (mult_j = mult_parity_2; mult_j < mult_parity_4; mult_j++) {
	 tmp1[mult_j] = 0;
      }
      for (mult_j = 0; mult_j < mult_parity_2; mult_j++) {
	 tmp1[mult_j] = gmult (p2[mult_j], p1[mult_i]);
      }
      for (mult_j = (mult_parity_4) - 1; mult_j >= mult_i; mult_j--) {
	 tmp1[mult_j] = tmp1[mult_j - mult_i];
      }
      for (mult_j = 0; mult_j < mult_i; mult_j++) {
	 tmp1[mult_j] = 0;
      }
      for (mult_j = 0; mult_j < mult_parity_4; mult_j++) {
	 dst[mult_j] ^= tmp1[mult_j];
      }
   }
}

/*
static void print_galois (void) 
{
   uint16_t i;
#ifdef PLATFORM_MICA_ANY
   for (i = 0; i < 512; i++) {
      printf ("gexp[%d]: %d\n", i, pgm_read_byte (&galois_exp[i]));
   }
   for (i = 0; i < 256; i++) {
      printf ("glog[%d]: %d\n", i, pgm_read_byte (&galois_log[i]));
   }
   for (i = 0; i < sizeof (galois_poly); i++) {
      printf ("gpoly[%d]: %d\n", i, pgm_read_byte (&galois_poly[i]));
   }
#else
   for (i = 0; i < 512; i++) {
      printf ("gexp[%d]: %d\n", i, galois_exp[i]);
   }
   for (i = 0; i < 256; i++) {
      printf ("glog[%d]: %d\n", i, galois_log[i]);
   }
   for (i = 0; i < sizeof (galois_poly); i++) {
      printf ("gpoly[%d]: %d\n", i, galois_poly[i]);
   }
#endif
}
*/

void fec_init (uint8_t parity_bytes)
{
   if (parity_bytes > MAXIMUM_PARITY_BYTES)
      parity_bytes = MAXIMUM_PARITY_BYTES;
   num_parity_bytes = parity_bytes;
   
#ifdef PLATFORM_LINUX
   galois_init ();
   compute_generator_poly ();
#else
   switch (parity_bytes) {
   case 2:
      galois_poly = galois_poly_2;
      break;
   case 4:
      galois_poly = galois_poly_4;
      break;
   case 8:
      galois_poly = galois_poly_8;
      break;
   default:
      printf ("Error: Unimplemented galois_poly table\n");
      break;
   }
#endif

   //print_galois ();
}

static void build_codeword (uint8_t *src, uint8_t len, uint8_t *parity_ptr)
{
   uint8_t i;

   for (i = 0; i < num_parity_bytes; i++) {
      parity_ptr[i] = global_parity_bytes[num_parity_bytes - 1 - i];
   }
}

// logarithmic multiplication
static inline uint8_t gmult (uint8_t a, uint8_t b)
{
   if (a == 0 || b == 0)
      return 0;
#ifdef PLATFORM_MICA_ANY
   gmult_i = pgm_read_byte (&galois_log[a]);
   gmult_j = pgm_read_byte (&galois_log[b]);
   return pgm_read_byte (&galois_exp[(uint16_t)(gmult_i + gmult_j)]);
#else
   gmult_i = galois_log[a];
   gmult_j = galois_log[b];
   return galois_exp[(uint16_t)(gmult_i + gmult_j)];
#endif
}

void fec_encode (uint8_t *src, uint8_t len, uint8_t *parity_ptr)
{
   uint8_t i, j, byte;

   for (i = 0; i < num_parity_bytes + 1; i++)
      LFSR[i] = 0;

   for (i = 0; i < len; i++) {
      byte = src[i] ^ LFSR[num_parity_bytes - 1];
      for (j = num_parity_bytes - 1; j > 0; j--) {
#ifdef PLATFORM_MICA_ANY
	 LFSR[j] = LFSR[j - 1] ^ gmult (pgm_read_byte (&galois_poly[j]), byte);
#else
	 LFSR[j] = LFSR[j - 1] ^ gmult (galois_poly[j], byte);
#endif
      }
#ifdef PLATFORM_MICA_ANY
      LFSR[0] = gmult (pgm_read_byte (&galois_poly[0]), byte);
#else
      LFSR[0] = gmult (galois_poly[0], byte);
#endif
   }

   for (i = 0; i < num_parity_bytes; i++)
      global_parity_bytes[i] = LFSR[i];

   build_codeword (src, len, parity_ptr);
}

static inline uint8_t check_syn (void)
{
   syn_nz = 0;
   for (syn_i = 0; syn_i < num_parity_bytes; syn_i++) {
      if (galois_syn[syn_i] != 0) {
	 syn_nz = 1;
      }
   }
   return syn_nz;
}

static void init_gamma (uint8_t *gamma)
{
   zero_poly (gamma);
   zero_poly (tmp1);
   gamma[0] = 1;
}

static inline uint8_t ginv (uint8_t elt)
{
#ifdef PLATFORM_MICA_ANY
   return pgm_read_byte (&galois_exp[255 - pgm_read_byte (&galois_log[elt])]);
#else
   return galois_exp[255 - galois_log[elt]];
#endif
}

static inline void mul_z_poly (uint8_t *src)
{
   for (mul_z_i = (num_parity_bytes * 2) - 1; mul_z_i > 0; mul_z_i--) {
      src[mul_z_i] = src[mul_z_i - 1];
   }
   src[0] = 0;
}

static inline uint8_t compute_discrepancy (uint8_t *lambda, uint8_t *S,
					   uint8_t L, uint8_t n)
{
   discrep_sum = 0;
   for (discrep_i = 0; discrep_i <= L; discrep_i++) {
      discrep_sum ^= gmult (lambda[discrep_i], S[n - discrep_i]);
   }
   return discrep_sum;
}

static inline void compute_modified_omega (void)
{
   mult_polys (product, lambda, galois_syn);
   zero_poly (omega);
   for (modified_i = 0; modified_i < num_parity_bytes; modified_i++) {
      omega[modified_i] = product[modified_i];
   }
}

static void modified_berlekamp_massey (void)
{
   berk_parity_2 = num_parity_bytes * 2;
   berk_psi = tmp1;
   berk_psi2 = tmp1 + berk_parity_2;
   berk_D = product;
   berk_gamma = product + berk_parity_2;

   init_gamma (berk_gamma);
   copy_poly (berk_D, berk_gamma);
   mul_z_poly (berk_D);

   copy_poly (berk_psi, berk_gamma);
   berk_k = -1;
   berk_L = 0;

   for (berk_n = 0; berk_n < num_parity_bytes; berk_n++) {
      berk_d = compute_discrepancy (berk_psi, galois_syn, berk_L, berk_n);
      if (berk_d != 0) {
	 for (berk_i = 0; berk_i < berk_parity_2; berk_i++) {
	    berk_psi2[berk_i] = berk_psi[berk_i] ^ gmult (berk_d, berk_D[berk_i]);
	 }

	 if (berk_L < (berk_n - berk_k)) {
	    berk_L2 = berk_n - berk_k;
	    berk_k = berk_n - berk_L;
	    for (berk_i = 0; berk_i < berk_parity_2; berk_i++) {
	       berk_D[berk_i] = gmult (berk_psi[berk_i], ginv (berk_d));
	    }
	    berk_L = berk_L2;
	 }

	 for (berk_i = 0; berk_i < berk_parity_2; berk_i++) {
	    berk_psi[berk_i] = berk_psi2[berk_i];
	 }
      }
      mul_z_poly (berk_D);
   }

   for (berk_i = 0; berk_i < berk_parity_2; berk_i++) {
      lambda[berk_i] = berk_psi[berk_i];
   }
   compute_modified_omega ();
}

static void find_roots (uint8_t len)
{
   num_errors = 0;

   for (root_r = 1; root_r < 256; root_r++) {
      root_sum = 0;
      for (root_k = 0; root_k < num_parity_bytes + 1; root_k++) {
#ifdef PLATFORM_MICA_ANY
	 root_sum ^= gmult (pgm_read_byte (&galois_exp[(root_k * root_r) % 255]),
		       lambda[root_k]);
#else
	 root_sum ^= gmult (galois_exp[(root_k * root_r) % 255],
			    lambda[root_k]);
#endif
      }
      if (root_sum == 0) {
	 error_locations[num_errors] = (255 - root_r);
	 if (++num_errors >= 8) {
	    printf ("Ran out of error location room\n");
	    return;
	 }
	 //printf ("Root found at r = %d, (255 - r) = %d\n", r, 255 - r);
      }
   }
}

static inline uint8_t correct_errors (uint8_t *src, uint8_t len)
{
   correct_parity_2 = num_parity_bytes * 2;
   
   modified_berlekamp_massey ();
   find_roots (len);

   if ((num_errors <= num_parity_bytes) && num_errors > 0) {
      for (correct_r = 0; correct_r < num_errors; correct_r++) {
	 if (error_locations[correct_r] >= len) {
	    //printf ("Error location %d outside codeword length %d\n",
	    //	    i, len);
	    return FEC_UNCORRECTABLE_ERRORS;
	 }
      }
      for (correct_r = 0; correct_r < num_errors; correct_r++) {
	 correct_i = error_locations[correct_r];

	 correct_num = 0;
	 for (correct_j = 0; correct_j < correct_parity_2; correct_j++) {
#ifdef PLATFORM_MICA_ANY
	    correct_num ^= gmult (omega[correct_j],
			  pgm_read_byte (&galois_exp[((255 - correct_i) *
						      correct_j) % 255]));
#else
	    correct_num ^= gmult (omega[correct_j], galois_exp[((255 - correct_i) *
							correct_j) % 255]);
#endif
	 }
	 correct_denom = 0;
	 for (correct_j = 1; correct_j < correct_parity_2; correct_j += 2) {
#ifdef PLATFORM_MICA_ANY
	    correct_denom ^= gmult (lambda[correct_j],
			    pgm_read_byte (&galois_exp[((255 - correct_i) *
							(correct_j - 1)) % 255]));
#else
	    correct_denom ^= gmult (lambda[correct_j],
			    galois_exp[((255 - correct_i) * (correct_j - 1)) % 255]);
#endif
	 }
	 correct_err = gmult (correct_num, ginv (correct_denom));
	 //printf ("Error magnitude %x at loc %d\n", err, len - i);

	 src[len - correct_i - 1] ^= correct_err;
      }
      return FEC_CORRECTED_ERRORS;
   } else {
      //printf ("Uncorrectable codeword\n");
      return FEC_UNCORRECTABLE_ERRORS;
   }
}

uint8_t fec_decode (uint8_t *src, uint8_t len, uint8_t *fec_ptr)
{
   for (decode_j = 0; decode_j < num_parity_bytes; decode_j++) {
      decode_sum = 0;
      for (decode_i = 0; decode_i < len; decode_i++) {
#ifdef PLATFORM_MICA_ANY
	 decode_sum = src[decode_i] ^
	    gmult (pgm_read_byte (&galois_exp[decode_j + 1]), decode_sum);
#else
	 decode_sum = src[decode_i] ^
	    gmult (galois_exp[decode_j + 1], decode_sum);
#endif
      }
      for (decode_i = 0; decode_i < num_parity_bytes; decode_i++) {
#ifdef PLATFORM_MICA_ANY
	 decode_sum = fec_ptr[decode_i] ^
	    gmult (pgm_read_byte (&galois_exp[decode_j + 1]), decode_sum);
#else
	 decode_sum = fec_ptr[decode_i] ^
	    gmult (galois_exp[decode_j + 1], decode_sum);
#endif
      }
      galois_syn[decode_j] = decode_sum;
   }

   if (check_syn ()) {
      return correct_errors (src, len + num_parity_bytes);
   }

   return FEC_NO_ERRORS;
}

uint8_t fec_get_parity_bytes (void)
{
   return num_parity_bytes;
}

#endif
