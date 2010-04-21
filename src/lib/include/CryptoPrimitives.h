/*
  This file provides fast bit shift operations for ATMEL chip
*/

/** @file CryptoPrimitives.h
 * @brief Provides fast bit shift operations for the ATMEL chip
 * @author Naveen Sastry
 * @author Modified: Jing Deng
 * @date 12/05/2003
 */

/*									
 * "Copyright (c) 2000-2002 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 * CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 * Authors: Naveen Sastry
 * Date:    10/24/02
 */

// Look at the movw instruction to shave a few more cycles.
// [probably only for the atmel 128's]

/** @brief Performs a leftward rotation on 32 bits of data, 1 bit
 * at a time.
 *
 * (2 + (n * 9)) cycles
 */
#define rol32(a, n) ({		                        \
    	unsigned long num = (unsigned long)(a);         \
    	unsigned char nsh = (unsigned char)(n);         \
	__asm__ __volatile__ (                          \
		"dec %0" "\n\t"			        \
		"brmi L_%=" "\n\t"		        \
	"L1_%=:" "\n\t"                                 \
		"clc" "\n\t"			        \
		"sbrc %D1, 7" "\n\t"		        \
		"sec" "\n\t"			        \
		"rol %A1" "\n\t"		        \
		"rol %B1" "\n\t"		        \
		"rol %C1" "\n\t"		        \
		"rol %D1" "\n\t"		        \
		"dec %0" "\n\t"			        \
		"brpl L1_%=" "\n\t"		        \
	"L_%=:" "\n\t"                                  \
		: "=r" (nsh), "=r" (num)                \
		: "0" (nsh), "1" (num)			\
	);						\
        a = num;                                        \
})

                                                        
/** @brief Performs a rightward rotation on 32 bits of data, 1 bit
 * at a time.
 *
 * (2 + (n * 9)) cycles 
 */
#define ror32(a, n) ({					\
    	unsigned long num = (unsigned long)(a); 	\
    	unsigned char nsh = (unsigned char)(n); 	\
	__asm__ (				        \
		"dec %0" "\n\t"				\
		"brmi L_%=" "\n\t"			\
	"L1_%=:" "\n\t"                                 \
		"clc" "\n\t"				\
		"sbrc %A1, 0" "\n\t"			\
		"sec" "\n\t"				\
		"ror %D1" "\n\t"			\
		"ror %C1" "\n\t"			\
		"ror %B1" "\n\t"			\
		"ror %A1" "\n\t"			\
		"dec %0" "\n\t"				\
		"brpl L1_%=" "\n\t"			\
	"L_%=:" "\n\t"                                  \
		: "=r" (nsh), "=r" (num)                \
		: "0" (nsh), "1" (num)			\
	);						\
        a = num;                                        \
})

/** @brief Copies a 4 byte char buf to a long and moves the ptr
 *
 * 10 cycles
 */
#define c2l(c,l) ({                                    \
  __asm__ (    "mov r30, %A1" "\n\t"                   \
               "mov r31, %B1" "\n\t"                   \
               "ld %A0, Z+" "\n\t"                     \
               "ld %B0, Z+" "\n\t"                     \
               "ld %C0, Z+" "\n\t"                     \
               "ld %D0, Z " "\n\t"                     \
               : "=r" (l)                              \
               : "r" (c)                               \
               : "r30", "r31");                        \
});

/** @brief Copies a long to a 4 byte char buf to a long and
 * doesn't advances the char ptr 
 *
 * 10 cycles
 */
#define l2c(l,c) ({                                    \
  __asm__ volatile (    "mov r30, %A0" "\n\t"          \
               "mov r31, %B0" "\n\t"                   \
               "st Z+, %A1" "\n\t"                     \
               "st Z+, %B1" "\n\t"                     \
               "st Z+, %C1" "\n\t"                     \
               "st Z,  %D1" "\n\t"                     \
               :                                       \
               : "r" (c), "r" (l)                      \
               : "r30", "r31");                        \
});

/** @brief Performs a 1 byte block roll to the left equiv to
 * rol32(a, 8) 
 *
 * 5 cycles
 */
#define brol1(a) ({                                    \
  uint8_t  brol1tmp;                                   \
  __asm__  (   "mov %1, %D0" "\n\t"                    \
               "mov %D0, %C0" "\n\t"                   \
               "mov %C0, %B0" "\n\t"                   \
               "mov %B0, %A0" "\n\t"                   \
               "mov %A0, %1" "\n\t"                    \
               : "=r"(a), "=r" (brol1tmp)              \
               : "0" (a)                               \
               );                                      \
});

/** @brief Performs a 2 byte block roll to the left equiv to
 * rol32(a, 16)
 * 
 * 6 cycles
 */
#define brol2(a) ({                                    \
  uint8_t  brol2tmp;                                   \
  __asm__  (   "mov %1, %A0"   "\n\t"                  \
               "mov %A0, %C0"  "\n\t"                  \
               "mov %C0, %1"   "\n\t"                  \
               "mov %1, %B0"   "\n\t"                  \
               "mov %B0, %D0"  "\n\t"                  \
               "mov %D0, %1"   "\n\t"                  \
               : "=r"(a), "=r" (brol2tmp)              \
               : "0" (a)                               \
               );                                      \
});

/** @brief Performs a 3 byte block roll to the left equiv to
 * rol32(a, 24)
 *
 * 5 cycles
 */
#define brol3(a) ({                                    \
  uint8_t  brol3tmp;                                   \
  __asm__  (   "mov %1, %A0" "\n\t"                    \
               "mov %A0, %B0" "\n\t"                   \
               "mov %B0, %C0" "\n\t"                   \
               "mov %C0, %D0" "\n\t"                   \
               "mov %D0, %1" "\n\t"                    \
               : "=r"(a), "=r" (brol3tmp)              \
               : "0" (a)                               \
               );                                      \
});

#define bror1(a) (brol3(a))
#define bror2(a) (brol2(a))
#define bror3(a) (brol1(a))

/** @brief Fast rol to the left using the above primitives.
 *
 * (switch): 16 cycles
 * (brol.) :  5 cycles
 * sub:    :  1 cycle
 * rol32:  :  2 + (9n), 0 <= n <= 4
 * ===============================
 * BEST    : 16 / 21  cycles (byte boundaries)
 * AVG     : 42       cycles
 * WORST   : 60       cycles
 */
#define fastrol32(a, n) ({                                                 \
  switch ((n)) {                                                           \
  case 0: break;                                                           \
  case 1: case 2: case 3: case 4: case 5: rol32 (a, (n)); break;           \
  case 6: case 7: brol1(a); ror32(a, 8-(n)); break;                        \
  case 8: case 9: case 10: case 11: case 12:  brol1(a); rol32(a, (n)-8 );  \
          break;                                                           \
  case 13: case 14: case 15: case 16: brol2(a); ror32(a, 16-(n)); break;   \
  case 17: case 18: case 19: case 20: brol2(a); rol32(a, (n) -16); break;  \
  case 21: case 22: case 23: case 24: brol3(a); ror32(a, 24-(n)); break;   \
  case 25: case 26: case 27: case 28: brol3(a); rol32(a, (n) -24); break;  \
  case 29: case 30: case 31: ror32(a, 32 - (n));                           \
  }                                                                        \
});

/** @brief Can be improved to eliminate the subtraction */
#define fastror32(a,n) fastrol32(a, (32-n)) 


/** @brief Convert a 2 byte char array to an unsigned short 
 *
 * [assumes MOST significant byte is first]
 */
#define c2sM(c, s)       (s = ((unsigned short)(*((c))))  <<8L ,             \
                          s|= ((unsigned short)(*((c+1)))))

/** @brief Convert a unsigned short to a 2 byte char array
 *
 * [assumes MOST significant byte is first]
 */
#define s2cM(s, c)      (*((c))   = (unsigned short)(((s) >> 8L)&0xff), \
                         *((c+1)) = (unsigned short)(((s)      ) &0xff))
