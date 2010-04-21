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

/** @file bitops.h
 * @brief Operations which specify how to set ports to input
 * or output. Additionally provides macros for setting and clearing
 * these registers.
 */

#ifdef PLATFORM_TELOSB
#define PORT_DIR_(port) P##port##DIR
#define PORT_OUT_(port) P##port##OUT
#define PORT_IN_(port) P##port##IN
#else
#define PORT_DIR_(port) DDR##port
#define PORT_OUT_(port) PORT##port
#define PORT_IN_(port) PIN##port
#endif

#define PORT_DIR(port) PORT_DIR_(port)
#define PORT_OUT(port) PORT_OUT_(port)
#define PORT_IN(port) PORT_IN_(port)

/* these macros set bits in a given byte */
#define MASK_1(byte, bit) (byte |= (1 << bit))
#define MASK_2(byte, bit1, bit2) (byte |= (1 << bit1) | (1 << bit2))
#define MASK_3(byte, bit1, bit2, bit3)			\
   (byte |= (1 << bit1) | (1 << bit2) | (1 << bit3))
#define MASK_4(byte, bit1, bit2, bit3, bit4)				\
   (byte |= (1 << bit1) | (1 << bit2) | (1 << bit3) | (1 << bit4))
#define MASK_5(byte, bit1, bit2, bit3, bit4, bit5)			\
   (byte |= (1 << bit1) | (1 << bit2) | (1 << bit3) | (1 << bit4) |	\
    (1 << bit5))
#define MASK_6(byte, bit1, bit2, bit3, bit4, bit5, bit6)		\
   (byte |= (1 << bit1) | (1 << bit2) | (1 << bit3) | (1 << bit4) |	\
    (1 << bit5) | (1 << bit6))
#define MASK_7(byte, bit1, bit2, bit3, bit4, bit5, bit6, bit7)		\
   (byte |= (1 << bit1) | (1 << bit2) | (1 << bit3) | (1 << bit4) |	\
    (1 << bit5) | (1 << bit6) | (1 << bit7))

/* these macros clear bits in a given byte */
#define UNMASK_1(byte, bit) (byte &= ~(1 << bit))
#define UNMASK_2(byte, bit1, bit2) (byte &= ~((1 << bit1) |	\
					      (1 << bit2)))
#define UNMASK_3(byte, bit1, bit2, bit3)		\
   (byte &= ~((1 << bit1) | (1 << bit2) | (1 << bit3)))
#define UNMASK_4(byte, bit1, bit2, bit3, bit4)				\
   (byte &= ~((1 << bit1) | (1 << bit2) | (1 << bit3) | (1 << bit4)))
#define UNMASK_5(byte, bit1, bit2, bit3, bit4, bit5)			\
   (byte &= ~((1 << bit1) | (1 << bit2) | (1 << bit3) | (1 << bit4) |	\
	      (1 << bit5)))
#define UNMASK_6(byte, bit1, bit2, bit3, bit4, bit5, bit6)		\
   (byte &= ~((1 << bit1) | (1 << bit2) | (1 << bit3) | (1 << bit4) |	\
	      (1 << bit5) | (1 << bit6)))
#define UNMASK_7(byte, bit1, bit2, bit3, bit4, bit5, bit6, bit7)	\
   (byte &= ~((1 << bit1) | (1 << bit2) | (1 << bit3) | (1 << bit4) |	\
	      (1 << bit5) | (1 << bit6) | (1 << bit7)))

#define SET_DIR_1_(port, pin) MASK_1 (PORT_DIR(port), pin)
#define SET_DIR_1(x) SET_DIR_1_ (x)
#define SET_DIR_2_(port1, pin1, port2, pin2)	\
   MASK_2 (PORT_DIR(port1), pin1, pin2)
#define SET_DIR_2(x, y) SET_DIR_2_ (x, y)
#define SET_DIR_3_(port1, pin1, port2, pin2, port3, pin3)	\
   MASK_3 (PORT_DIR(port1), pin1, pin2, pin3)
#define SET_DIR_3(x, y, z) SET_DIR_3_ (x, y, z)
#define SET_DIR_4_(port1, pin1, port2, pin2, port3, pin3, port4, pin4)	\
   MASK_4 (PORT_DIR(port1), pin1, pin2, pin3, pin4)
#define SET_DIR_4(w, x, y, z) SET_DIR_4_ (w, x, y, z)
#define SET_DIR_5_(port1, pin1, port2, pin2, port3, pin3, port4, pin4,	\
		   port5, pin5)						\
   MASK_5 (PORT_DIR(port1), pin1, pin2, pin3, pin4, pin5)
#define SET_DIR_5(v, w, x, y, z) SET_DIR_5_ (v, w, x, y, z)
#define SET_DIR_6_(port1, pin1, port2, pin2, port3, pin3, port4, pin4,	\
		   port5, pin5, port6, pin6)				\
   MASK_6 (PORT_DIR(port1), pin1, pin2, pin3, pin4, pin5, pin6)
#define SET_DIR_6(u, v, w, x, y, z) SET_DIR_6_ (u, v, w, x, y, z)
#define SET_DIR_7_(port1, pin1, port2, pin2, port3, pin3, port4, pin4,	\
		   port5, pin5, port6, pin6)				\
   MASK_7 (PORT_DIR(port1), pin1, pin2, pin3, pin4, pin5, pin6, pin7)
#define SET_DIR_7(t, u, v, w, x, y, z)		\
   SET_DIR_7_ (t, u, v, w, x, y, z)


#define UNSET_DIR_1_(port, pin) UNMASK_1 (PORT_DIR(port), pin)
#define UNSET_DIR_1(x) UNSET_DIR_1_ (x)
#define UNSET_DIR_2_(port1, pin1, port2, pin2)	\
   UNMASK_2 (PORT_DIR(port1), pin1, pin2)
#define UNSET_DIR_2(x, y) UNSET_DIR_2_ (x, y)
#define UNSET_DIR_3_(port1, pin1, port2, pin2, port3, pin3)	\
   UNMASK_3 (PORT_DIR(port1), pin1, pin2, pin3)
#define UNSET_DIR_3(x, y, z) UNSET_DIR_3_ (x, y, z)
#define UNSET_DIR_4_(port1, pin1, port2, pin2, port3, pin3, \
		     port4, pin4)			    \
   UNMASK_4 (PORT_DIR(port1), pin1, pin2, pin3, pin4)
#define UNSET_DIR_4(w, x, y, z) UNSET_DIR_4_ (w, x, y, z)
#define UNSET_DIR_5_(port1, pin1, port2, pin2, port3, pin3, \
		     port4, pin4, port5, pin5)		    \
   UNMASK_5 (PORT_DIR(port1), pin1, pin2, pin3, pin4, pin5)
#define UNSET_DIR_5(v, w, x, y, z) UNSET_DIR_5_ (v, w, x, y, z)
#define UNSET_DIR_6_(port1, pin1, port2, pin2, port3, pin3,	\
		     port4, pin4, port5, pin5, port6, pin6)	\
   UNMASK_6 (PORT_DIR(port1), pin1, pin2, pin3, pin4, pin5, pin6)
#define UNSET_DIR_6(u, v, w, x, y, z) UNSET_DIR_6_ (u, v, w, x, y, z)
#define UNSET_DIR_7_(port1, pin1, port2, pin2, port3, pin3,		\
		     port4, pin4, port5, pin5, port6, pin6)		\
   UNMASK_7 (PORT_DIR(port1), pin1, pin2, pin3, pin4, pin5, pin6, pin7)
#define UNSET_DIR_7(t, u, v, w, x, y, z)	\
   UNSET_DIR_7_ (t, u, v, w, x, y, z)

#define SET_PORT_1_(port, pin) MASK_1 (PORT_OUT(port), pin)
#define SET_PORT_1(x) SET_PORT_1_ (x)
#define SET_PORT_2_(port1, pin1, port2, pin2)	\
   MASK_2 (PORT_OUT(port1), pin1, pin2)
#define SET_PORT_2(x, y) SET_PORT_2_ (x, y)
#define SET_PORT_3_(port1, pin1, port2, pin2, port3, pin3)	\
   MASK_3 (PORT_OUT(port1), pin1, pin2, pin3)
#define SET_PORT_3(x, y, z) SET_PORT_3_ (x, y, z)
#define SET_PORT_4_(port1, pin1, port2, pin2, port3, pin3, port4, pin4) \
   MASK_4 (PORT_OUT(port1), pin1, pin2, pin3, pin4)
#define SET_PORT_4(w, x, y, z) SET_PORT_4_ (w, x, y, z)
#define SET_PORT_5_(port1, pin1, port2, pin2, port3, pin3, port4, pin4, \
			port5, pin5)					\
   MASK_5 (PORT_OUT(port1), pin1, pin2, pin3, pin4, pin5)
#define SET_PORT_5(v, w, x, y, z) SET_PORT_5_ (v, w, x, y, z)
#define SET_PORT_6_(port1, pin1, port2, pin2, port3, pin3, port4, pin4, \
			port5, pin5, port6, pin6)			\
   MASK_6 (PORT_OUT(port1), pin1, pin2, pin3, pin4, pin5, pin6)
#define SET_PORT_6(u, v, w, x, y, z) SET_PORT_6_ (u, v, w, x, y, z)
#define SET_PORT_7_(port1, pin1, port2, pin2, port3, pin3, port4, pin4, \
			port5, pin5, port6, pin6)			\
   MASK_7 (PORT_OUT(port1), pin1, pin2, pin3, pin4, pin5, pin6, pin7)
#define SET_PORT_7(t, u, v, w, x, y, z)	\
   SET_PORT_7_ (t, u, v, w, x, y, z)

#define UNSET_PORT_1_(port, pin) UNMASK_1 (PORT_OUT(port), pin)
#define UNSET_PORT_1(x) UNSET_PORT_1_ (x)
#define UNSET_PORT_2_(port1, pin1, port2, pin2)	\
   UNMASK_2 (PORT_OUT(port1), pin1, pin2)
#define UNSET_PORT_2(x, y) UNSET_PORT_2_ (x, y)
#define UNSET_PORT_3_(port1, pin1, port2, pin2, port3, pin3)	\
   UNMASK_3 (PORT_OUT(port1), pin1, pin2, pin3)
#define UNSET_PORT_3(x, y, z) UNSET_PORT_3_ (x, y, z)
#define UNSET_PORT_4_(port1, pin1, port2, pin2, port3, pin3, \
		      port4, pin4)			     \
   UNMASK_4 (PORT_OUT(port1), pin1, pin2, pin3, pin4)
#define UNSET_PORT_4(w, x, y, z) UNSET_PORT_4_ (w, x, y, z)
#define UNSET_PORT_5_(port1, pin1, port2, pin2, port3, pin3, \
		      port4, pin4, port5, pin5)		     \
   UNMASK_5 (PORT_OUT(port1), pin1, pin2, pin3, pin4, pin5)
#define UNSET_PORT_5(v, w, x, y, z) UNSET_PORT_5_ (v, w, x, y, z)
#define UNSET_PORT_6_(port1, pin1, port2, pin2, port3, pin3, \
		      port4, pin4, port5, pin5, port6, pin6)	\
   UNMASK_6 (PORT_OUT(port1), pin1, pin2, pin3, pin4, pin5, pin6)
#define UNSET_PORT_6(u, v, w, x, y, z) UNSET_PORT_6_ (u, v, w, x, y, z)
#define UNSET_PORT_7_(port1, pin1, port2, pin2, port3, pin3, \
		      port4, pin4, port5, pin5, port6, pin6)		\
   UNMASK_7 (PORT_OUT(port1), pin1, pin2, pin3, pin4, pin5, pin6, pin7)
#define UNSET_PORT_7(t, u, v, w, x, y, z)	\
   UNSET_PORT_7_ (t, u, v, w, x, y, z)
