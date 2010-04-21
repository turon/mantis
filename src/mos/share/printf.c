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

/**************************************************************************/
/* File:    printf.c                                                      */
/* Author   Adam Torgerson                                                */
/*   Date: 01/25/04                                                       */
/* Edited:  Jeff Rose   :  rosejn@colorado.edu                            */
/*   Date: 03/10/04                                                       */
/*                                                                        */
/* String printing over the uart. (Now using the com layer.)              */
/**************************************************************************/

/** @file printf.c
 * @brief String printing over the uart. (Now using the com layer)
 * @author Adam Torgerson
 * @author Modified: Jeff Rose
 * @author Modified: Charles Gruenwald
 * @date Created: 01/25/2004
 * @date Modified: 03/10/2004
 */
#include "mos.h"

#ifndef PLATFORM_LINUX

#include <string.h>
#include "printf.h"
#include "com.h"
#include "mutex.h"

static comBuf printf_packet;
static mos_mutex_t printf_lock;
static uint8_t charbuff[16]; //buffer for reversing itoa converisions
static uint8_t buffsize;
static uint8_t fieldwidth;
static char padchar;

static void send_string(const char *s);
static void norecurse_32(uint32_t input, uint8_t radix);
static void norecurse_16(uint16_t input, uint8_t radix);
static void check_size(void);

#ifdef ARCH_AVR
static void send_prog_string(PGM_P s);
#endif

#ifdef PLATFORM_TELOSB
#define com_send_packet(packet_ptr) com_send(IFACE_SERIAL2, packet_ptr)
#else
#define com_send_packet(packet_ptr) com_send(IFACE_SERIAL, packet_ptr)
#endif

/* check the size of the packet, send it off if it's too big... */
inline static void check_size(void)
{
   if(printf_packet.size == COM_DATA_SIZE - 2) {
      printf_packet.data[printf_packet.size++] = '\0';
      //com_send(IFACE_SERIAL, &printf_packet);
      com_send_packet(&printf_packet);
      printf_packet.size = 0;
   }
}

/* initialize the mutex */
void printf_init(void)
{
   mos_mutex_init(&printf_lock);
}

/* HACK to let us optimize, define our own version of puts()
   the assembler "optimizes" calls to printf() with no args by
   converting it to a puts () */
int puts(const char *msg)
{
   //printf("%s\n\0", msg);

   mos_mutex_lock(&printf_lock);

   send_string(msg);
   printf_packet.data[printf_packet.size++] = '\n';
   printf_packet.data[printf_packet.size++] = '\0';
   //com_send(IFACE_SERIAL, &printf_packet);
   com_send_packet(&printf_packet);
   printf_packet.size = 0;

   mos_mutex_unlock(&printf_lock);

   return 0;
}

/* HACK to let us optimize, define our own version of putchar()
   the assembler "optimizes" calls to printf() with a single char
   argument by converting it to a putchar() */
int putchar(int character)
{
   mos_mutex_lock(&printf_lock);
   
   printf_packet.size = 2;
   printf_packet.data[0] = character;
   printf_packet.data[1] = '\0';
   //com_send(IFACE_SERIAL, &printf_packet);
   com_send_packet(&printf_packet);
   printf_packet.size = 0;
   
   mos_mutex_unlock(&printf_lock);
   return 0;
}

/** @brief printf Prints the formatted string over the serial line
 * after which a shell must display the text to the screen. Note that
 * %C is for 8 bit numbers, %d for 16-bit and %l for 32 bit. 
 * @param format Format to print
 */
static uint32_t num32;
int16_t printf(const char *format, ...)
{
   va_list arg;
   const char *p = format;

   if(format == NULL)
      return -1;

   mos_mutex_lock(&printf_lock);
   va_start(arg, format);

   printf_packet.size = 0;
   memset(printf_packet.data, 0, COM_DATA_SIZE);
   
   while(*p) {
      // If its not a conversion specifier we just send it
      if(*p != '%') {
	 printf_packet.data[printf_packet.size++] = *p++;
	 check_size();
      } else {
	 // Otherwise we need to substitute in a variable
	 *p++;
	 
	 // TODO the padding options are not implemented for non-numeric stuff
	 // Pad with zeros or spaces?
	 if (*p == '0') {
	    padchar = '0';
	    p++;
	 } else
	    padchar = ' ';
	 
	 // Width of field
	 fieldwidth = 0;
	 while(*p >= '0' && *p <= '9') {
	    fieldwidth = fieldwidth * 10 + *p - '0';
	    p++;
	 }
	 
	 switch(*p++) {
	 case 's': {
	    char *s;
	    s = va_arg(arg, char *);
	    send_string(s);
	    break;
	 }
#ifdef ARCH_AVR
	 case 'P': {
	    PGM_P s;
	    s = va_arg(arg, PGM_P);
	    send_prog_string(s);
	 }
#endif
	 case 'd': {
	    uint16_t num16;
	    num16 = va_arg(arg, uint16_t);
	    norecurse_16(num16, 10);
	    break;
	 }
	 case 'l': {
	    num32 = va_arg(arg, uint32_t);
	    norecurse_32(num32, 10);
	    break;
	 }
	 case 'c': {
	    uint16_t num8 = 0;
	    num8 = (uint8_t)va_arg(arg, uint16_t);
	    printf_packet.data[printf_packet.size++] = num8;
	    check_size();
	    break;
	 }
	 case 'C': {
	    uint16_t num8 = 0;
	    num8 = (uint8_t) va_arg(arg, uint16_t);
	    norecurse_16((uint16_t)num8, 10);
	    break;
	 }
	 case 'x': 
	    printf_packet.data[printf_packet.size++] = '0';
	    check_size();
	    printf_packet.data[printf_packet.size++] = 'x';
	    check_size();
	 case 'h': {
	    uint16_t num16;
	    num16 = va_arg(arg, uint16_t);
	    norecurse_16(num16, 16);
	    break;
	 }
	 case 'o': {
	    uint16_t num16;
	    num16 = va_arg(arg, uint16_t);
	    norecurse_16(num16, 8);
	    break;
	 }
	 case '%':
	    printf_packet.data[printf_packet.size++] = '%';
	    check_size();
	    break;
	 case 'b': {
	    uint16_t num16;
	    num16 = va_arg(arg, uint16_t);
	    norecurse_16(num16, 2);
	    break;
	 }
	 default:
	    printf_packet.data[printf_packet.size++] = '?';
	    check_size();
	    printf_packet.data[printf_packet.size++] = '?';
	    check_size();
	    printf_packet.data[printf_packet.size++] = '?';
	    check_size();
	    break;
	 }
      }
   }
   va_end(arg);

   printf_packet.data[printf_packet.size++] = '\0';
   //com_send(IFACE_SERIAL, &printf_packet);
   com_send_packet(&printf_packet);
   printf_packet.size = 0;

   mos_mutex_unlock(&printf_lock);

   return 0;
}

#ifdef ARCH_AVR
/** @brief printf_P Same as printf, except that the format string must be stored
 * int program memory.
 * @param format Format to print
 */
int16_t printf_P(const ARCH_PROGMEM char *format, ...)
{
   va_list arg;
   const prog_char *p = format;

   if(format == NULL)
      return -1;

   mos_mutex_lock(&printf_lock);
   va_start(arg, format);

   printf_packet.size = 0;
   
   char c = pgm_read_byte(p);
   while(c) {
      // If its not a conversion specifier we just send it
      if(c != '%') {
	 printf_packet.data[printf_packet.size++] = c;
	 c = pgm_read_byte(++p);
	 check_size();
      } else {
	 // Otherwise we need to substitute in a variable
	 c = pgm_read_byte(++p);
	 
	 // TODO the padding options are not implemented for non-numeric stuff
	 // Pad with zeros or spaces?
	 if (c == '0') {
	 	padchar = '0';
	 	c = pgm_read_byte(++p);
	 } else
	 	padchar = ' ';
	 
	 // Width of field
	 fieldwidth = 0;
	 while(c >= '0' && c <= '9') {
	 	fieldwidth = fieldwidth * 10 + c - '0';
	 	c = pgm_read_byte(++p);
	 }
	 
	 switch(c) {
	 case 's': {
	    char *s;
	    s = va_arg(arg, char *);
	    send_string(s);
	    break;
	 }
	 case 'P': {
	    PGM_P s;
	    s = va_arg(arg, PGM_P);
	    send_prog_string(s);
	 }
	 case 'd': {
	    uint16_t num16;
	    num16 = va_arg(arg, uint16_t);
	    norecurse_16(num16,10);
	    break;
	 }
	 case 'l': {
//	    uint32_t num32;
	    num32 = va_arg(arg, uint32_t);
	    norecurse_32(num32, 10);
	    break;
	 }
	 case 'c': {
	    uint16_t num8 = 0;
	    num8 = (uint8_t)va_arg(arg, uint16_t);
	    printf_packet.data[printf_packet.size++] = num8;
	    check_size();
	    break;
	 }
	 case 'C': {
	    uint16_t num8 = 0;
	    num8 = (uint8_t) va_arg(arg, uint16_t);
	    norecurse_16((uint16_t)num8, 10);
	    break;
	 }
	 case 'x': 
	    printf_packet.data[printf_packet.size++] = '0';
	    check_size();
	    printf_packet.data[printf_packet.size++] = 'x';
	    check_size();
	 case 'h': {
	    uint16_t num16;
	    num16 = va_arg(arg, uint16_t);
	    norecurse_16(num16, 16);
	    break;
	 }
	 case 'o': {
	    uint16_t num16;
	    num16 = va_arg(arg, uint16_t);
	    norecurse_16(num16, 8);
	    break;
	 }
	 case '%':
	    printf_packet.data[printf_packet.size++] = '%';
	    check_size();
	    break;
	 case 'b': {
	    uint16_t num16;
	    num16 = va_arg(arg, uint16_t);
	    norecurse_16(num16, 2);
	    break;
	 }
	 default:
	    printf_packet.data[printf_packet.size++] = '?';
	    check_size();
	    printf_packet.data[printf_packet.size++] = '?';
	    check_size();
	    printf_packet.data[printf_packet.size++] = '?';
	    check_size();
	    break;
	 }
	c = pgm_read_byte(++p);
      }
   }
   va_end(arg);

   printf_packet.data[printf_packet.size++] = '\0';
   com_send(IFACE_SERIAL, &printf_packet);
   printf_packet.size = 0;

   mos_mutex_unlock(&printf_lock);

   return 0;
}
#endif

/* Send a NULL terminated string. */
static void send_string(const char *s)
{
   /*while(fieldwidth--) {
      printf_packet.data[printf_packet.size++] = padchar;
      check_size();
   }*/
   while(*s) {
      printf_packet.data[printf_packet.size++] = *s++;
      check_size();
   }
}

#ifdef ARCH_AVR
/* Send a NULL terminated string from flash. */
static void send_prog_string(PGM_P s)
{
   /*while(fieldwidth--) {
      printf_packet.data[printf_packet.size++] = padchar;
      check_size();
   }*/
   uint8_t byte;
   while((byte = pgm_read_byte(s++))) {
      printf_packet.data[printf_packet.size++] = byte;
      check_size();
   }
}
#endif

/* Convert a number to a certain radix and send over uart.
 * 16-bit version */
static void norecurse_16(uint16_t input, uint8_t radix)
{
   uint8_t remainder = input % radix;
   buffsize = 0;

   if(input == 0)  
      charbuff[buffsize++] = '0';
   
   while(input >= radix) {
      if(remainder <= 9)
	 charbuff[buffsize++] = remainder + '0';
      else
	 charbuff[buffsize++] = remainder - 10 + 'A';

      input /= radix;
      remainder = input % radix;
   }
   
   //last place
   if(remainder > 0) {
      if(remainder <= 9)
	 charbuff[buffsize++] = remainder + '0';
      else
	 charbuff[buffsize++] = remainder - 10 + 'A';
   }
   
   // Calculate amount we still need to pad.  We never truncate a result.
   if(fieldwidth > buffsize)
      fieldwidth -= buffsize;
   else
      fieldwidth = 0;
   while(fieldwidth--) {
      printf_packet.data[printf_packet.size++] = padchar;
      check_size();
   }
   
   while(buffsize > 0) {
      printf_packet.data[printf_packet.size++] = charbuff[--buffsize];
      check_size();
   }
}

/* Convert a number to a certain radix and send over uart.
 * 32-bit version */
static void norecurse_32(uint32_t input, uint8_t radix)
{  
   uint8_t remainder = input % radix;

   uint8_t *p = charbuff;
   
   buffsize = 0;
   
   if(input == 0)
      charbuff[buffsize++] = '0';   
   
   while(input >= radix) {
      if(remainder <= 9)
	 *p++ = remainder + '0';
      else
	 *p++ = remainder - 10 + 'A';

      buffsize++;
      input /= radix;
      remainder = input % radix;
   }

   if(remainder > 0) {
      if(remainder <= 9)
	 *p++ = remainder + '0';
      else
	 *p++ = remainder - 10 + 'A';
      buffsize++;
   }
   
   // Calculate amount we still need to pad.  We never truncate a result.
   if(fieldwidth > buffsize)
      fieldwidth -= buffsize;
   else
      fieldwidth = 0;
   while(fieldwidth--) {
      printf_packet.data[printf_packet.size++] = padchar;
      check_size();
   }

   
   while(buffsize > 0) {
      buffsize--;
      printf_packet.data[printf_packet.size++] = charbuff[buffsize];
      check_size();
   }
}

#endif

