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
/* File:    command_daemon.c                                              */
/* Author      Charles Gruenwald III   :  gruenwal@colorado.edu           */
/*   Date:  04/14/2004                                                    */
/*                                                                        */
/* Mantis Commander:                                                      */
/* This is an application which will allow interfacing with a nymph from  */
/* a computer over the serial line, it provides arbitrary functions.      */
/**************************************************************************/

#include "mos.h"

#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>

#ifndef SCONS
#include <config.h>
#endif

#include "com.h"
#include "command_list.h"
#include "command_daemon.h"
#include "crc.h"
#include "dev.h"

#ifdef ARCH_AVR
#include "boot.h" //gives us the boot control block prototype
#else
#warning "other arches need a boot control block"
#endif

#include "led.h"
#include "mcs.h"
#include "msched.h"
#include "node_id.h"
#include "printf.h"

#ifdef PLATFORM_MICA2 || PLATFORM_MICA2DOT
#include "cc1000.h"
#include "cc1000_csma.h"
#endif
#if defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)
#include "cc2420.h"
#endif

static comBuf *recvd;
static comBuf send;

static inline void send_byte_command(uint8_t iface, uint8_t cmd);

/** @brief A typecast for a function pointer to the reset vector */
typedef void (*reboot_func)(void);

static void reboot();
static void mos_show_prompt();
static bool command_packet_parse(comBuf *recvd);
static void set_id();
static void get_id();
static void change_leds();
static void radio_send();
static void pong();
static void mos_help_list();

/** @brief Maximum number of registered commands */
#define MAX_COMMAND_COUNT 30

command_list_t mycommands[MAX_COMMAND_COUNT];

extern mos_mutex_t command_mutex;

static void mos_help_list()
{
   help_list(mycommands, MAX_COMMAND_COUNT);
}

/** @brief Sends one byte of data over the serial line
 *
 * @param iface Interface to send byte on
 * @param cmd Data to send
 */
static inline void send_byte_command(uint8_t iface, uint8_t cmd)
{
   send.data[0] = cmd;
   send.size = 1;
   
   if(iface == IFACE_SERIAL)
      com_send(IFACE_STDIO, &send);
   else if(iface == IFACE_RADIO)
      ;//com_send(IFACE_RADIO, &send);
}

/** @brief Sends the command to tell the shell to reset it's timer */
void mos_clear_ext_timer()
{
   send.data[0] = 255;
   send.data[1] = 255;
   send.data[2] = 0;
   send.size = 3;
   com_send(IFACE_STDIO, &send);
}

/** @brief Sends the command telling the shell to show it's timer */
void mos_show_ext_timer()
{
   send.data[0] = 255;
   send.data[1] = 255;
   send.data[2] = 1;
   send.size = 3;
   com_send(IFACE_STDIO, &send);
}

/** @brief Sends the command telling the shell to show then clear it's timer */
void mos_show_and_clear_ext_timer()
{
   send.data[0] = 255;
   send.data[1] = 255;
   send.data[2] = 2;
   send.size = 3;
   com_send(IFACE_STDIO, &send);
}

/** @brief Displays the mos prompt. */
static void mos_show_prompt()
{
   printf("\nMOS Commander %d$", mos_node_id_get());
}

void mos_command_daemon_init(void)
{
   char ping[2];

   mos_mutex_init(&command_mutex);   

   mos_register_function("leds", change_leds);
   mos_register_function("help", mos_help_list);

   mos_register_function("set_id", set_id);
   mos_register_function("get_id", get_id);

#if defined(ARCH_AVR)  
   ping[0] = SHELL_PRESENT;
   ping[1] = '\0';
   //mycommands[0].input = NULL; //init the list;
   mos_register_function((char *)ping, pong);
   mos_register_function("MR", reboot);
   mos_register_function("radio", radio_send);
#endif
}

void mos_command_daemon(void)
{
   // For backward compatibility, initialize if not done already
   mos_command_daemon_init();
   com_mode(IFACE_STDIO, IF_LISTEN); // Put the uart into listen mode

   printf("Mantis Commander v1.3. Node:%d\n", mos_node_id_get());
   while(1) {
      mos_show_prompt();
      recvd = com_recv(IFACE_STDIO);
      if(!recvd) {
	 printf("serial busy?\n");
	 continue;
      }
      if(recvd->size > 2 && recvd->data[0] == 0 && recvd->data[1] == 0) {

#ifdef ARCH_AVR	 
	 com_send(IFACE_RADIO, recvd);
#endif
      } else if(mos_command_parse(recvd) == false) {//find/execute command
	 recvd->data[recvd->size] = '\0';
	 mos_help_list();
      }
      com_free_buf(recvd);//free buffer to the pool
   }
}

bool mos_register_function(char * name, void (*func_pointer)(void))
{
   bool ret = register_function(mycommands, name, func_pointer,
				MAX_COMMAND_COUNT);
   return ret;
}

bool mos_command_parse(comBuf *recvd)
{
   if(recvd->data[recvd->size - 1] == '\n')
      recvd->data[recvd->size - 1] = '\0';
   if(command_packet_parse(recvd) == false) {
#if defined(PLATFORM_TELOSB)
      printf("command not understood.\n");
#else
      printf("[");
      com_send(IFACE_STDIO, recvd);
      printf("] size [%d] not understood!\n", recvd->size);
#endif

      return false;
   }
   return true;
}

/* wrapper function for the shared library call command_parse. */
static bool command_packet_parse(comBuf *recvd)
{
   char *actual_command = (char *)recvd->data;
   bool ret = command_parse(mycommands, actual_command, MAX_COMMAND_COUNT);
   return ret;
}

char prompt_char(char *string)
{
   char input;
   
   printf(string);                 
   recvd = com_recv(IFACE_STDIO);
   
   input = recvd->data[0];         //grab the first byte
   com_free_buf(recvd);            
   return(input);                   //return the byte
}

uint16_t prompt_long(char *string)
{
   uint8_t i = 0;
   uint16_t input;
   char curr_char;

   input = 0;
   printf (string);

   recvd = com_recv(IFACE_STDIO);

   uint8_t minus = 0;
   if (recvd->data[0]=='-') {
      minus = 1;
      i = 1;
   }
	
   for (; i < recvd->size; i++) {
      curr_char = recvd->data[i];
      if (curr_char >= '0' && curr_char <= '9') {
	 input *= 10;
	 input += curr_char - '0';
      }
   }
   com_free_buf (recvd);
   
   if (minus)
      return (uint16_t)-(int16_t)input;
   
   return input;
}

uint32_t prompt_longlong(char *string)
{
   uint8_t i = 0;
   uint32_t input;
   char curr_char;

   input = 0;
   printf (string);

   recvd = com_recv(IFACE_STDIO);

   uint8_t minus = 0;
   if (recvd->data[0] == '-') {
      minus = 1;
      i = 1;
   }
	
   for (; i < recvd->size; i++) {
      curr_char = recvd->data[i];
      if (curr_char >= '0' && curr_char <= '9') {
	 input *= 10;
	 input += curr_char - '0';
      }
   }
   com_free_buf (recvd);

   if (minus)
      return (uint32_t)-(int32_t)input;
   
   return input;
}

uint8_t prompt_uint8(char * string)
{
   uint8_t i = 0;
   uint8_t input = 0;
   char curr_char;

   printf(string);

   recvd = com_recv(IFACE_STDIO);

   uint8_t minus = 0;
   if (recvd->data[0] == '-') {
      minus = 1;
      i = 1;
   }
	
   for(; i < recvd->size; i++) {
      curr_char = recvd->data[i];

      if(curr_char >= '0' && curr_char <= '9'){
	 input *= 10;
	 input += curr_char - '0';
      }
   }
   com_free_buf(recvd);

   if (minus)
      return (uint8_t)-(int8_t)input;
   
   return input;
}

/* default functions */
static void change_leds()
{   
   static uint8_t num = 0;
   mos_led_display(num++);
}

static void set_id()
{
   uint16_t addr;
   addr = prompt_long("#:");
   mos_node_id_set(addr);
}

static void get_id()
{
   uint16_t addr;
   addr = mos_node_id_get();
   printf("addr: %d\n",addr);
}

/*send a string over the radio... soon to be deprecated */
static void radio_send()
{
   printf("send:");
   recvd = com_recv(IFACE_STDIO);
   if(recvd->data[recvd->size - 1] == '\n')
      recvd->data[recvd->size--] = '\0';

#ifdef ARCH_AVR
   com_send(IFACE_RADIO, recvd);
#endif
   com_free_buf(recvd);
}

static void reboot(void)
{
   // disable the scheduler.
#ifdef ARCH_AVR
   TCCR0 = (0 << CS02) | (0<< CS01) | (0 << CS00);  // sys clk /1024 (1 sec)
   SPCR &= ~(1 << SPE);
   cli();
#else
#warning "need to implement reboot for other arches"
#endif
   reboot_func rfunc = (reboot_func)0x1E000;
   rfunc();
}

static void pong()//ping from shell, pong from app
{
   send_byte_command(IFACE_STDIO, APP_PRESENT);
}
