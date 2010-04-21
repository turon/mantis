//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/*
  Project Mantis
  File:   shell_server.c
  Author: Brian Shucker & Jeff Rose
  Date: 2-21-03

  Simple shell server for MANTIS node
*/

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "uart.h"
#include "msched.h"
#include "network.h"
#include "mcs.h"
#include "i2c_eeprom.h"
#include "led.h"

#ifndef PLATFORM_LINUX
#include "mem.h"
#include "program.h"
#include "flash.h"

extern Thread threads[];
extern node *freelist;
#endif

#define MAX_COMMAND_LEN 64
#define UART 0

#define TRUE 1
#define FALSE 0

uint16_t load_state;

void mcs_comm_open()
{
   mos_uart_open(UART);
}

void mcs_send(uint8_t byte)
{
   mos_uart_send(UART, byte);
}

uint8_t mcs_recv()
{
   return mos_uart_recv(UART);
}

/*peeks at a memory address*/
void peek()
{

}

/*pokes a memory address*/
void poke()
{

}

void heapDump()
{
#ifndef PLATFORM_LINUX
   node *cur = freelist;
   //char buf[32];

   do{
      //sprintf(buf, "%04x: %d\n", (int16_t)cur, cur->size);
      //print(buf);
      cur = cur->next;
   } while(cur != freelist);
#else
   mcs_send(NOT_IN_XMOS);
#endif
}

/*prints out the thread table*/
void threadDump()
{
#ifndef PLATFORM_LINUX
   /*
     int i;

     print("ID\tstate\tpri\tsp\tstack-base\tstack-size\tfunc\n");
     print("-------------------------------------------------------------------\n");  
     for(i=0; i<MAX_THREADS; i++)
     {
     sprintf(buf, "%d\t%d\t%d\t0x%04x\t0x%04x\t\t  %4d\t\t0x%04x\n",
     i,
     threads[i].state,
     threads[i].priority,
     (uint16_t)threads[i].sp,
     (uint16_t)threads[i].stack,
     (uint16_t)threads[i].stackSize,
     (uint16_t)threads[i].func);
     print(buf);
     }
   */
#else
   mcs_send(NOT_IN_XMOS);
#endif

}

void stackDump()
{
#ifndef PLATFORM_LINUX
   /*
     uint8_t id;
     uint8_t *addr;
  
     id = (uint8_t)strtol(args, NULL, 10);
     if(id >= MAX_THREADS)
     print("Error: no such thread\n");
     else
     for(addr=threads[id].stack;
     addr<(threads[id].stack + threads[id].stackSize);
     addr+=2)
     {
     char buf[24];
     sprintf(buf, "0x%04x: 0x%02x 0x%02x\n",
     (uint16_t)addr, *addr, *(addr+1));
     print(buf);
     }
   */
#else
   mcs_send(NOT_IN_XMOS);
#endif
}

void call()
{
   //void (*func)(void);

   /*TODO: do address translation on client side*/
   /*  
       #ifdef PLATFORM_MICA
       //text addressing on AVR is by 16-bit word, not byte
       func = (void *)((uint16_t)(strtol(args, NULL, 16) >> 1));
       #elif PLATFORM_NYMPH
       //text addressing on AVR is by 16-bit word, not byte
       func = (void *)((uint16_t)(strtol(args, NULL, 16) >> 1));
       #else
       func = (void *)((uint16_t)strtol(args, NULL, 16));
       #endif
       (func)();
   */
}

void spawn()
{
   /*
     void (*func)(void);
     char *afterFunc;
     char *afterStack;
     uint16_t stack;
     uint8_t priority;
   */
   /*TODO: do address translation on client side*/
   /*
   //get function pointer
   #ifdef PLATFORM_MICA
   //text addressing on AVR is by 16-bit word, not byte
   func = (void *)((uint16_t)(strtol(args, &afterFunc, 16) >> 1));
   #elif PLATFORM_NYMPH
   //text addressing on AVR is by 16-bit word, not byte
   func = (void *)((uint16_t)(strtol(args, &afterFunc, 16) >> 1));
   #else
   func = (void *)((uint16_t)strtol(args, &afterFunc, 16));
   #endif

   //get stack size
   stack = (uint16_t)strtol(afterFunc, &afterStack, 10);
   //get priority
   priority = (uint8_t)strtol(afterStack, NULL, 10);

   //TODO: error checking!
   thread_new(func, stack, priority);
   */
}

void *get_address()
{
   uint8_t i;
   void *addr = NULL;

   for(i=0; i<sizeof(void *); i++)
      ((uint8_t *)&addr)[i] = mcs_recv();

#ifndef PLATFORM_LINUX  
   /* on AVR, Now set RAMPZ if necessary. */
   if((uint16_t)addr >= 0x7FFF)
      RAMPZ |= 0x01;
   else
      RAMPZ = 0;
 
   /* Convert to a byte address and return. */
   addr = (void *)((uint16_t)addr << 1);
#endif

   //addr = (void *)ntohl(addr);
   mcs_send(ADDRESS_RECVD);
   return addr;
}

void read_fuses()
{
   RAMPZ = 0;
   
   flash_enable_rww(); // Make sure the RWW section is enabled

   /* Send the fuse values. */
   mcs_send(flash_read_fuse((void *)(0x0000))); // Low
   mcs_send(flash_read_fuse((void *)0x0003)); // High
   mcs_send(flash_read_fuse((void *)0x0002)); // Extended
   mcs_send(flash_read_fuse((void *)0x0001)); // Lock bits
}

void read_flash()
{
#ifndef PLATFORM_LINUX
   uint16_t i;
   void *addr;
   uint16_t word;

   flash_enable_rww(); // Make sure the RWW section is enabled
   
   /* Get the flash page address. */
   addr = get_address();

   for(i = 0; i < PAGESIZE; i+=2)
   {
      word = flash_read_word(addr);
      mcs_send((uint8_t)(word >> 8));
      mcs_send((uint8_t)word);
      addr += 2;
   }
   mos_uart_close(0);
   mos_uart_open(0);
   mcs_send(FLASH_READ_COMPLETE);
#else
   //TODO: linux program read goes here
#endif
}

//TODO: get some real error checking in here
void write_page(void)
{
#ifndef PLATFORM_LINUX
   uint8_t pagebuf[PAGESIZE];
   uint16_t i;
   void *addr;
   //uint8_t int_handle;
   
/*
   if(load_state == 0)
      start_code_image();
   load_state++;
*/
  
   addr = get_address();
   for(i=0; i<PAGESIZE; i++)
      pagebuf[i] = mcs_recv();

   mcs_send(PAGE_RECVD);

#warning "Not sure if this mos_prog_add_page () is correct"
   if(mos_prog_add_page (pagebuf) == PROG_OK)
      mcs_send(PAGE_WRITTEN);
   else
      mcs_send(FLASH_FULL);

#else
   //TODO: linux program write goes here
#endif
}

void write_eeprom(void)
{
   uint16_t addr, i;
   uint8_t len;
   uint8_t buf[64];

   /*get address*/
   addr = mcs_recv();
   addr <<= 8;
   addr |= mcs_recv();

   /*get length*/
   len = mcs_recv();

   /*get data*/
   for(i=0; i<len; i++)
      buf[i] = mcs_recv();

   if (mos_i2c_eeprom_write(EEPROM_DEV, addr, buf, len) == EEPROM_OK)
      mcs_send(EEPROM_DONE);
   else
      mcs_send(BAD_EEPROM);
}

void read_eeprom(void)
{
   uint16_t addr, i;
   uint8_t len;
   uint8_t buf[64];
   uint8_t result;

   /*get address*/
   addr = mcs_recv();
   addr <<= 8;
   addr |= mcs_recv();

   /*get length*/
   len = mcs_recv();

   /*read eeprom*/
   result = i2c_eeprom_read(EEPROM_DEV, addr, buf, len);
  
   for(i=0; i<len; i++)
      mcs_send(buf[i]);

   if(result == EEPROM_OK)
      mcs_send(EEPROM_DONE);
   else
      mcs_send(BAD_EEPROM);
}

void shell_server(void)
{
   uint8_t command;
   uint8_t keepGoing = FALSE;

   load_state = 0;

   mcs_comm_open();
   i2c_eeprom_init();

   /*identify ourselves to the shell*/
   mcs_send(MCS_PRESENT);
   mcs_send(MCS_PRESENT);

   if(mcs_recv() == SHELL_PRESENT)
   {
      keepGoing = TRUE;
      while(keepGoing)
      {
	 command = mcs_recv();
	 switch(command)
	 {
	 case WRITE_PAGE:
	    write_page();
	    break;
	 case READ_FLASH:
#warning "Using 1 for red LED"
	    mos_led_on(1);
	    read_flash();
	    break;
	 case READ_FUSES:
	    read_fuses();
	    break;
	 case WRITE_EEPROM:
	    write_eeprom();
	    break;
	 case READ_EEPROM:
	    read_eeprom();
	    break;
	 case QUIT:
	    keepGoing = FALSE;
	    break;
	 case START:
	    mcs_send(START);
	    //commit_and_reset();
	    break;
	 default:
	    ;
	 }
      }
   }
}

void blink(void)
{
   uint16_t i;
   while(1)
   {
      mos_led_toggle(1);
      for(i=0; i<0xffff; i++);
   }
}

void blink2(void)
{
   uint16_t i;
   while(1)
   {
      mos_led_toggle(0);
      mos_led_toggle(2);
      for(i=0; i<0xffff; i++);
   }
}

void start(void)
{
   mos_thread_new(shell_server, 512, PRIORITY_NORMAL);
   mos_thread_new(blink, 128, PRIORITY_NORMAL);
   mos_thread_new(blink2, 128, PRIORITY_NORMAL);
}
