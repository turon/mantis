//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "serial.h"
#include "mcs.h"

int peek(char *arg)
{
   int val;
   uint16_t word;
   
   printf("Enter memory address: 0x");
   scanf("%x", &val);

   serial_send_byte('r');
   send_address((uint16_t)val);

   word = serial_recv_byte() << 8;
   word |= serial_recv_byte();
   printf("Word at address %.5x: %.4x\n", val, word);

   return 0;
}

int poke(char *arg)
{
   return 0;

}

int heap(char *arg)
{

   return 0;
}

int stack(char *arg)
{
 
   return 0;
}

int thread(char *arg)
{
   uint8_t max_threads;
   int i;

   uint8_t state, priority;
   uint16_t sp, stack, stack_size, function;

   serial_send_byte('t');
   
   max_threads = serial_recv_byte();
   printf("MAX_THREADS: %d\n", max_threads);
   
   printf("ID\tstate\tpri\tsp\tstack-base\tstack-size\tfunc\n");
   printf("-------------------------------------------------------------------\n");  

   for(i = 0; i < max_threads; i++)
   {
      state = serial_recv_byte();
      priority = serial_recv_byte();
      sp = serial_recv_word();
      stack = serial_recv_word();
      stack_size = serial_recv_word();
      function = serial_recv_word();
      printf("%d\t%d\t%d\t0x%04x\t0x%04x\t\t  %4d\t\t0x%04x\n",
	     i,state,priority,sp,stack,stack_size,function);
   }
   printf("-------------------------------------------------------------------\n");

   return 0;
}

int call(char *arg)
{

   return 0;
}

int spawn(char *arg)
{

   return 0;
}

int read_eeprom(char *arg)
{
  int val;
  uint16_t addr;
  uint8_t len, i;

  printf("Enter memory address: 0x");
  scanf("%x", &val);
  addr = (uint16_t)val;
  printf("Bytes to read (1-64): ");
  scanf("%d", &val);
  len = (uint8_t)val;

  serial_send_byte(READ_EEPROM);
  serial_send_byte(addr >> 8); //addr high byte
  serial_send_byte(addr & 0xFF); //addr low byte
  serial_send_byte(len);

  for(i=0; i<len; i++)
    printf("%02x ", serial_recv_byte());  
  printf("\n");

  if(serial_recv_byte() != EEPROM_DONE)
    fprintf(stderr, "Error reading from EEPROM!\n");

  return 0;
}

int write_eeprom(char *arg)
{
  int val;
  uint16_t addr;
  uint8_t len, i;
  uint8_t buf[64];

  printf("Enter memory address: 0x");
  scanf("%x", &val);
  addr = (uint16_t)val;
  printf("Bytes to write (1-64): ");
  scanf("%d", &val);
  len = (uint8_t)val;
  printf("Enter bytes:\n");
  for(i=1; i<len; i++)
    {
      scanf("%x ", &val);
      buf[i] = (uint8_t)val;
    }
  
  serial_send_byte(WRITE_EEPROM);
  serial_send_byte(addr >> 8); //addr high byte
  serial_send_byte(addr & 0xFF); //addr low byte
  serial_send_byte(len);
  for(i=0; i<len; i++)
    serial_send_byte(buf[i]);

  if(serial_recv_byte() != EEPROM_DONE)
    fprintf(stderr, "Error reading from EEPROM!\n");
  return 0;
}
