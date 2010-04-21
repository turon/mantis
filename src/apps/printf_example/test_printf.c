//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    test_printf.c                                                 */
/* Author(s)      Adam Torgerson                                          */
/*                Charles Gruenwald III                                   */
/*   Date:  12/18/03                                                      */
/*                                                                        */
/* test_printf:                                                           */
/* This short application demonstrates the use of the printf library      */
/*                                                                        */
/**************************************************************************/

#include "mos.h"

#include "msched.h"
#include "uart.h"

#include "printf.h"
#include "plat_dep.h"

//const char prog_str[] ARCH_PROGMEM = "test from flash";

static void test_printf(void)
{
   uint32_t test = 0xfffffffe;

   printf("32 bit test: %l\n", test);
   
   printf("This is the number 0 [ %d ]\n", 0);
   printf("This is a string [ %s ]\n","string");
   printf("This is an 8-bit character [ %c ]\n", 'j');
#ifdef ARCH_MICRO
   printf("This is an 8-bit number printed in decimal [ %C ]\n", 230);
   printf("This is a 32-bit number in decimal [ %l ]\n", 4769231);
   printf("This is the number 'fde8' in hex [ %h ]\n", 0xfde8);
   printf("This is the number 1024 in binary [ %b ]\n", 16);
   printf("This is an 8-bit number printed in decimal [ %8C ]\n", 230);
   printf("This is a 32-bit number in decimal [ %8l ]\n", 4769231);
   printf("This is the number 'fde8' in hex [ %8h ]\n", 0xfde8);
   printf("This is the number 1024 in binary [ %8b ]\n", 16);
   printf("This is an 8-bit number printed in decimal [ %08C ]\n", 230);
   printf("This is a 32-bit number in decimal [ %08l ]\n", 4769231);
   printf("This is the number 'fde8' in hex [ %08h ]\n", 0xfde8);
   printf("This is the number 1024 in binary [ %08b ]\n", 16);
#else
   printf("This is an 8-bit number printed in decimal [ %hhd ]\n", 230);
   printf("This is a 32-bit number in decimal [ %uld ]\n", 4769231);
   printf("This is an 8-bit number printed in decimal [ %8hhd ]\n", 230);
   printf("This is a 32-bit number in decimal [ %8uld ]\n", 4769231);
   printf("This is an 8-bit number printed in decimal [ %08hhd ]\n", 230);
   printf("This is a 32-bit number in decimal [ %08uld ]\n", 4769231);
#endif
   printf("This is a 16-bit number in decimal [ %d ]\n", 65000);
   printf("This is another 16-bit number in decimal %x [ %d ]\n",1000, 1000);
   printf("This is the number 1024 in hex [ %x ]\n", 1024);   
   printf("This is the number 1024 in octal [ %o ]\n", 1024);
   printf("The following are the same numbers with padding:\n");
   printf("This is the number 0 [ %8d ]\n", 0);
   printf("This is a string [ %8s ]\n","string");
   printf("This is an 8-bit character [ %8c ]\n", 'j');
   printf("This is a 16-bit number in decimal [ %8d ]\n", 65000);
   printf("This is another 16-bit number in decimal %8x [ %8d ]\n",1000, 1000);
   printf("This is the number 1024 in hex [ %8x ]\n", 1024);   
   printf("This is the number 1024 in octal [ %8o ]\n", 1024);

   printf("The following are the same number with zero-padding:\n");
   printf("This is the number 0 [ %08d ]\n", 0);
   printf("This is a 16-bit number in decimal [ %08d ]\n", 65000);
   printf("This is another 16-bit number in decimal %08x [ %08d ]\n",1000, 1000);
   printf("This is the number 1024 in hex [ %08x ]\n", 1024);   
   printf("This is the number 1024 in octal [ %08o ]\n", 1024);
	
   printf("This is a really long paragraph that is part of our licence agreement... it should be split up into packets however it should arrive as once piece  This program is distributed in the hope that it will be useful,but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.");
   printf("This is a really long paragraph that is part of our licence agreement...\n it should be split up into packets however it should arrive as once piece\n\n  This program is distributed in the hope that it will be useful,\nbut WITHOUT ANY WARRANTY; without even the implied warranty of\n  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n  GNU General Public License for more details.\n\n\n");
   printf("Test complete.\n");
   printf("Test complete2.");

   //printf("This is a string from program memory: %P\n", prog_str);
   
}

void start(void)
{
   mos_thread_new(test_printf, 192, PRIORITY_NORMAL);
}
