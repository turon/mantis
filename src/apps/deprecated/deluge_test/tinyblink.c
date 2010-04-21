//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/*
  Project Mantis
  File: mantis.c
  Author: Jeff Rose
  Date: 1-25-03

  This is where it all starts, and ends.
*/

/** @file main.c
 * @brief This is where it all starts, and ends.
 * @author Jeff Rose
 * @date 01/25/2003
 */

#include "mos.h"
#include <inttypes.h>
//#include <config.h>

inline void mos_udelay(uint16_t usec)
{
   while (usec > 0) {
      asm volatile  ("nop" ::);
      asm volatile  ("nop" ::);
      asm volatile  ("nop" ::);
      asm volatile  ("nop" ::);
      asm volatile  ("nop" ::);
      asm volatile  ("nop" ::);
      usec--;
   }
}

int main(void)
{
	while (1)
	{
		PORTA = ~1;
		mos_udelay (0xffff);
		PORTA = ~2;
		mos_udelay (0xffff);
		PORTA = ~4;
		mos_udelay (0xffff);
	}
}
