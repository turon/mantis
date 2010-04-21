//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>
#include "led.h"
#include "printf.h"
#include "clock.h"

void start (void)
{
  uint16_t a = 0;
  
  mos_led_on(2);
  while(1)
    {
      a++;
      
      mos_led_toggle(0);
      mos_mdelay(1000);
      printf(".");
      if (a== 10)
        {
          a=0;
          mos_led_toggle(1);
          printf("A different message\n");
        }
    }
}
