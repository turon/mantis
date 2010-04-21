//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>
#include "led.h"
#include "dev.h"
#include "com.h"
#include "msched.h"
#include "clock.h"
#include "adc.h"
#include "led.h"
#include "printf.h"
#include "sem.h"
#include "user_button.h"

mos_sem_t button;


void button_thread()
{
  while(1)
    {
      mos_sem_wait(&button);
      mos_led_toggle(0);
      
      printf("Node button pushed\n");
    }
}


void button_handler(void)
{
   mos_sem_post(&button);
}

void start (void)
{
  mos_sem_init(&button,0);
  
  mos_enable_user_button(button_handler);
  
  
  mos_thread_new (button_thread, 128, PRIORITY_NORMAL);
}





