//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/*   file:  throttle.c  
 * edited:  Charles Gruenwald III
 *   date:  02/25/04
 *
 */

#include "mos.h"

#include "clock.h"
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "led.h"
#include "msched.h"
#include "com.h"
#include "command_daemon.h"


#define THRESHOLD 32 //approx one minute
// time registers
uint16_t msec;
uint16_t sec;
uint16_t bytes_recvd;

void recv_bytes();

void start(void){
   mos_thread_new(mos_command_daemon, 128, PRIORITY_NORMAL);
   mos_thread_new(recv_bytes, 128, PRIORITY_NORMAL);
   clock_init();
}

void recv_bytes()
{
   comBuf *recv_pkt;
   uint8_t i, size;
   uint8_t new_val;

   printf("Throttle Test initialized....\n");
   mos_led_display(0);
   while(1){
      recv_pkt = com_recv(IFACE_RADIO);
      bytes_recvd+=recv_pkt->size;
      mos_led_display(bytes_recvd);
   }
}


void every_second(){
  sec++;
  if(sec>=THRESHOLD){ //threshold allows us to be called more than 1/s
      printf("%d b/s\n",bytes_recvd>>5);
      bytes_recvd=0;
      sec=0;
  }
}
void clock_init(void){
  sec=0;
  msec=0;
  
  timer3_init();
  
  // attach service to clock interrupt
  timer_attach(TIMER3OUTCOMPAREA_INT, clock_service);
}

/* counts the time, called every ms */
void clock_service(){
  msec++;
  if(msec >= 1000){ //every s...  
    every_second();
    msec = 0;       //reset ms count
    //mos_led_display(sec);
  }//mtics
}
