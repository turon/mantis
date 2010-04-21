//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/*
 * file: rf_commander.c
 * author: charles gruenwald   :   gruenwal@colorado.edu
 *
 * description: commander_rf.c is an application that is an example
 * of how to use net_event layer to register functions with commands.
 * in the 'start' function, a better description is provided of how
 * to register these functions.
 *
 */
#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "command_daemon.h"
#include "node_net_event.h"
#include "printf.h"
#include "com.h"
#include "clock.h"
#include "led.h"
#include "node_id.h"
#ifdef PLATFORM_MICA_ANY
#include "mica2-sounder.h"
#endif

static boolean verbose;

typedef void (*reset_func)(void);

static uint8_t sending_commands;
static uint16_t destination;


void send(){
   while(1){
      mos_led_display(7);
      mos_udelay(5000);
      mos_led_display(0);
      send_event(destination, COMM_DELAY_GET);
      mos_udelay(5000);
   }
}

void local_toggle(){
   mos_led_toggle(1);
   printf("\nlocal led toggle");

}

void set_destination(){
   uint16_t new_dest;
   new_dest=prompt_long("dest for remote commands:");
   destination=new_dest;
}

/* setters */
void rset_power(){
   uint16_t new_power;
   new_power = prompt_long("new rf power for remote node:");
   send_event_arg(destination, RF_POWER_SET,new_power);
}

void rset_node_id(){
   uint16_t new_node_id;
   new_node_id = prompt_long("new remote node id:");
   send_event_arg(destination, SET_NODE_ID,new_node_id);
}

void rset_comm_delay(){
   uint16_t forward_delay;
   forward_delay=prompt_long("data forwarding delay (sec):");
   send_event_arg(destination, COMM_DELAY_SET,forward_delay);
}

void rset_sense_delay(){
   uint16_t comm_delay;
   comm_delay=prompt_long("sensor reading delay (sec):");
   send_event_buf(destination, SENSE_DELAY_SET,(uint8_t *)&comm_delay,sizeof(comm_delay));
}

/* getters */
void rget_comm_delay(){
   printf("comm delay request...\n");
   send_event(destination, COMM_DELAY_GET);
}

void rget_sense_delay(){
   printf("sense delay request...\n");
   send_event(destination, SENSE_DELAY_GET);
}


void rget_power(){
   printf("rf power request...\n");
   send_event(destination, RF_POWER_GET);
}

/* printf's for responses */
void print_sense_delay(void *p){
   uint16_t sense_delay = *(uint16_t *)p;
   printf("%d :", mos_get_event_source(p));
   printf(" delaying [ %d ] counts for sensor readings",sense_delay);
   printf ("\nMOS Commander %d$",mos_node_id_get ());
}
void print_comm_delay(void *p){
   uint16_t comm_delay = *(uint16_t *)p;
   printf("%d :", mos_get_event_source(p));
   printf(" delaying [ %d ] counts for communication delay",comm_delay);
   printf ("\nMOS Commander %d$",mos_node_id_get ());
}
void print_light_temp(void *p){
   uint8_t *params = (uint8_t *)p;
   if(verbose)
      printf("%d : light: %d temp: %d \n", mos_get_event_source(p),
	     params[0], params[1]);
}
void print_rf_power_value(void *p){
   uint8_t *params = (uint8_t *)p;
   printf("%d : rf power: %d \n", mos_get_event_source(p),params[0]);
}
void rset_leds(){
   uint8_t leds;
   leds=prompt_uint8("remote change leds to:");
   send_event_arg8(destination, LEDS, leds);
}

void set_quiet(){    verbose=FALSE;}
void set_verbose(){   verbose=TRUE;}

void start(void)
{
   verbose = FALSE;
   mos_thread_new(mos_command_daemon,128,PRIORITY_NORMAL);
   mos_register_function("quiet",set_quiet);
   mos_register_function("verbose",set_verbose);
   mos_register_function("rset comm delay",rset_comm_delay);
   mos_register_function("rget comm delay",rget_comm_delay);
   mos_register_function("rset sense delay",rset_sense_delay);
   mos_register_function("rget sense delay",rget_sense_delay);
   mos_register_function("send",send);
   mos_register_function("destination", set_destination);
   mos_register_function("rset leds", rset_leds);
   mos_register_function("rset power", rset_power);
   mos_register_function("rget power", rget_power);
   mos_register_function("rset node id",rset_node_id);
   

/* the following is where we register the net events...
    * first we must start the mos_net_daemon thread,
    * next we must register a function with a net_event
    * the net_event must be a uint16_t, while the function
    * must be the prototype int my_function(void *p)
    */
   mos_node_id_set(0);
   mos_thread_new(mos_net_daemon,128,PRIORITY_NORMAL);
   mos_register_rf_function(SENSE_DELAY_VALUE, (void *)print_sense_delay);
   mos_register_rf_function(COMM_DELAY_VALUE, (void *)print_comm_delay);
   mos_register_rf_function(LIGHT_AND_TEMP, (void *)print_light_temp);
   mos_register_rf_function(RF_POWER_VALUE, (void *)print_rf_power_value);
   sending_commands=0;
   destination = 255;
}
