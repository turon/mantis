//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>

#include "led.h"
#include "msched.h"
#include "dev.h"
#include "command_daemon.h"

void sensor_test();

void get_temp(){
   uint8_t temp;
   temp=dev_read(DEV_MICA2_TEMP, &temp, sizeof(temp));
   printf("Currently it is %d degrees F. (local)\n",temp);
}

void get_light(){
   uint8_t light;
   light=dev_read(DEV_MICA2_LIGHT, &light, sizeof(light));
   printf("light ammount: [%d]\n",light);
}
void start(void){
  mos_thread_new(mos_command_daemon, 256, PRIORITY_HIGH); //start the command server
  mos_register_function("temp",(void *)get_temp);    //register the function 'get temp'
  mos_register_function("light",(void *)get_light); //tell commander to respond to command 'get light'
}

