//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**********************************
NOTE:
In order to get the application to function properly, you must uncomment the line:
#define MICA2_PROTO 1
from the file:
$MANTIS_HOME/src/mos/dev/include/mica2-light-temp.h
***********************************/
 

#include <inttypes.h>
#include "mos.h"
#include "led.h"
#include "dev.h"
#include "com.h"
#include "msched.h"
#include "printf.h"
#include "clock.h"
#include "adc.h"
#include "avr-adc.h"
#include "mica2-light-temp.h"
#include "fire.h"
#include "cc1000.h"
#include "sem.h"

static comBuf sendBuf;
void recv();

void start (void)
{
    com_mode(IFACE_RADIO, IF_LISTEN);
    mos_thread_new (recv, 128, PRIORITY_NORMAL);
}

//receives packets and outputs sequence number
void recv()
{
    comBuf * recvBuf;

    while (1)
    {
	recvBuf = com_recv(IFACE_RADIO);

	printf ("received packet %d with size %d\n", recvBuf->data[0], recvBuf->size);
	//mos_led_toggle(1);
	com_free_buf(recvBuf);
	      
    }//end while

}//end recv
