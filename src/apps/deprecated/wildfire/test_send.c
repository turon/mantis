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

static comBuf sendBuf;

void send();

void start (void)
{

    //turn the radio on
    com_mode(IFACE_RADIO, IF_LISTEN);
    mos_thread_sleep(200);

    mos_thread_new (send, 128, PRIORITY_NORMAL);
}

//sends packets at the rate of 1 per second
void send()
{ 
    uint8_t mySeqNo;
    mySeqNo = 0;

    while (1)
    {
	sendBuf.size = 45;
	sendBuf.data[0] = mySeqNo;

	printf ("sending packet with seqno = %d\n", mySeqNo);
	com_send(IFACE_RADIO, &sendBuf);
	printf ("packet sent\n");
	mySeqNo++;
	if (mySeqNo > 254)
	    mySeqNo = 0;
	mos_led_toggle(0);
	mos_thread_sleep(1000);
    }
}
