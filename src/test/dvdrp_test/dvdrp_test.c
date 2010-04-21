/*
  This file is part of DV/DRP
  See http://serl.cs.colorado.edu/

  Copyright (C) 2003 University of Colorado, Boulder

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  (See http://www.gnu.org/copyleft/gpl.html)
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307,
  USA, or send email to mantis-users@cs.colorado.edu.
*/

/**
 * File:     dvdrp_test.c      
 * Author:   Cyrus Hall - hallcp@cs.colorado.edu
 * Date:     07-20-2004
 */

#include <inttypes.h>

#include "printf.h"
#include "led.h"
#include "msched.h"
#include "dvdrp.h"
#include "net.h"
#include "clock.h"
#include "cc1000.h"
#include "clock.h"
#include "mem.h"

#include "dvdrp_test.h"

static comBuf stack_buf;
dvdrp_attrib attribs[1];
mos_alarm_t alarm;

Thread *main_t;
Thread *stk_chk_t;
extern Thread *net_t;

void generator ();

mos_alarm_t alarm_stk;

void wakeup_stk(void *n_data)
{
    mos_thread_resume(stk_chk_t);

    mos_alarm(&alarm_stk, 5, 0);
}

void stack_check() {
    stk_chk_t = mos_thread_current();

    alarm_stk.func = wakeup_stk;
    mos_alarm(&alarm_stk, 1, 0);

    while(1) {
	printf("ge: %d\n", mos_check_stack(main_t));
//	printf("nt: %d\n", mos_check_stack(net_t));
	printf("ts: %d\n", mos_check_stack(stk_chk_t));

	mos_thread_suspend();
    }
}

void start (void)
{
    //give us control over serial/rf
    //mos_thread_new(mos_command_daemon, 128, PRIORITY_NORMAL);
    //start this thread
    mos_thread_new(generator, 224, PRIORITY_NORMAL);
    mos_thread_new(net_thread, 192, PRIORITY_NORMAL);
//    mos_thread_new(stack_check, 128, PRIORITY_NORMAL);
}

void wakeup(void *n_data) {
    mos_thread_resume(main_t);

//    mos_alarm(&alarm, 1, 0);
}

void generator ()
{
    int8_t ret;
    uint8_t i;

    printf("Hello from DV/DRP test!\n");
    net_init();
    dvdrp_init();

    com_ioctl(IFACE_RADIO, CC1000_TX_POWER, 1);
    
    main_t = mos_thread_current();
    
    //printf("Doing mem tests.\n");
    //pool_tests();
    
    attribs[0].name = VELOCITY;

    ret = net_proto_set(DVDRP_PROTO_ID);
    if(ret == NET_PROTO_INVALID) {
	printf("Invalid proto\n");
    }

    alarm.func = wakeup;
    i = 0;
    while(1) {
	stack_buf.size = 0; 
//	printf("%C ", i);

	attribs[0].value = i++;
	ret = net_event_send(1, &stack_buf, 1, &attribs);
	if(ret)  //ret == 0 if no valid routes
	    mos_led_on(0);
	mos_led_toggle(1);
//	printf("crc  errs: %d ",cc1000_get_crc_errors());
//        printf("sync errs: %d \n",cc1000_get_sync_errors());

	mos_alarm(&alarm, 1, 0);
	
//	timeout_attach(t, wakeup);
	mos_thread_suspend();
	
    }

    printf("End test.\n");
}
