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
#include "clock.h"
#include "net.h"
#include "cc1000.h"

#include "dvdrp_test.h"

#include "mem.h"

void m_thread();

Thread *recv_thread;
extern Thread *net_t;

Thread *stk_chk_t;
mos_alarm_t alarm;

void wakeup(void *n_data) {
    mos_thread_resume(stk_chk_t);

    mos_alarm(&alarm, 5, 0);
}

void stack_check() {
    stk_chk_t = mos_thread_current();
    
    alarm.func = wakeup;
    mos_alarm(&alarm, 1, 0);
	
    while(1) {
	printf("rc: %d\n", mos_check_stack(recv_thread));
//	printf("nt: %d\n", mos_check_stack(net_t));
	printf("ts: %d\n", mos_check_stack(stk_chk_t));

	mos_thread_suspend();
    }
}

void start(void) {
    //give us control over serial/rf
    //mos_thread_new(mos_command_daemon, 128, PRIORITY_NORMAL);
    //start this thread
    mos_thread_new(m_thread, 224, PRIORITY_NORMAL);
//    mos_thread_new(net_thread, 224, PRIORITY_NORMAL);
    mos_thread_new(net_thread, 192, PRIORITY_NORMAL);
//    mos_thread_new(stack_check, 128, PRIORITY_NORMAL);
}

int8_t recv(uint8_t event_id, comBuf *pkt, uint8_t proto, uint8_t *footer) {
    dvdrp_mesg_pkt *hdr = (dvdrp_mesg_pkt *)footer;

    mos_led_toggle(0);
    
    printf(".d;%d(htl=%C)", hdr->attributes[0].value, hdr->header.htl);
    
    return 0;
}

void m_thread() {
    dvdrp_filter_list *pred;
    int8_t ret;

    printf("Hello from DV/DRP test recv!\n");

    net_init();
    dvdrp_init();

    //turn down the power to make it easy to test multihop
    com_ioctl(IFACE_RADIO, CC1000_TX_POWER, 1);

    recv_thread = mos_thread_current();
    printf(" !! %x\n", recv_thread);
    
    ret = net_proto_set(DVDRP_PROTO_ID);
    if(ret == NET_PROTO_INVALID) {
	printf("Invalid proto\n");
    }

    net_event_register(1, recv);

    pred = pool_malloc(&filter_pool);
    pred->constraints = pool_malloc(&constraint_pool);
    pred->constraints->next = NULL;
    pred->constraints->name = VELOCITY;
    pred->constraints->value = 0;
    pred->constraints->compare_type = DVDRP_COMPARE_GT;
    pred->next = NULL;

    net_ioctl(DVDRP_PROTO_ID, DVDRP_IOCTL_SETPRED, pred, 0);

    mos_led_toggle(2);
    
    printf("Waiting for mesgs.\n");

    mos_thread_suspend();
}
