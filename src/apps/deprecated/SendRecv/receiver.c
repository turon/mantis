/*
  This file is part of MANTIS OS, Operating System for Nymph.
  See http://mantis.cs.colorado.edu/

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

/*   file:  receiver.c  
 * Written:  Robert Havlik and Brianna Bethel
 *   date:  05/09/04
 */

#include <inttypes.h>
#include <stdio.h>
#include "config.h"
#include "led.h"
#include "uart.h"
#include "msched.h"
#include "com.h"
#include "cc1000_raw.h"

#define NET_FULL_POWER      0xff

void receiver();

void start(void){
  mos_thread_new(receiver, 512, PRIORITY_NORMAL);
}

void receiver(){

  uint16_t i, size;
  uint16_t j,k;
  uint8_t new_val, old_val;
  uint8_t power;
  uint8_t frequency;
  uint8_t testnumber;
  uint8_t go=1;
  char dup='d';  //100 in ascii decimal
  char destroy='D';  //68 in ascii decimal
  char newfreqsig ='F';  //70 in ascii decimal
  uint8_t duplicate = 0;
  uint8_t value1;
  uint8_t value2;
  uint8_t value3;
  uint8_t n=0;
  uint8_t receiving =1;
  uint8_t endfrequency =31;

comBuf *packet;
comBuf outpacket;
comBuf *outpacket2;


//test number
   
//change frequency

//rf_set_freq(frequency);original
 cc1000_raw_ioctl(CC1000_FREQ, frequency);

//change power
 

 cc1000_raw_ioctl(CC1000_TX_POWER, power);
 com_mode(IFACE_RADIO, IF_LISTEN);
 com_mode(IFACE_SERIAL, IF_LISTEN);
 
  mos_led_display(1);

  mos_led_display(2);
  
  while(receiving==1){
    
    packet = com_recv(IFACE_RADIO);
    value1 = packet->data[0];

    mos_led_display(value1);
    com_free_buf(packet);
    
    outpacket.data[0] = value1;
    
    outpacket.size = 2;

    com_send(IFACE_SERIAL, &outpacket);
    com_free_buf(&outpacket);
  }

#include <inttypes.h>
#include <stdio.h>
#include "config.h"
#include "led.h"
#include "uart.h"
#include "msched.h"
#include "com.h"
#include "cc1000_raw.h"

/* some defines for our hard-coded network... */


#define NET_FULL_POWER      0xff

void receiver();


void start(void){
  mos_thread_new(receiver, 512, PRIORITY_NORMAL);
}

void receiver(){

  uint16_t i, size;
  uint16_t j,k;
  uint8_t new_val, old_val;
  uint8_t power;
  uint8_t frequency;
  uint8_t testnumber;
  uint8_t go=1;
  char dup='d';  //100 in ascii decimal
  char destroy='D';  //68 in ascii decimal
  char newfreqsig ='F';  //70 in ascii decimal
  uint8_t duplicate = 0;
  uint8_t value1;
  uint8_t value2;
  uint8_t n=0;
  uint8_t receiving =1;
  uint8_t endfrequency =31;

comBuf *packet;
comBuf outpacket;
comBuf *outpacket2;

//test number
   
//change frequency

 cc1000_raw_ioctl(CC1000_FREQ, frequency);

//change power
 

 cc1000_raw_ioctl(CC1000_TX_POWER, power);
 com_mode(IFACE_RADIO, IF_LISTEN);
 com_mode(IFACE_SERIAL, IF_LISTEN);
 
  mos_led_display(1);
  // printf("XXX");
  mos_led_display(2);
  
  //size = flooding_recv((char *)&data, NET_PORT);
  
  while(receiving==1){
    
    packet = com_recv(IFACE_RADIO);
    value1 = packet->data[0];
    value2 = packet->data[0];

    mos_led_display(value1);
    com_free_buf(packet);
    
    outpacket.data[0] = value1;

    outpacket.size = 2;

    if(value2 != 70)
      {
	com_send(IFACE_SERIAL, &outpacket);
	com_free_buf(&outpacket);
      }

    else
     {       
       frequency=value1;
       outpacket.data[0] = 255;//acknowledge signal
       for(i=0; i<15; i++)
	 {
	   com_send(IFACE_RADIO, &outpacket);
	 }
       cc1000_raw_ioctl(CC1000_FREQ, frequency);
       com_free_buf(&outpacket);
       //now tell base autorf program the frequency has changed
       outpacket.data[0]= 97;
       outpacket.data[1]=70;
       outpacket.data[2]= frequency;
       com_send(IFACE_SERIAL, &outpacket);
       com_free_buf(&outpacket);
     }
  
  }

   
return;
}





