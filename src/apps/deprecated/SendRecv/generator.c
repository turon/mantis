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

/*   file:  generator.c  
 * Written:  Robert Havlik and Brianna Bethel
 *   date:  05/09/04
 */

#include <inttypes.h>
#include <stdio.h>
#include "config.h"
#include "led.h"
#include "msched.h"
#include "com.h"
#include "cc1000_raw.h"

#define NET_FULL_POWER      0xff
#define NET_FREQUENCY       0x03  //FREQ_915_430_MHZ

void generator();

void start(void){
  mos_thread_new(generator, 512, PRIORITY_NORMAL);
}


void generator()
{
  uint8_t value=0;
  uint8_t size = 1;
  uint16_t cycles=0;
  uint8_t frequency = 0;
  uint8_t power = 255;
  uint16_t i,j;
  uint16_t numsamples; 
  uint8_t acknowledge=0;
  uint8_t again = 1;
  uint8_t frequencyindex =0;
  uint8_t testnumber;
  uint8_t max_frequency = 30;

comBuf *inpacket;
comBuf outpacket;

//test number

//change numsamples
    
//change frequency

//change power

cc1000_raw_ioctl(CC1000_TX_POWER, power); 
//com_mode(IFACE_RADIO, IF_LISTEN);
 //com_mode(IFACE_SERIAL, IF_LISTEN);

outpacket.size=1;

while(again==1)
{
  cc1000_raw_ioctl(CC1000_FREQ, frequency);
  
  //com_ioctl(IFACE_RADIO, outpacket);
  
 cycles=0;
 
 while(cycles<numsamples) 
   {
     value = value + 11;
     //value = 170 //10101010 use this to test for bit errors
     for(i = 0; i<0xff; i++); //Pause to give receiver time to process, this is to reduce non-radio related errors
     mos_led_display(value);
     
     cycles = cycles +1;  
     outpacket.data[0] = value;
     com_send(IFACE_RADIO, &outpacket);
   }

 
 if(testnumber==1)
   {
     value = 97;
      for(i=0; i<30; i++)
	{ 
	  outpacket.size = 2;
	  outpacket.data[0] = value;
	  outpacket.data[1] = value;
          for(i=0; i<0xf;i++);
	  com_send(IFACE_RADIO, &outpacket);
	  com_free_buf(&outpacket);
	}
   }
 

 else if(testnumber==2)
   {
     frequency= frequency+1;
     
     while(acknowledge!=255)  //have not received an acknowledge to change frequency
	{
	  value= frequency;
	  outpacket.data[0] = frequency;
	  outpacket.data[1] = 70; //F character
	  
	  for(i=0; i<5; i++)
	    {   
	      com_send(IFACE_RADIO, &outpacket);
	      com_free_buf(&outpacket);
	      for(j=0;j<0xff;j++);
	      mos_led_display(value);
	    }

	  for(i=0;i<0xff;i++);

	  inpacket = com_recv(IFACE_RADIO);
	  acknowledge = inpacket->data[0];
	  mos_led_display(acknowledge);
	}

      cycles = 0;
      acknowledge=0;
	
      if(frequency== max_frequency+1)
	{
	  again=0;
	}
      
      //wait a sec before sending again
      for(i=1;i<15;i++)
	{
	    for(j=0;j<0x1fff;j++);
	    mos_led_display(5);
	    for(j=0;j<0x1fff;j++);
	    mos_led_display(2);
      }
      

   }
    
 else
   {
    again=0;
   }

}

}
