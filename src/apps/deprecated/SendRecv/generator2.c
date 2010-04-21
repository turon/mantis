//generator.c Robert Havlik and Brianna Bethel

#include <inttypes.h>
#include <stdio.h>
#include "config.h"
#include "led.h"
#include "msched.h"
//#include "adc.h"
#include "com.h"
#include "cc1000_raw.h"

//testing
//#include <stdlib.h>
//#include <stdarg.h>
//#include <config.h>

/* some defines for our hard-coded network... */
#define NET_FULL_POWER      0xff
//#define NET_LOCAL_ADDRESS   0x01
//#define NET_FREQUENCY       0x03  //FREQ_915_430_MHZ //used to be 0x03
//#define NET_REMOTE_ADDRESS  0x04
//#define NET_PORT            0x02

void generator();


void start(void){
  //flooding_init(NET_FREQUENCY);
  //mos_enable_pwr_mgt();
  //mos_set_addr(NET_LOCAL_ADDRESS);
  //mos_set_net_power(NET_FULL_POWER);
  //mos_thread_new(mos_net_daemon, 128, PRIORITY_NORMAL);
  mos_thread_new(generator, 512, PRIORITY_NORMAL);
}

void generator(){

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
  uint8_t max_frequency = 30;//30;
  //Packet data;

  //declarations for new com layer
comBuf *inpacket;
comBuf outpacket;

//test number
testnumber=1;
//change numsamples
numsamples=2000;    
//change frequency
frequency=3;
//change power
power=255;
//probably need to change
//rf_set_power(power);
//probably need to change

cc1000_raw_ioctl(CC1000_TX_POWER, power); 
//com_mode(IFACE_RADIO, IF_LISTEN);
 //com_mode(IFACE_SERIAL, IF_LISTEN);

outpacket.size=1;

while(again==1)
{

  //rf_set_freq(frequency); //original
  cc1000_raw_ioctl(CC1000_FREQ, frequency);//will probably need to change evenutally to not use the raw
  
  //com_ioctl(IFACE_RADIO, outpacket);
  //need to figure out command to set frequency
  
 cycles=0;
 
 while(cycles<numsamples) 
   // while(1)   
   {
     value = value + 11;
     //value = 170 //10101010 use this to test for bit errors  
      //mos_thread_sleep(70);
     for(i = 0; i<0xff; i++); //try to make pause for calculation faster
     mos_led_display(value);
     
     cycles = cycles +1;  
     
     outpacket.data[0] = value;
     com_send(IFACE_RADIO, &outpacket);
      
   }
 
 if(testnumber==1)
   {
     value = 97;
      for(i=0; i<100; i++)
	{ 
	  //mos_led_display(value);
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
     //This is a change to test
     frequency= frequency+1;
     
     while(acknowledge!=255)//have not recieved anything
	{
	  
	  value= frequency;
	  outpacket.data[0] = frequency;
	  
	  for(i=0; i<5; i++)
	    {   
	      //flooding_sendto(NET_REMOTE_ADDRESS, NET_PORT, &value, 0x01);//original
	      com_send(IFACE_RADIO, &outpacket);
	      com_free_buf(&outpacket);//may not need this since sending the same packet
	      for(j=0;j<0x1fff;j++);
	      mos_led_display(value);
	    }

	  for(i=0;i<0x1fff;i++);//maybe shorthen this
/*     
/* 	  size = flooding_recv((char *)&data, NET_PORT); */
	  
/* 	  for(i=0; i<size; i++) */
/* 	    { */
/* 	      acknowledge = ((char*)&data)[i]; */
/* 	    } */
	  //original

	  inpacket = com_recv(IFACE_RADIO);
	  acknowledge = inpacket->data[0];//just make inpacket condition if this works
	  mos_led_display(acknowledge);
	}

      cycles = 0;
      acknowledge=0;
	
      if(frequency== max_frequency+1)
	{
	  //again=0;
	}
      
      //wait a sec before sending again
      for(i=1;i<15;i++){
	    for(j=0;j<0x1fff;j++);
	    mos_led_display(5);
	    for(j=0;j<0x1fff;j++);
	    mos_led_display(2);
      }
      
    }
    
  //else
    //again=0;
}
}
