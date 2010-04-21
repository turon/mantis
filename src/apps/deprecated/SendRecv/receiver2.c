//receiver.c  robert havlik

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
  uint8_t value3;
  uint8_t n=0;
  uint8_t receiving =1;
  uint8_t endfrequency =31;//31;

  //declarations for new com layer
comBuf *packet;
comBuf outpacket;
comBuf *outpacket2;
//test number
testnumber=1;   
//change frequency
frequency=3;
//rf_set_freq(frequency);original
 cc1000_raw_ioctl(CC1000_FREQ, frequency);

//change power
power=255; 
 //rf_set_power(power);//origional
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

    mos_led_display(value1);
    com_free_buf(packet);
    
    outpacket.data[0] = value1;
    
    outpacket.size = 2;

    com_send(IFACE_SERIAL, &outpacket);
    com_free_buf(&outpacket);
  }

   
return;
}





