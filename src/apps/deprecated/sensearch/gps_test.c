//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <inttypes.h>
#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "mica2-gps.h"
#include "uart.h"
#include "dev.h"
#include "avr-i2c.h"
#include "com.h"
#include "sem.h"

// this structure represents a GGA (gps fix data) packet.
typedef struct dbentry {
   uint8_t	srcid;	  //1 byte
   latlong_t	lat, lon; //5 bytes each, 10 total
   timestamp_t	rectime;  //3 bytes
   uint8_t	hop_cnt;  //1 byte
   uint8_t	hops[5];
} dbentry_t; //15 bytes

typedef struct handshake {
   uint8_t	msg_type; //0 for beacon, 1 for reply 
   uint8_t	srcid;
   boolean	isempty[50];
} handshake_t;

typedef struct entrypkt {
   uint8_t	msg_type; //2 for entry packet
   dbentry_t	entry;
} entrypkt_t;

gps_gga_t gga;
//gpstime_t gpstime;
dbentry_t recdb[50];
boolean	  isempty[50];
uint16_t  myID = 9;
uint8_t	  dbWriteHash[10];
uint8_t  redundantRecv[10]; 
uint8_t   emptySlot, oldestSlot;
// this will hold the raw string we receive from the GPS.
char gps_buffer[96], checksum;
static comBuf send_pkt;
static boolean notTheFirstTime = 0;
comBuf* recvBuf;
entrypkt_t data_entry;

uint8_t	findEmptySlot(uint8_t srcid);
uint8_t	findOldestSlot(uint8_t srcid);
uint16_t acqTimeDiff(uint8_t m1, uint8_t s1, uint8_t m2, uint8_t s2);
boolean second_time_newer(timestamp_t t1, timestamp_t t2);
boolean time_identical(timestamp_t t1, timestamp_t t2);
boolean redundantRec(uint8_t srcid, timestamp_t t);

mos_alarm_t GPStimer, RADIOtimer;
mos_sem_t GPSsem, Radiosem;
void GPStimer_callback(void *p);
void RADIOtimer_callback(void *p);
void GPSThread();
void RadioThread();

/************************************************************
 * IMPORTANT: You may need to make changes in other files
 *   for the GPS to work.
 *
 * in src/mos/com/uart.c, line 266 ( uart_init() ):
 *    change 'uart_set_baud(UART1, DEFAULT_BAUD_RATE);'
 *    to 'uart_set_baud(UART1, B4800);'
 *
 * this is currently the only change you need to make.
 ************************************************************/
void watcherThread(void)
{
   while(1)
   {
      print_clock_list();
      mos_thread_sleep(100);
   }
   
}


void start(void)
{
   uint8_t i;
   mos_sem_init(&GPSsem, 0);
   mos_sem_init(&Radiosem, 0);
   mos_node_id_set(myID);
      
   for (i=0; i < 20; i++) dbWriteHash[i] = 0;
   for (i=0; i < 50; i++) isempty[i] = 1;
   gps_on();  
   gps_disable_bits(GPS_RX_ADDR);
   gps_enable_bits(GPS_TX_ADDR);

   com_mode(IFACE_RADIO, IF_LISTEN);
   GPStimer.func = GPStimer_callback;
   GPStimer.msecs = 10000;
   GPStimer.reset_to = 10000;
   RADIOtimer.func = RADIOtimer_callback;
   RADIOtimer.msecs = 3000;
   RADIOtimer.reset_to = 3000;
   

   
 
   if (mos_thread_new(GPSThread, 128, PRIORITY_NORMAL) != THREAD_OK)
      printf("failed to create GPSThread\n");
   
  if (mos_thread_new(RadioThread, 128, PRIORITY_NORMAL) != THREAD_OK)
      printf("failed to create RadioThread\n");

     
   printf("&GPStimer: 0x%x\n", &GPStimer);
   printf("&RADIOtimer: 0x%x\n", &RADIOtimer);
   

   mos_alarm(&RADIOtimer);
   mos_alarm(&GPStimer);
  
}


mos_thread_t* tptr = NULL;
uint16_t _stack_p = 0, _stack_b = 0;


void RADIOtimer_callback(void *p)
{
  mos_sem_post(&Radiosem);
}

void RadioThread()
{
   uint16_t timeout;
   handshake_t handshake;
   uint8_t i, looptime;
   boolean stopListening = 0;

   looptime = 0;
   timeout = 3000;  
  
   while (1) {
      printf("wait for Radio sem here\n"); 
      mos_sem_wait(&Radiosem);
      

      for (i=0; i < 50; i++) {
      printf("%d lat %d.%d lon %d.%d rectime %02d:%02d:%02d hop_cnt %d ",\
      recdb[i].srcid, recdb[i].lat.degrees, recdb[i].lat.minutes,\
      recdb[i].lon.degrees, recdb[i].lon.minutes, recdb[i].rectime.hours,\
      recdb[i].rectime.minutes, recdb[i].rectime.seconds, recdb[i].hop_cnt);
      printf("hops %d %d %d %d %d\n",\
      recdb[i].hops[0],recdb[i].hops[1],recdb[i].hops[2],recdb[i].hops[3],recdb[i].hops[4]);
      }
      
      
      printf("isempty array ");
      for (i=0; i < 50; i++)  printf("%d ",recdb[i].srcid);
      printf("\n");
      
      //send a type 0 beacon
      timeout = 0;
      data_entry.msg_type = 0;
      handshake.msg_type = 0;
      data_entry.entry.srcid = myID;
      handshake.srcid = myID;

//      memcpy(&handshake.isempty, &isempty, 50*sizeof(boolean));
      memcpy(&data_entry.entry, &handshake, sizeof(handshake_t));
      memcpy(&send_pkt.data, &data_entry, sizeof(entrypkt_t));
      send_pkt.size = sizeof(entrypkt_t);
      mos_thread_sleep(srand(myID));
      com_send(IFACE_RADIO, &send_pkt);
      //mos_led_toggle(0);
      stopListening = 0;
	 
      while (!stopListening) {
	 if (timeout == 0) { 
	    timeout = 3000; //reset timeout
	 }

	 //recvBuf = com_recv_timed(IFACE_RADIO, timeout);
	 if (recvBuf == NULL) {
	    stopListening = 1;
	 } //end of if recvBuf NULL
	 else { //got beacon or db from another node
	    memcpy(&data_entry, recvBuf->data, sizeof(entrypkt_t));
	    com_free_buf(recvBuf);
	    if (data_entry.msg_type == 0) { //handshake
	       memcpy(&handshake, &data_entry.entry, sizeof(handshake_t));
	       if (handshake.srcid != myID) { //not my own beacon
		  if (handshake.msg_type == 0) { //Got type 0 beacon from some node
		     printf("Recive beacon type 0, timeout %d, looptime %d\n",timeout,looptime);           
		     //mos_led_toggle(1);
		     for (i=0; i < 50; i++) {
			if (recdb[i].srcid != 0) {
			   if (recdb[i].hop_cnt < 5) {
			      data_entry.msg_type = 2;
			      memcpy(&data_entry.entry, &recdb[i], sizeof(dbentry_t));
			      memcpy(&send_pkt.data, &data_entry, sizeof(entrypkt_t));
			      send_pkt.size = sizeof(entrypkt_t);
			      com_send(IFACE_RADIO, &send_pkt);
			      mos_thread_sleep(50); //short pause to avoid crash
			   } //end of if hot_cnt < 5
			} //end of if is not empty
		     } //end of for loop
        
		     data_entry.msg_type = 1; //This is for sending redundant array
		     //to base station, not for db exchange
		     data_entry.entry.srcid = myID;                                 
		     redundantRecv[0] = myID;
		     printf("redundantRecv ");
		     for (i=0; i < 10; i++) printf("%d ",redundantRecv[i]);
		     printf("\n");        
		     memcpy(&data_entry.entry, redundantRecv, 10*sizeof(uint8_t));
		     memcpy(&send_pkt.data, &data_entry, sizeof(entrypkt_t));
		     send_pkt.size = sizeof(entrypkt_t);
		     com_send(IFACE_RADIO, &send_pkt);
		     // end sending redundant array to base
		     mos_thread_sleep(50);
		     // send a type 1 beacon to request beacon sender to send me db
		     data_entry.msg_type = 0;
		     handshake.msg_type = 1;
		     data_entry.entry.srcid = myID;        
		     handshake.srcid = myID;

//        memcpy(&handshake.isempty, &isempty, 50*sizeof(boolean));
		     memcpy(&data_entry.entry, &handshake, sizeof(handshake_t));
		     memcpy(&send_pkt.data, &data_entry, sizeof(entrypkt_t));
		     send_pkt.size = sizeof(entrypkt_t);
		     com_send(IFACE_RADIO, &send_pkt);
		     looptime++;
		     //mos_led_toggle(0);
		  } //end of processing beacon, dump db and send type 1 beacon
		  if (handshake.msg_type == 1) { //Got type 1 beacon, my turn to dump db
		     printf("Recive beacon type 1, timeout %d, looptime %d\n",timeout,looptime);           
		     //mos_led_toggle(2);
		     for (i=0; i < 50; i++) {
			if (recdb[i].srcid != 0) {
			   if (recdb[i].hop_cnt < 5) {
			      data_entry.msg_type = 2;
			      memcpy(&data_entry.entry, &recdb[i], sizeof(dbentry_t));
			      memcpy(&send_pkt.data, &data_entry, sizeof(entrypkt_t));
			      send_pkt.size = sizeof(entrypkt_t);
			      com_send(IFACE_RADIO, &send_pkt);
			      mos_thread_sleep(50); //short pause for avoiding crash
			   } //end of if hot_cnt < 5
			} //end of if is not empty
		     } //end of for loop
		     //we don't send another beacon because we're the second one to dump db
		     data_entry.entry.srcid = myID;        
		     looptime++;

		     mos_thread_sleep(100);
		     data_entry.msg_type = 3;
		     data_entry.entry.srcid = myID;        
		     memcpy(&send_pkt.data, &data_entry, sizeof(entrypkt_t));
		     send_pkt.size = sizeof(entrypkt_t);
		     com_send(IFACE_RADIO, &send_pkt);
        
		  } //end of processing type 1 beacon
	       } //end of not my own beacon 
	    }//end handshake, data_entry.msg_type == 0     
	    if (data_entry.msg_type == 2) { //Got a record entry
	       if ((data_entry.entry.srcid == myID) || (data_entry.entry.srcid == 0)){ continue; }
	       if (redundantRec(data_entry.entry.srcid, data_entry.entry.rectime)) { 
		  printf("Receive redundant record from %d\n", data_entry.entry.srcid);
		  continue; }
	       printf("myID : %d, receive new db entry from %d\n",myID, data_entry.entry.srcid);
	       emptySlot = findEmptySlot(data_entry.entry.srcid);
	       printf("emptySlot : %d\n",emptySlot);       
	       if (emptySlot != 255) { //empty slot found. 255 stands for no empty slot found
		  memcpy(&recdb[data_entry.entry.srcid+emptySlot*10], &data_entry.entry, sizeof(dbentry_t));
		  recdb[data_entry.entry.srcid+emptySlot*10].hops[data_entry.entry.hop_cnt] = myID;
		  recdb[data_entry.entry.srcid+emptySlot*10].hop_cnt = data_entry.entry.hop_cnt+1;
		  if ((data_entry.entry.hop_cnt+1) != 5)
		     recdb[data_entry.entry.srcid+emptySlot*10].hops[data_entry.entry.hop_cnt+1] = 255;         
		  printf("write into %d\n",data_entry.entry.srcid+emptySlot*10);           
		  isempty[data_entry.entry.srcid+emptySlot*10] = 0;
		  for (i=0;i<50;i++) printf("%d ",recdb[i].srcid);
		  printf("\n");         
	       } //end of empty slot found
	       else { //no empty slot found, find the oldest one instead
		  oldestSlot = findOldestSlot(data_entry.entry.srcid);
		  printf("oldestSlot : %d\n",oldestSlot);
		  if (second_time_newer(recdb[data_entry.entry.srcid+oldestSlot*10].rectime, data_entry.entry.rectime)) {
		     memcpy(&recdb[data_entry.entry.srcid+oldestSlot*10], &data_entry.entry, sizeof(dbentry_t));
		     recdb[data_entry.entry.srcid+oldestSlot*10].hops[data_entry.entry.hop_cnt] = myID;
		     recdb[data_entry.entry.srcid+oldestSlot*10].hop_cnt = data_entry.entry.hop_cnt+1;
		     if ((data_entry.entry.hop_cnt+1) != 5)
			recdb[data_entry.entry.srcid+oldestSlot*10].hops[data_entry.entry.hop_cnt] = 255;
//           isempty[data_entry.entry.srcid+oldestSlot*10] = 0;
		  } //end received record is newer, replace it
		  else { //my existing record is newer, don't replace
         
		  }
	       } //end of I don't have this record so try to find a slot for it
	    } //end of processing record entry
	    if (data_entry.msg_type == 3) { //Got a finish sending handshake
	       if ((data_entry.entry.srcid == myID) || (data_entry.entry.srcid == 0))  continue;
	       else  { stopListening = 1;  }
	    } //end of finish sending handshake
	 } //end of beacon or db from another node 
      } //end of while !stopListening
   } //end of while 1

}

uint8_t findEmptySlot(uint8_t srcid)
{
   if (recdb[srcid].srcid == 0) return 0;
   if (recdb[srcid+10].srcid == 0) return 1;
   if (recdb[srcid+20].srcid == 0) return 2;
   if (recdb[srcid+30].srcid == 0) return 3;
   if (recdb[srcid+40].srcid == 0) return 4;
  
   return 255;//no empty spot
}

uint8_t findOldestSlot(uint8_t srcid)
{
   uint8_t o1, o2, o3;
   if (second_time_newer(recdb[data_entry.entry.srcid].rectime,
			 recdb[data_entry.entry.srcid+10].rectime))
      o1 = 0;
   else
      o1 = 1;
   
   if (second_time_newer(recdb[data_entry.entry.srcid+20].rectime,
			 recdb[data_entry.entry.srcid+30].rectime))
      o2 = 2;
   else
      o2 = 3;
  
   if (second_time_newer(recdb[data_entry.entry.srcid+o1*10].rectime,
			 recdb[data_entry.entry.srcid+o2*10].rectime)) 
      o3 = o1;
   else
      o3 = o2;

   if (second_time_newer(recdb[data_entry.entry.srcid+40].rectime,
			 recdb[data_entry.entry.srcid+o3*10].rectime)) 
      return 4;
   else
      return o3;  
}

uint16_t acqTimeDiff(uint8_t m1, uint8_t s1, uint8_t m2, uint8_t s2)
{
   uint16_t rtn;
  
   if (s1 >= s2) {
      rtn = (m1-m2)*60;
      if (s1 > s2) return (rtn+s1-s2);
      if (s1==s2) return rtn;
   }
   else {
      rtn = (m1-1-m2)*60;
      return (rtn+60-s2+s1);
   }
}


boolean second_time_newer(timestamp_t t1, timestamp_t t2)
{
   if (t1.hours < t2.hours) return 1;
   if (t1.hours > t2.hours) return 0;
   //then hours is the same
   if (t1.minutes < t2.minutes) return 1;
   if (t1.minutes > t2.minutes) return 0;
  
   if (t1.seconds < t2.seconds) return 1;
   if (t1.seconds > t2.seconds) return 0;

   return 0;
}

boolean time_identical(timestamp_t t1, timestamp_t t2)
{
   if (t1.hours != t2.hours) return 0;
   if (t1.minutes != t2.minutes) return 0;
   if (t1.seconds != t2.seconds) return 0;
  
   return 1;
}

boolean redundantRec(uint8_t srcid, timestamp_t t)
{

   printf("srcid %d, rectime %02d:%02d:%02d",srcid, t.hours, t.minutes, t.seconds);
   if (time_identical(recdb[srcid].rectime, t)) { redundantRecv[srcid]++; return 1; }
   if (time_identical(recdb[srcid+10].rectime, t)) { redundantRecv[srcid]++; return 1; }
   if (time_identical(recdb[srcid+20].rectime, t)) { redundantRecv[srcid]++; return 1; }
   if (time_identical(recdb[srcid+30].rectime, t)) { redundantRecv[srcid]++; return 1; }
   if (time_identical(recdb[srcid+40].rectime, t)) { redundantRecv[srcid]++; return 1; }
   printf("not iden to any rec\n");
   return 0;

}

void GPStimer_callback(void *p){
    if (tptr != NULL){
      _stack_p = (uint16_t)(tptr->sp);
      _stack_b = (uint16_t)(tptr->stack);
   }
   //mos_led_toggle(1);
   mos_sem_post(&GPSsem);
//   GPStimer.reset_to = 60000;
}

void GPSThread() {

   tptr = mos_thread_current();
   
   uint16_t gpsReadTimes=0;
   boolean stopLocating = 0;
   while (1) {
      printf("In waiting for GPS sem\n");  
      mos_sem_wait(&GPSsem);
      printf("GPS alarm launched\n");   
      stopLocating = 0;
      gpsReadTimes = 0;

      printf("int. stack ptr: %x, stack base: %x\n\n", _stack_p, _stack_b);
 
      
      //com_ioctl_IFACE_RADIO(CC2420_LOW_POWER_MODE);
      while (0) //(!stopLocating)
      {
            
	 mica2_gps_readline(gps_buffer, sizeof(gps_buffer));
//      printf("\n%s\n", gps_buffer);      

	 // if the packet is a GGA (GPS Fix Data) packet
	 if (mica2_gps_is_gga(gps_buffer)) {
        
	    mica2_gps_parse_gga(gps_buffer, &gga);

	    gpsReadTimes++;
	    printf("gpsReadTimes %d\n",gpsReadTimes);        
	    //printf("lla channelno %s, strlen %d\n",lla.channelno, strlen(lla.channelno));
	    if (gga.satellite_count > 3) { //got coordinates
	       mica2_gps_print_gga(&gga);        

	       memcpy(&recdb[myID+dbWriteHash[myID]*10].lat, &gga.latitude, sizeof(latlong_t));
	       memcpy(&recdb[myID+dbWriteHash[myID]*10].lon, &gga.longitude, sizeof(latlong_t));
	       memcpy(&recdb[myID+dbWriteHash[myID]*10].rectime, &gga.utc, sizeof(timestamp_t));
	       recdb[myID+dbWriteHash[myID]*10].srcid = myID;
	       recdb[myID+dbWriteHash[myID]*10].hop_cnt = 0;
	       isempty[myID+dbWriteHash[myID]*10] = myID;
	       dbWriteHash[myID]++;
	       if (dbWriteHash[myID] == 5) dbWriteHash[myID] = 0;
	       notTheFirstTime = 1;
	       stopLocating = 1;             

	    } //end of if satellite number > 3
	    else {
	       if (notTheFirstTime) { //give up after 60 secs
		  if (gpsReadTimes == 60) {
		     //gps_disable_tx_rx();
		     stopLocating = 1;             
		     //cenwits_exchange_db();
		     //gps_enable_tx_rx();
		  }
	       }
	       else { //Cannot get first coordinate after turned on
		  if (gpsReadTimes == 300) { //We don't want to spend more than 300 seconds 
		     //to get first coordinate, go to exchange db mode
		     //gps_disable_tx_rx();
		     stopLocating = 1;
		     //cenwits_exchange_db();
		     //gps_enable_tx_rx();
            
		  }
	       }
	    }
	 } //end of if is_ggs()
      } //end of while !stopLocating
   } //end of while 1
}
      

      
