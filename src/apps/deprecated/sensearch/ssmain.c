#include "mos.h"
#include "mica2-gps.h"
#include "com.h"
#include "dev.h"
#include "led.h"
#include "sstypes.h"
#include "printf.h"
#include "node_id.h"
#include "mutex.h"
#include "sem.h"
#include "clock.h"



////////////////////////////////////////////////////////////////
// Constant Definitions                                       //
////////////////////////////////////////////////////////////////
// the ID of this node.                                       //
#define MY_ID 3                                               //
// the maximum number of nodes that we'll be tracking.        //
// no node can have an ID greater than MAX_NODES - 1          //
#define MAX_NODES 10                                          //
// the number of entries in the database.                     //
#define ENTRY_COUNT 50                                        //
// the maximum number of entries to store for each node       // 
#define MAX_RECORDS  ((ENTRY_COUNT) / (MAX_NODES))            //
// returned if no empty slot is found for the specified id    //
// from db_find_empty()                                       //
#define DB_NO_EMPTY_SLOT 0xFF                                 //
////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////
// Global Variables                                           //
////////////////////////////////////////////////////////////////
// the database stores the location data for our node         //
// as well as the nodes we've come into contact with.         //
dbentry_t db[ENTRY_COUNT];                                    //
// i have no idea what this is for                            //
boolean is_empty[ENTRY_COUNT];                                //
// for each node's id, db_write_loc[id] is a number, 0-4,     //
// which tells us where in the database we can find their     //
// most recent location                                       //
uint8_t db_write_loc[MAX_NODES];                              //
uint8_t db_redundant[MAX_NODES];                              //
////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////
// Application Threads                                        //
////////////////////////////////////////////////////////////////
void radio_thread(void);                                      //
void gps_thread(void);                                        //
////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////
// Helper Functions                                           //
////////////////////////////////////////////////////////////////
void radio_send_handshake(uint8_t type);                      //
                                                              //
void db_dump(void);                                           //
void db_init(void);                                           //
boolean db_is_redundant(dbentry_t* e);                        //  
void db_store_entry(dbentry_t* e);                            //
void db_store_gga(uint8_t id, const gps_gga_t* gga);          //
uint8_t db_find_empty(uint8_t id);                            //
uint8_t db_find_oldest(uint8_t id);                           //
                                                              //
void handle_beacon(handshake_t* hs);                          //
void handle_query(handshake_t* hs);                           //
void handle_entry(entrypkt_t* ent);                           //
void handle_packet(comBuf* pkt);                              //
                                                              //
boolean timestamp_isequal(timestamp_t a, timestamp_t b);     //
boolean timestamp_isnewer(timestamp_t a, timestamp_t b);     //
boolean timestamp_isolder(timestamp_t a, timestamp_t b);     //
////////////////////////////////////////////////////////////////

mos_sem_t radio_sem;
mos_alarm_t radio_alarm;
mos_alarm_t gps_alarm;


void radio_callback(void* p)
{
   mos_sem_post(&radio_sem);
}


void gps_callback(void* p)
{
   mos_thread_new(gps_thread, 128, PRIORITY_NORMAL);
}


inline boolean timestamp_isequal(timestamp_t a, timestamp_t b)
{
   return ((a.hours == b.hours) &&
	   (a.minutes == b.minutes) &&
	   (a.seconds == b.seconds));
}

boolean timestamp_isnewer(timestamp_t a, timestamp_t b)
{
   if (a.hours > b.hours)
      return true;
   if (a.hours < b.hours)
      return false;


   if (a.minutes > b.minutes)
      return true;
   if (a.minutes < b.minutes)
      return false;

   return (a.seconds > b.seconds);
}

boolean timestamp_isolder(timestamp_t a, timestamp_t b)
{
   if (a.hours < b.hours)
      return true;
   if (a.hours > b.hours)
      return false;

   if (a.minutes < b.minutes)
      return true;
   if (a.minutes > b.minutes)
      return false;

   return (a.seconds < b.seconds);
}



uint8_t db_find_empty(uint8_t id)
{

   uint8_t i;
   for( i = id; i < ENTRY_COUNT; i += 10)
   {   
      if (db[i].src_id == 0)
	 return i;
   }
   
   return DB_NO_EMPTY_SLOT;
}

uint8_t db_find_oldest(uint8_t id)
{
   uint8_t oldest = id;
   uint8_t i;

   // loop through each entry for 'id' if it is older than the first entry,
   // set oldest to point to that entry.
   for(i = id ; i < ENTRY_COUNT; i += 10)
   {
      if (timestamp_isolder(db[i].rec_time, db[oldest].rec_time))
	 oldest = i;
   }

   return oldest;
   
}

                                     

void db_dump(void)
{
   comBuf pkt;
   pkt.size = sizeof(entrypkt_t);
   entrypkt_t* e = (entrypkt_t*)(&pkt.data);
   
   uint8_t i;
   for(i = 0 ; i < ENTRY_COUNT; ++i)
   {
      if (db[i].src_id != 0 && db[i].hop_count < 5)
      {
	 e->msg_type = MSG_DATABASE_ENTRY;
	 e->entry = db[i];
	 com_send(IFACE_RADIO, &pkt);
	 mos_thread_sleep(50);
      }
   }
}




void handle_beacon(handshake_t* hs)
{  
   db_dump();
   radio_send_handshake(MSG_HANDSHAKE_QUERY);
}

void handle_query(handshake_t* hs)
{
   db_dump();
   radio_send_handshake(MSG_HANDSHAKE_END);
}

void handle_entry(entrypkt_t* ent)
{
   printf("Radio: got entry.\n");
   
   printf("  > src_id: %C\n",
	  ent->entry.src_id);
   printf("  > lat   : %d deg. %d.%d min\n",
	  ent->entry.lat.degrees,
	  ent->entry.lat.minutes.whole,
	  ent->entry.lat.minutes.decimal);
   printf("  > lon   : %d deg. %d.%d min\n",
	  ent->entry.lon.degrees,
	  ent->entry.lon.minutes.whole,
	  ent->entry.lon.minutes.decimal);

   printf("  > utc   : %02d:%02d:%02d\n",
	  ent->entry.rec_time.hours,
	  ent->entry.rec_time.minutes,
	  ent->entry.rec_time.seconds);
   printf("  > hops  : %d\n",
	  ent->entry.hop_count);
   
}


void handle_packet(comBuf* pkt)
{
   uint8_t msg_id = pkt->data[0];
   
   switch(msg_id)
   {
   case MSG_HANDSHAKE_BEACON:
   {
      printf("  > Received MSG_HANDSHAKE_BEACON\n");
      
      handshake_t* h = (handshake_t*)(&pkt->data);
      
      if (h->src_id != MY_ID)
	 handle_beacon(h);
      break;
   }
   case MSG_HANDSHAKE_QUERY:
   {
      printf("  > Received MSG_HANDSHAKE_QUERY\n");
      
      handshake_t* h = (handshake_t*)(&pkt->data);
      
      if (h->src_id != MY_ID)
	 handle_query(h);
      break;
   }
   case MSG_HANDSHAKE_END:
   {
      printf("  > Received MSG_HANDSHAKE_END\n");
      break;
   }
   case MSG_DATABASE_ENTRY:
   {
      printf("  > Received MSG_DATABASE_ENTRY\n");
      
      entrypkt_t* e = (entrypkt_t*)(&pkt->data);
      handle_entry(e);
      break;
   }
   case MSG_BASESTATION:
      break;
   default:
      printf("  > Received Unknown Message ID %x\n", (uint16_t)msg_id);
      break;
   }
   
}



void db_init(void)
{
   uint8_t i;

   for(i = 0 ; i < MAX_NODES; ++i)
      db_write_loc[i] = 0;

}

// this function is called when the node updates is GPS location
void db_store_gga(uint8_t id, const gps_gga_t* gga)
{
   uint8_t i = id + (db_write_loc[id] * 10);
   
   db[i].lat = gga->latitude;
   db[i].lon = gga->longitude;
   db[i].rec_time = gga->utc;
   db[i].src_id = id;
   db[i].hop_count = 0;
   
   //is_empty[i] = id;
   
   if (++db_write_loc[id] == MAX_RECORDS)
      db_write_loc[id] = 0;
}


boolean db_is_redundant(dbentry_t* e)
{
   uint8_t i;

   for(i = 0; i < ENTRY_COUNT; i += 10)
   {
      // if the timestamps on the two enttries are the same,
      // the new record is redundant
      if (timestamp_isequal(db[e->src_id + i].rec_time, e->rec_time))
      {
	 db_redundant[e->src_id]++;
	 return true;
      }
   }
   
   return false;
}



void db_store_entry(dbentry_t* e)
{
   uint8_t empty_slot = db_find_empty(e->src_id);

   // if we found an empty slot, store the entry there
   if (empty_slot != DB_NO_EMPTY_SLOT)
      db[empty_slot] = *e;
   else
   {
      // otherwise, find the oldest slot for this source id
      uint8_t oldest_slot = db_find_oldest(e->src_id);

      // if the new entry has a newer timestamp than our oldest record,
      // replace it with the new version
      if (timestamp_isnewer(e->rec_time, db[oldest_slot].rec_time))
      {
	 // overwrite the entry at 'oldest'
	 db[oldest_slot] = *e;
      }
   }
   
   
}



void radio_send_handshake(uint8_t type)
{
   mos_led_toggle(1);
   comBuf p;
   p.size = sizeof(handshake_t);
   handshake_t* h = (handshake_t*)(&p.data);

   h->msg_type = type;
   h->src_id = MY_ID;
  
   com_send(IFACE_RADIO, &p);
}

void start(void)
{
   mos_node_id_set(MY_ID);
   
   db_init();
   
   dev_mode(DEV_MICA2_GPS, DEV_MODE_ON);
   dev_ioctl(DEV_MICA2_GPS, MICA2_GPS_READ_GGA);
   
   com_mode(IFACE_RADIO, IF_LISTEN);

   printf("Application Start\n");

   radio_alarm.func = radio_callback;
   radio_alarm.msecs = 3000;
   radio_alarm.reset_to = 3000;

   gps_alarm.func = gps_callback;
   gps_alarm.msecs = 5000;
   gps_alarm.reset_to  = 60000;
   
   mos_sem_init(&radio_sem, 0);
   mos_alarm(&radio_alarm);
   mos_alarm(&gps_alarm);
   
   mos_thread_new(radio_thread, 128, PRIORITY_NORMAL);
   
}


void gps_thread(void)
{
   
   gps_gga_t gga;

   static boolean first_time = true;
   

   boolean stop_locating = false;
   uint8_t gps_read_times = 0;

      
   printf("GPS: Looking for coordinates.\n");
      
   while(!stop_locating)
   {
      //handle_t h = mos_disable_ints();
      dev_read(DEV_MICA2_GPS, &gga, sizeof(gga));
	 
      ++gps_read_times;
      //mos_enable_ints(h);
      mos_led_toggle(1);
	 
	 
      printf("  > [%02d] %d satellites @ %02C:%02C:%02C\n",
	     gps_read_times,
	     gga.satellite_count,
	     gga.utc.hours,
	     gga.utc.minutes,
	     gga.utc.seconds);
	 
      // if we have enough satellites to get good coordinates
      if (gga.satellite_count > 3)
      {
	 printf("  > Found 4+ Satellites.\n");
	    
	 mica2_gps_print_gga(&gga);
	    
	 // store coordinates
	 db_store_gga(MY_ID, &gga);
	    
	 //exit loop
	 stop_locating = true;
	 first_time = false;
      }
      // if we have tried 60 times and not gotten a fix,
      // give up.
      else if (!first_time && gps_read_times >= 30)
      {
	 printf("  > Giving up after 30 reads.\n");
	 stop_locating = true;
      }
      else if (first_time && gps_read_times >= 50)
      {
	 printf("  > Giving up after 150 reads.\n");
	 stop_locating = true;
      }
   }

   // }
}



void radio_thread(void)
{
   printf("Application Thread running.\n");
   uint8_t gps_counter = 0;
   
   while(1)
   {

      mos_sem_wait(&radio_sem);

/*       if (gps_counter == 0) */
/* 	 gps_thread(); */

/*       if (++gps_counter == 3) */
/* 	 gps_counter = 0; */

      printf("Radio: Sending Handshake Beacon.\n");
      radio_send_handshake(MSG_HANDSHAKE_BEACON);

      boolean stop_listening = false;

      while(!stop_listening)
      {
	 printf("  > Listening for a response...");
	 comBuf* pkt = com_recv_timed(IFACE_RADIO, 3000);
	 
	 if (pkt != NULL)
	 {
	    printf("got a response.\n");
	    handle_packet(pkt);
	    com_free_buf(pkt);
	 }
	 else
	 {
	    printf("no response.\n");
	    stop_listening = true;
	 }
	 
      }

      printf("Radio: done listening.\n");
      
   }
}
