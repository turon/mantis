//  This file is part of MOS, the MANTIS Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (c) 2002 - 2007 University of Colorado, Boulder
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//       * Redistributions of source code must retain the above copyright
//         notice, this list of conditions and the following disclaimer.
//       * Redistributions in binary form must reproduce the above
//         copyright notice, this list of conditions and the following
//         disclaimer in the documentation and/or other materials provided
//         with the distribution. 
//       * Neither the name of the MANTIS Project nor the names of its
//         contributors may be used to endorse or promote products derived
//         from this software without specific prior written permission.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//   POSSIBILITY OF SUCH DAMAGE.


#include "mos.h"
#include "mos_debugging.h"
#include "mutex.h"
#include "com.h"
#include "plat_dep.h"
#include "clock.h"
#include "sem.h"
#include "led.h"
#include "printf.h"
#include "dev.h"
#include "com.h"


#ifdef ARCH_AVR
#include "avr-eeprom.h"
#include "avr-adc.h"
#include "atmel-flash.h"
#include "avr-i2c.h"
#include <avr/wdt.h>
#endif

#ifdef PLATFORM_MICA2
#include "cc1000.h"
#include "cc1000_csma.h"
#endif
#ifdef PLATFORM_MICAZ
#include "cc2420.h"
#endif

#include "command_daemon.h"


#ifdef MOS_DEBUG


static uint8_t *trace_cursor;
static uint8_t bit_cursor;

static uint8_t debug_trace[DEBUG_TRACE_SIZE];

//static mos_mutex_t debug_mutex;
static mos_mutex_t debug_lock;
static mos_mutex_t checkpoint_lock;

static debug_checkpoint_t* cp_head;

static boolean debugging;

static uint8_t error_signal;

#define DEBUG_TRACE_FLASH_ADDR 10000


#define enable_debugging() debugging = true;
#define disable_debugging() debugging = false;

#define unlock_debugging() mos_mutex_unlock(&debug_lock);
#define lock_debugging() mos_mutex_lock(&debug_lock);


static inline void write_ptr_to_flash(uint8_t *ptr, uint16_t addr)
{
   uint16_t data[1];
   data[0] = 4;

   //printf ("%d\n", data[0]);  
   
   dev_ioctl (DEV_ATMEL_FLASH, DEV_SEEK, addr);
   dev_write (DEV_ATMEL_FLASH, data, sizeof (data));
}

static inline uint8_t* read_ptr_from_flash(uint16_t addr)
{
   uint16_t data[1];

   dev_ioctl (DEV_ATMEL_FLASH, DEV_SEEK, addr);
   dev_read (DEV_ATMEL_FLASH, data, sizeof (data));

   //printf ("%d\n", data[0]);

   return data[0];
}

void watchdog_reset_thread(void)
{
   adc_init();
   printf_init();
   PLAT_INIT();
   uart_init();
   avr_eeprom_init();
   avr_i2c_init();
   
   atmel_flash_init();
   mos_node_id_init();
   clock_init();
   
#ifdef PLATFORM_MICA2
   cc1000_csma_init();
#endif
   
#ifdef PLATFORM_MICAZ
   cc2420_init();
#endif
   
   trace_cursor = read_ptr_from_flash(DEBUG_TRACE_FLASH_ADDR);
   mos_led_on(1);
   //printf("new trace cursor is %d\n", trace_cursor);

//   printf("\n***** *****\n");
   //printf("watchdog reset, scheduler must have stopped running\n");
//   printf("TODO: fix flash driver so we can actually save the pointer to our trace\n");
   
   //just while(1) for now
   while(1);
   mos_debug_error_report(SIGNAL_WATCHDOG_EXPIRED);
   
}

void mos_debug_init()
{
   uint16_t i;

   // init the cursor to the start of the trace
   trace_cursor = &debug_trace[DEBUG_TRACE_START];
   bit_cursor = 0;

   mos_mutex_init(&debug_lock);
   mos_mutex_init(&checkpoint_lock);

   // init the debug vector to 0
   for(i=0; i<DEBUG_TRACE_SIZE; i++)
      debug_trace[i] = 0;

   cp_head = NULL;

}



void mos_debug_post_init()
{

   if(MCUCSR & (1 << WDRF))
   {
      // if we have watchdog reset, start a debugging thread in place of start, but start the scheduler normally once that's finished

      //zero the watchdog reset flag
      MCUCSR &= ~(1 << WDRF);
      
      mos_thread_new(watchdog_reset_thread, 128, PRIORITY_HIGH);

      // starts the idle thread
      mos_sched_start();

      
   }

   //mos_thread_new(mos_debug_error_report, 128, PRIORITY_HIGH);
   
//   printf("writing addr %d to flash\n", trace_cursor);
   
   write_ptr_to_flash(trace_cursor, DEBUG_TRACE_FLASH_ADDR);
      
   //printf("read %d\n", read_ptr_from_flash(DEBUG_TRACE_FLASH_ADDR));
   
   //data = &debug_trace[DEBUG_TRACE_START];

   enable_debugging();

   wdt_enable(WDTO_2S);

}


boolean mos_debug_enabled()
{
   return debugging;
}

void mos_debug_register_checkpoint(debug_checkpoint_t *cp, uint32_t ms)
{
   debug_checkpoint_t* curr;

   mos_mutex_lock(&checkpoint_lock);

   // setting the timeout to 2x the expected time
   cp->timeout = ms*2;
   cp->next = NULL;
   cp->last_timestamp = 0;

   curr = cp_head;
   if(curr == NULL)
   {
      cp_head = cp;
      mos_mutex_unlock(&checkpoint_lock);
      return;
   }

   while(curr->next != NULL)
   {
      if(curr == cp)
      {
	 //printf("error, checkpoint already registered\n");
	 mos_mutex_unlock(&checkpoint_lock);
	 return;
      }
      curr = curr->next;
   }
   
   curr->next = cp;
   //printf("checkpoint registered\n");
   
   mos_mutex_unlock(&checkpoint_lock);
}

void mos_debug_checkpoint_reached(debug_checkpoint_t *cp)
{
   debug_checkpoint_t* curr;

   mos_mutex_lock(&checkpoint_lock);
 
   curr = cp_head;
   while(curr != NULL)
   {
      if(curr == cp)
      {
	 curr->last_timestamp = mos_get_realtime();
         //printf("real time is %d\n", curr->last_timestamp);
	 mos_mutex_unlock(&checkpoint_lock);
	 return;
      }
      curr=curr->next;
   }

   // if we are here, we never found the thread
   //printf("error, thread id not found in checkpoint list\n");
   mos_mutex_unlock(&checkpoint_lock);
   

}

void mos_debug_status_check()
{
   debug_checkpoint_t* curr;
   uint32_t realtime;
   //printf("1\n");

   //called from ints disabled context,
   //no potential for race condition

   disable_debugging(); 

   wdt_reset();
   
   //printf("2\n");
   curr = cp_head;

   realtime = mos_get_realtime();
   while(curr != NULL)
   {
      // check whether we have exceeded the timeout threads need to
      // have reached their first checkpoint before the check will
      // work
      //printf("3\n");
      if(curr->last_timestamp != 0 &&
	 realtime - curr->last_timestamp > curr->timeout)
      {
	 wdt_disable();
	 //mos_mutex_unlock(&checkpoint_lock);
	 mos_debug_error_report(SIGNAL_DEADLOCK);
      }
      curr = curr->next;
   }
   
   //printf("4\n");

   // if we're here, all checks are OK
   enable_debugging();
}

	 
void mos_debug_set_trace(uint8_t code)
{
   if(debugging)
   {
      lock_debugging();
//printf("set trace %d at %d, bits %d\n", code, trace_cursor, bit_cursor);

      *trace_cursor |= (code << bit_cursor);
      
      bit_cursor += DBCODE_SIZE;
      if(bit_cursor > 7)
      {
	 //check if we need to cycle around
	 if(trace_cursor == &debug_trace[DEBUG_TRACE_END])
	    trace_cursor = &debug_trace[DEBUG_TRACE_START];
	 else
	    trace_cursor++;
	 
	 bit_cursor = 0;
      }
      *trace_cursor &= ~(LOWER_BIT_MASK << bit_cursor);
      unlock_debugging();
   }
}

void mos_debug_clear_trace()
{
   // doesn't work properly
   
   lock_debugging();
   uint8_t i;
   trace_cursor = &debug_trace[DEBUG_TRACE_START];
   bit_cursor = 0;

   for(i=0; i<DEBUG_TRACE_SIZE; i++)
      debug_trace[i] = 0;
   unlock_debugging();
}

static void mos_debug_print_trace()
{
   uint8_t *curr_cursor = trace_cursor;
   uint8_t curr_bit = bit_cursor;
   uint8_t to_print;
   uint16_t i;

   /*
   printf("Entire trace:\n");
   for(i=0; i<DEBUG_TRACE_SIZE; i++)
      printf("%d\t%d\n", debug_trace[i], &debug_trace[i]);
   printf("End entire trace.\n");
   */
   
   //run through the circular trace until we reach the front again
   //printf("Last trace: %d\n", trace_cursor);
   do 
   {
      if(((*curr_cursor >> curr_bit) & LOWER_BIT_MASK) != 0)
      {
	 // look at the next available code
	 to_print = ((*curr_cursor >> curr_bit) & LOWER_BIT_MASK);
	 printf("%d\t%d\t%d\n", to_print, curr_cursor, curr_bit);
	 
	 
      }
      
      curr_bit += DBCODE_SIZE;
      if(curr_bit > 7)
      {
	 //check if we need to cycle around
	 if(curr_cursor == &debug_trace[DEBUG_TRACE_END])
	    curr_cursor = &debug_trace[DEBUG_TRACE_START];
	 else
	    curr_cursor++;

	 //printf("end = %d\t", &debug_trace[DEBUG_TRACE_END]);
	 //printf("curr = %d\n", curr_cursor);
	 curr_bit = 0;
      }
   } while(curr_cursor != trace_cursor || curr_bit != bit_cursor);
   printf("End of trace: %d\n", trace_cursor);

}

static uint8_t error_signal;
static comBuf buf;
   
void error_report_thread()
{
  handle_t fault_handle = mos_disable_ints();
   uint16_t i;
   uint8_t recvcount;
   uint8_t to_print;
   uint8_t curr_bit = 0;
   uint8_t* mycursor;
   uint16_t count;
   uint16_t bytecount;
   comBuf* temp;
   comBuf* recvbuf = NULL;

   debug_net_packet_t* mynetpkt;
   debug_packet_t* mydebugpkt;

   debug_net_packet_t* recvednetpkt;
   debug_packet_t* recveddebugpkt;

    disable_debugging();

    wdt_disable();

    mos_led_on(2);
    
     //printf("in error report\n");
   
   //DISABLE_ALARM_TIMER();

/*   if(temp = !get_free_buf()) //no free combufs left
   {
      //do something
   }
   else
      com_free_buf(temp);
*/ 
   
   // reinit device drivers

   //com_init();

/*#ifdef PLATFORM_MICA2
   cc1000_csma_init();
#else
   

   cc2420_init();
   
#endif
*/

   
     
   com_mode(IFACE_RADIO, IF_LISTEN);
   
   buf.size = sizeof(debug_net_packet_t) + sizeof(debug_packet_t);

   // set up debugging packets

   mynetpkt = (debug_net_packet_t*)&(buf.data[0]);
   mydebugpkt = (debug_packet_t*)&(buf.data[sizeof(debug_net_packet_t)]);
   
   mynetpkt->type = 254; //debug packet
   mynetpkt->src   = mos_node_id_get(); //preset
   mynetpkt->seqno = 0;
   mynetpkt->dest = 0; //base station addr
   
   mydebugpkt->error_code = error_signal;

   /**********************************/
   /* THESE SHOULD NORMALLY BE COMMENTED, DEBUG CODE */
   
   mos_enable_ints(fault_handle);
   mos_debug_print_trace();

   /**************************************/

   
 recvcount = 0;
   mycursor = trace_cursor;
   while(1) {
      mydebugpkt->offset = 0;

      bytecount = 0;
      for(i=DEBUG_TRACE_SIZE-1; i>0;i--)
      {
	 if((*(mycursor)) != 0)
	 {
	    mydebugpkt->data[bytecount] = *(mycursor);
	    //printf("%d ", mydebugpkt->data[bytecount]);

	    bytecount++;
	    if(bytecount >= DEBUG_DATA_SIZE)
	    {
	       mydebugpkt->size = DEBUG_DATA_SIZE;
	       com_send(IFACE_RADIO, &buf);
	       //printf("\n");
	       mydebugpkt->offset++;
	       bytecount = 0;
	       mos_mdelay(200);
	    }
	 }
	 mycursor++;

	 if(mycursor == &debug_trace[DEBUG_TRACE_END])
	    mycursor = &debug_trace[DEBUG_TRACE_START];

	 
      }

      // if we still need to send a packet
      if(bytecount != 0)
      {
	 mydebugpkt->size = bytecount;
	 com_send(IFACE_RADIO, &buf);
	 mos_led_on(1);
	//printf("\n");
      }
      //printf("sent packets %d\n\n", signal);


      while(1);
      
      mos_mdelay(1000);

      //printf("**Trace\t\t\tbyte\tbit\n");

      //while(1)
      {
	 recvbuf = com_recv_noblock(IFACE_RADIO);
	 recvednetpkt = (debug_net_packet_t*)&(recvbuf->data[0]);
	 recveddebugpkt = (debug_packet_t*)&(recvbuf->data[sizeof(debug_net_packet_t)]);

	 if(recvbuf == NULL)
	 {
	    
	 }
	 else
	 {
	    

	 if(recvednetpkt->type == 254)
	 {
	    //printf("recved\n");
	    
	    i = 0;
	    curr_bit = 0;
	    count = 1;
	    while(i < recveddebugpkt->size)
	    {
	       if(curr_bit == 0)
	       {
		  to_print = ((recveddebugpkt->data[i] & LOWER_BIT_MASK));
		  curr_bit = 4;
	       }
	       else
	       {
		  to_print = ((recveddebugpkt->data[i++] >> curr_bit) & LOWER_BIT_MASK);
		  curr_bit = 0;
	       }
	       
	       
	       if(to_print != 0)
	       {
		  /*printf("%d. ", count++);
		  switch(to_print)
		  {
		  case DBCODE_CONTEXT_SWITCH: printf("context switch\t"); break;
		  case DBCODE_PROCEDURE_CALL: printf("procedure call\t"); break;
		  case DBCODE_PROCEDURE_RETURN: printf("procedure return\t");break;
		  case DBCODE_INTERRUPT: printf("interrupt\t\t");break;
		  case DBCODE_THREAD_BLOCK: printf("thread block\t");break;
		  case DBCODE_THREAD_UNBLOCK: printf("thread unblock\t");break;
		  case DBCODE_TIMER_SET: printf("timer set\t\t");break;
		  case DBCODE_TIMER_FIRED: printf("timer fired\t\t");break;
		  case DBCODE_THREAD_SLEEP: printf("thread sleep\t");break;
		  case DBCODE_THREAD_WAKEUP: printf("thread wakeup\t");break;
		  case DBCODE_NODE_SLEEP: printf("deep sleep\t\t");break;
		  case DBCODE_NODE_WAKEUP: printf("deep wakeup\t\t");break;
		  case DBCODE_THREAD_NEW: printf("thread new\t\t");break;
		  case DBCODE_THREAD_EXIT: printf("thread exit\t\t");break;
		  case DBCODE_BREAKPOINT: printf("breakpoint\t\t");break;
		  default: break;
		     
		  }
		  printf("\n");*/
		  //printf("%d\t%d\n", i, curr_bit);
	       }
	    }
	 }
	 
	 
	 
	 com_free_buf(recvbuf);
	 }
	 
      }
      
   }


   
}

void mos_debug_error_report(uint8_t signal)
{
   handle_t int_handle = mos_disable_ints();
   lock_debugging();
   error_signal = signal;
   disable_debugging();
   mos_enable_ints(int_handle);
   mos_thread_new(error_report_thread, 128, PRIORITY_HIGH);
}

static stackval_t *mysp;
void mos_debug_check_stack()
{
   if(_current_thread)
   {
      
      asm volatile(			\
	 "in %A0, __SP_L__\n\t"		\
	 "in %B0, __SP_H__\n\t"		\
	 : "=r" (mysp) : );	

      // need to account for the overhead needed for the stack checking function, use 4 bytes for now
      if((mysp-4) < _current_thread->stack)
	 mos_debug_error_report(SIGNAL_STACK_OVERFLOW);

      //printf("mysp %d, sp %d, sst %d, ssi %d\n", mysp, mos_thread_current()->sp, mos_thread_current()->stack, mos_thread_current()->stackSize);
   
   }
   
}

#endif //MOS_DEBUG
