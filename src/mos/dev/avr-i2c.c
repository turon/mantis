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

#ifdef ARCH_AVR

#include "sem.h"
#include "dev.h"
#include "led.h"
#include "mutex.h"
#include "avr-i2c.h"
#include "printf.h"

mos_mutex_t i2c_mutex;
static mos_sem_t i2c_sem_read;
static mos_sem_t i2c_sem_write;
static boolean reading;
static uint8_t *read_buf;
static uint16_t read_count;
static uint16_t read_max;

static uint8_t *write_buf;
static uint16_t write_count;
static uint16_t write_max;

static uint8_t mode;
static uint8_t state;
static uint8_t SLA_W;

#define avr_i2c_wait_status_read(status_code)	\
   while(!(TWCR & (1<<TWINT)))                  \
   ;                                            \
                                                \
   if(avr_i2c_get_status() != status_code)      \
   {					        \
      mos_mutex_unlock (&i2c_mutex);	        \
      if (mode == DEV_MODE_OFF)				 \
	 avr_i2c_disable();				 \
      return read_count;		        \
   }					        \



#define avr_i2c_wait_status_write(status_code)	 \
   while(!(TWCR & (1<<TWINT)))                   \
   ;                                             \
                                                 \
   if(avr_i2c_get_status() != status_code)       \
   {					         \
      mos_mutex_unlock (&i2c_mutex);	         \
      if (mode == DEV_MODE_OFF)			 \
	 avr_i2c_disable();			 \
      return write_count;			 \
   }					         \


uint16_t dev_read_DEV_AVR_I2C(void *buf, uint16_t count)
{
   if(count == 0)
      return 0;
   
   if(mode == DEV_MODE_OFF)
      avr_i2c_enable();
   
   read_max = count;
   read_count = 0;
   read_buf = (uint8_t *)buf;
   reading = true;

   uint8_t polling = true;

   if(polling)
   {
      //transmit a start condition
      TWCR |= (1 << TWINT) | (1<<TWSTA);

      //wait for response
      avr_i2c_wait_status_read(START);

      //transmit SLA+R
      avr_i2c_put_byte(SLA_W | 0x01); //lsb = 1 for read


      //wait for response
      TWCR = (1 << TWINT) | (1 << TWEN);
      avr_i2c_wait_status_read(MR_SLA_ACK);

      //get count -1 bytes and return ACK
      while(read_count < read_max - 1){
	 TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);

	 //wait for response
	 avr_i2c_wait_status_read(MR_DATA_ACK);

	 read_buf[read_count++] = avr_i2c_get_byte();
      }

      //get last byte, return NACK
      TWCR = (1 << TWINT) | (1 << TWEN);
      
      //wait for response
      avr_i2c_wait_status_read(MR_DATA_NACK);

      read_buf[read_count++] = avr_i2c_get_byte();

      //issue a stop condition
      TWCR = (1 << TWSTO) | (1 << TWINT);
      
   } else {
      mos_sem_wait (&i2c_sem_read);
   }

   if(mode == DEV_MODE_OFF)
      avr_i2c_disable();
   
   mos_mutex_unlock(&i2c_mutex);
   return read_count;
}


uint16_t dev_write_DEV_AVR_I2C (const void *buf, uint16_t count)
{
   uint8_t int_handle;
   mos_mutex_lock (&i2c_mutex);

   //initialize counting vars
   write_count = 0;
   write_max = count;
   write_buf = (uint8_t *)buf;

   
   if(mode == DEV_MODE_OFF)
      avr_i2c_enable();

   state = START;
   reading = false;

   uint8_t polling = true;

   if(polling) {
      TWCR |= (1 << TWINT) | (1<<TWSTA);

      avr_i2c_wait_status_write(START);
      avr_i2c_put_byte(SLA_W);

      TWCR = (1 << TWINT) | (1 << TWEN);

      avr_i2c_wait_status_write(MT_SLA_ACK);

      while(write_count < write_max) {
	 TWDR = write_buf[write_count++];
	 TWCR = (1 << TWINT) | (1 << TWEN);

	 avr_i2c_wait_status_write(MT_DATA_ACK);
      }

      TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
   } else {
      int_handle = mos_disable_ints();
      avr_i2c_interrupt_enable(); //enable interrupt mode
      avr_i2c_clear_int_flag(); //clear old flag
      avr_i2c_start_condition(); //initiate start condition
      
      mos_led_display(5);
      mos_enable_ints(int_handle);
      mos_led_display(4);
      mos_sem_wait(&i2c_sem_write);
   }
   
   if(mode == DEV_MODE_OFF)
      avr_i2c_disable();
   
   mos_mutex_unlock(&i2c_mutex);
   return write_count;
}

uint8_t dev_mode_DEV_AVR_I2C (uint8_t newMode)
{
   mode = newMode;
   switch (newMode) {
   case DEV_MODE_ON:
      avr_i2c_enable();
      break;
   case DEV_MODE_OFF:
      avr_i2c_disable();
      break;
   default:
      return DEV_UNSUPPORTED;
   }

   return DEV_OK;
}

uint8_t dev_ioctl_DEV_AVR_I2C (int8_t request, ...)
{
   uint8_t input;
   va_list ap;
   va_start(ap, request);

   switch (request) {
   case I2C_ENABLE_ACK:
      TWCR |= (1 << TWEA);
      break;
   case I2C_DISABLE_ACK:
      TWCR &= ~(1 << TWEA);
      break;
   case I2C_DEST_ADDR:
      input = (uint8_t)va_arg (ap, uint16_t);
      SLA_W = (input << 1);
      break;
   case I2C_SLAVE_ADDR:
      TWAR |= va_arg (ap, uint16_t);
      break;
   case I2C_SET_BRR:
      input = (uint8_t)va_arg (ap, uint16_t);
      TWBR = input;
      break;
   default:
      return DEV_BAD_IOCTL;
   }

   return DEV_OK;
}

void avr_i2c_init(void)
{
   mode = DEV_MODE_OFF;
   mos_mutex_init(&i2c_mutex);
   mos_sem_init(&i2c_sem_write, 0);
   mos_sem_init(&i2c_sem_read, 0);
   avr_i2c_disable();
}

SIGNAL (SIG_TWI) {
#ifdef MOS_DEBUG
   mos_debug_set_trace(DBCODE_INTERRUPT);
#endif
   mos_led_display(3);
   if(reading){
      switch (state) {
      case START:
	 if(avr_i2c_get_status() != START) {
	    mos_sem_post_dispatch (&i2c_sem_read);
	    return;
	 }
	 
	 avr_i2c_put_byte (TWAR >> 1); //write SLA+R for master - receiver
	 state = MR_SLA_ACK;
	 //Clear TWINT bit in TWCR to start xmission of address
	 avr_i2c_clear_int_flag (); 
	 break;
      case MR_SLA_ACK:
	 if(avr_i2c_get_status() != MR_SLA_ACK) {
	    mos_sem_post_dispatch (&i2c_sem_read);	    
	    return;
	 }
	 
	 read_buf[read_count++] = avr_i2c_get_byte();
	 state = MR_DATA_ACK;
 	 avr_i2c_clear_int_flag (); 
	 break;
      case MR_DATA_ACK:
	 if(avr_i2c_get_status() != MR_DATA_ACK){
	    mos_sem_post_dispatch (&i2c_sem_read);
	    return;
	 }
	    
	 if(read_count < read_max){ //still reading
	    read_buf[read_count++] = avr_i2c_get_byte();
	    avr_i2c_clear_int_flag ();
	 } else { //done reading
	    //send a NACK after last recieved byte
	    avr_i2c_stop_condition ();
	    avr_i2c_interrupt_disable ();
	    mos_sem_post_dispatch (&i2c_sem_read);
	 }
	 break;
      default:
	 return;
      }
   } else { //writing
      switch (state) {
      case START:
	 if(avr_i2c_get_status() != START) {
	    mos_sem_post_dispatch (&i2c_sem_write);
	    return;
	 }
         avr_i2c_put_byte(SLA_W); //Load SLA_W into TWDR Register
	 
	 state = MT_SLA_ACK;
	 //Clear TWINT bit in TWCR to start xmission of address
	 avr_i2c_clear_int_flag (); 
	 break;
      case MT_SLA_ACK:
	 if(avr_i2c_get_status() != MT_SLA_ACK) {
	    mos_sem_post_dispatch (&i2c_sem_write);
	    return;
	 }
	 
	 avr_i2c_put_byte(write_buf[write_count++]); //write one byte
	 
	 state = MT_DATA_ACK;
	 
	 avr_i2c_clear_int_flag (); //clear int to start xmission
	 break;
      case MT_DATA_ACK:
	 if(avr_i2c_get_status() != MT_DATA_ACK) {
	    mos_sem_post_dispatch (&i2c_sem_write);
	    return;
	 }
	 
	 if(write_count < write_max){
	    avr_i2c_put_byte(write_buf[write_count++]); //write one byte
	    avr_i2c_clear_int_flag (); //clear int to start xmission
	 } else {
	    avr_i2c_stop_condition ();
	    avr_i2c_clear_int_flag ();
	    state = MT_DATA_DONE;
	 }
	 break;
      case MT_DATA_DONE:
	 avr_i2c_interrupt_disable ();
	 mos_sem_post_dispatch(&i2c_sem_write);
	 break;
      default:
	 return;
      } // switch
   } //if reading
  
}

#endif
