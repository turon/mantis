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

#include "com.h"
#include "mos.h"
#include "dev.h"
#include "spi.h"
#include "cc2420-gpio.h"

#if defined(PLATFORM_SUPPORTS_CC2420)

#include <stdarg.h>
#include "sem.h"
#include "msched.h"
#include "node_id.h"
#include "mutex.h"
#include "clock.h"
#include "cc2420.h"
#include "led.h"
#include "bitops.h"
#include "plat_dep.h"
#include "printf.h"

#if defined(CC2420) || !defined(SCONS)

#define DEFAULT_CHANNEL 26

static uint16_t pan_id = 0x2420;
static uint16_t id;
static uint8_t cc2420_com_mode;
static comBuf *cc2420_recv_buf;  //receive buffer

//recv variables
static uint8_t len;
static uint16_t fcf;
static uint8_t seq;
static uint16_t addr;
static uint16_t rcv_id;

static uint16_t last_rssi;
#ifdef ENABLE_AUTOACK
static uint8_t last_acked_seq = 0xff;
static uint8_t last_send_seq = 0xff;
#ifdef ENABLE_RETRANSMIT

#define MAX_CACHE_SIZE     10
static uint8_t cache_next = 0;
typedef struct cache_entry_t {
  uint16_t source;
  uint8_t  seqNo;
} cache_entry_t;
static cache_entry_t dup_cache[MAX_CACHE_SIZE];

#endif
#endif

//send variables
static uint8_t status;
static uint16_t dest_addr = BROADCAST_ADDR;
static uint8_t send_len;
static uint16_t send_fcf;
static uint8_t send_seq = 0;


//timeout
//static uint16_t overflow;

#define CC2420_ADDR_READ_MASK 0x40;

static inline void cc2420_rx_address(uint8_t rx_addr)
{
    rx_addr |= CC2420_ADDR_READ_MASK;
    dev_write(DEV_SPI, &rx_addr, sizeof(rx_addr));
}

static void cc2420_strobe(uint8_t byte)
{
    dev_ioctl(DEV_SPI, SPI_SET_SLAVE_SELECT, CC2420_SLAVE_SELECT);
    dev_write(DEV_SPI, &byte, sizeof(byte));
    dev_ioctl(DEV_SPI, SPI_CLEAR_SLAVE_SELECT);
}
uint16_t cc2420_get_register(uint8_t reg);

void cc2420_set_register(uint8_t reg, uint16_t val)
{
    dev_ioctl(DEV_SPI, SPI_SET_SLAVE_SELECT, CC2420_SLAVE_SELECT);
    dev_write(DEV_SPI, &reg, sizeof(reg));
    dev_write(DEV_SPI, &val, sizeof(val));
    dev_ioctl(DEV_SPI, SPI_CLEAR_SLAVE_SELECT);
}

uint16_t cc2420_get_register(uint8_t reg)
{
    uint16_t ret;
    dev_ioctl(DEV_SPI, SPI_SET_SLAVE_SELECT, CC2420_SLAVE_SELECT);
    cc2420_rx_address(reg);
    dev_read(DEV_SPI, &ret, sizeof(ret));
    dev_ioctl(DEV_SPI, SPI_CLEAR_SLAVE_SELECT);
    return ret;
}

static uint8_t cc2420_update_status(void)
{
    uint8_t ret;
    dev_ioctl(DEV_SPI, SPI_SET_SLAVE_SELECT, CC2420_SLAVE_SELECT);
    dev_read(DEV_SPI, &ret, sizeof(ret));
    dev_ioctl(DEV_SPI, SPI_CLEAR_SLAVE_SELECT);
    return ret;
}

static void cc2420_write_ram(uint8_t *addr, uint16_t radio_addr,
        uint8_t count)
{
    uint8_t tmp;
    dev_ioctl(DEV_SPI, SPI_SET_SLAVE_SELECT, CC2420_SLAVE_SELECT);
    tmp = 0x80 | (radio_addr & 0x7F);
    dev_write(DEV_SPI, &tmp, sizeof(tmp));
    tmp = (radio_addr >> 1) & 0xC0;
    dev_write(DEV_SPI, &tmp, sizeof(tmp));

    dev_write(DEV_SPI, addr, count);
    dev_ioctl(DEV_SPI, SPI_CLEAR_SLAVE_SELECT);
}

static void cc2420_read_ram(uint8_t *addr, uint16_t radio_addr, uint8_t count)
{
    uint8_t i;
    dev_ioctl(DEV_SPI, SPI_SET_SLAVE_SELECT, CC2420_SLAVE_SELECT);
    i = 0x80 | (radio_addr & 0x7F);
    dev_write(DEV_SPI, &i, sizeof(i));
    i = ((radio_addr >> 1) & 0xC0) | 0x20;
    dev_write(DEV_SPI, &i, sizeof(i));
    dev_read(DEV_SPI, (uint8_t *)addr, count);
    //uint8_t *p = (uint8_t *)addr;
    //   p[0] = 0xEF;
    dev_ioctl(DEV_SPI, SPI_CLEAR_SLAVE_SELECT);
}

uint16_t dev_read_DEV_SPI_1(void *buf, uint8_t count)
{
    uint8_t i;
    for(i = 0; i < count && cc2420_get_fifo(); i++)
    {
        dev_ioctl(DEV_SPI, SPI_SET_SLAVE_SELECT, CC2420_SLAVE_SELECT);
        dev_read(DEV_SPI, &((uint8_t *)buf)[i], 1);
        dev_ioctl(DEV_SPI, SPI_CLEAR_SLAVE_SELECT);
    }
    return i;
}


static void cc2420_write_fifo(void *addr, uint8_t count)
{
    dev_ioctl(DEV_SPI, SPI_SET_SLAVE_SELECT, CC2420_SLAVE_SELECT);
    uint8_t tmp = CC2420_TXFIFO;
    mos_udelay(1);
    dev_write(DEV_SPI, &tmp, sizeof(tmp));
    mos_udelay(1);
    dev_write(DEV_SPI, (uint8_t *)addr, count);
    mos_udelay(1);
    dev_ioctl(DEV_SPI, SPI_CLEAR_SLAVE_SELECT);
}

static void cc2420_read_fifo(void *addr, uint8_t count)
{
    uint8_t i;
    uint8_t *p = (uint8_t *)addr;
    dev_ioctl(DEV_SPI, SPI_SET_SLAVE_SELECT, CC2420_SLAVE_SELECT);
    cc2420_rx_address(CC2420_RXFIFO);
    for(i = 0; i < count; i++) {
        while(!cc2420_get_fifo())
            asm volatile("nop");
        volatile uint8_t in_byte;
        dev_read(DEV_SPI, (void *)&in_byte, sizeof(uint8_t));
        p[i] = in_byte;
    }

    dev_ioctl(DEV_SPI, SPI_CLEAR_SLAVE_SELECT);
}

static void cc2420_read_fifo_garbage(uint8_t count)
{
    uint8_t i;
    dev_ioctl(DEV_SPI, SPI_SET_SLAVE_SELECT, CC2420_SLAVE_SELECT);
    cc2420_rx_address(CC2420_RXFIFO);
    uint8_t garbage;
    for(i = 0; i < count; i++) {
        dev_read(DEV_SPI, &garbage, sizeof(garbage));
        //dev_read_DEV_SPI_1(&garbage, sizeof(garbage));
    }
    dev_ioctl(DEV_SPI, SPI_CLEAR_SLAVE_SELECT);
}

static void cc2420_read_fifo_no_wait(void *addr, uint8_t count)
{
    //uint8_t i;
    dev_ioctl(DEV_SPI, SPI_SET_SLAVE_SELECT, CC2420_SLAVE_SELECT);
    cc2420_rx_address(CC2420_RXFIFO);
    dev_read(DEV_SPI, (uint8_t *)addr, count);
    //i = dev_read_DEV_SPI_1((uint8_t *)addr, count);
    dev_ioctl(DEV_SPI, SPI_CLEAR_SLAVE_SELECT);
    //return i;
    return;
}

void cc2420_init(void)
{
    handle_t int_handle;
    id = mos_node_id_get();
    int_handle = mos_disable_ints();

    cc2420_gpio_init();

    cc2420_strobe(CC2420_SXOSCON);

#ifdef ENABLE_AUTOACK    
    // enable automatic packet ack, hw address recognition and auto crc 
    cc2420_set_register (CC2420_MDMCTRL0, 0x0AF2);
#else
    // enable auto crc. no auto ack, no hw address recognition
    //cc2420_set_register(CC2420_MDMCTRL0, 0x02E2);

    // enable hw address recognition and auto crc. no auto ack
    cc2420_set_register(CC2420_MDMCTRL0, 0x0AE2);  
#endif

    // set correlation threshold = 20
    cc2420_set_register(CC2420_MDMCTRL1, 0x0500);
    // set fifop threshold to max
    cc2420_set_register(CC2420_IOCFG0, 0x007F);
    // turn off security
    cc2420_set_register(CC2420_SECCTRL0, 0x01C4);

    // set the channel
    cc2420_set_channel(DEFAULT_CHANNEL);   

    // wait for the oscillator
    //   id = 0xFFFF;
    cc2420_wait_for_osc();
    uint16_t id1 = ((id>>8) | (id &0xff) <<8);
    cc2420_write_ram(&id1, CC2420RAM_SHORTADDR, sizeof(id));
    cc2420_write_ram(&pan_id, CC2420RAM_PANID, sizeof(pan_id));

    // no need to do this any more. we fixed the mos_node_id_init problem
    //cc2420_read_ram(&id, CC2420RAM_PANID, sizeof(id));

    // start out with a free com buf
    com_swap_bufs(IFACE_RADIO, NULL, &cc2420_recv_buf);

    // set initial com mode
    cc2420_com_mode = IF_OFF;

    // clear out any starting data in the fifos
    cc2420_strobe(CC2420_SFLUSHRX);
    cc2420_strobe(CC2420_SFLUSHTX);

    mos_enable_ints(int_handle);

#ifdef ENABLE_AUTOACK
#ifdef ENABLE_RETRANSMIT
    uint8_t ci;
    for(ci = 0; ci < MAX_CACHE_SIZE; ci++) {
       dup_cache[ci].source = BROADCAST_ADDR;
       dup_cache[ci].seqNo  = 0xff;
    }
#endif
#endif

}


void cc2420_wait_for_osc(void)
{
    uint8_t status_byte;
    do {
        status_byte = cc2420_update_status();
    } while(!(status_byte & (1 << CC2420_XOSC16M_STABLE)));
}

void cc2420_set_channel(uint8_t chan)
{
    uint16_t c;

    c = (uint16_t)(chan - 11);
    c = c + (c << 2);
    c = c + 357;
    c = c + 0x4000;

    handle_t int_handle = mos_disable_ints();
    cc2420_set_register(CC2420_FSCTRL, c);
    mos_enable_ints(int_handle);
}

uint16_t cc2420_get_last_rssi() {
    return last_rssi;
}

#ifdef ENABLE_AUTOACK
uint8_t cc2420_get_last_send_seq() {
    return last_send_seq;
}
uint8_t cc2420_get_last_acked_seq() {
    return last_acked_seq;
}
#endif

void com_ioctl_IFACE_RADIO(uint8_t request, ...)
{
    int arg;
    va_list ap;
    va_start(ap, request);
    arg = va_arg(ap, int);
    uint16_t power;

    switch(request) {
        case CC2420_LOW_POWER_MODE:
            // Reduces the power to minimum
            cc2420_set_register(CC2420_TXCTRL, 0xA0E0);
            break;
        case CC2420_HIGH_POWER_MODE:
            // Sets power to maximum
            cc2420_set_register(CC2420_TXCTRL, 0xA0FF);
            break;


        case CC2420_TX_POWER:

            power = arg;

            if(power>0xFF)
                break;
            power += 0xA0E0;
            cc2420_set_register(CC2420_TXCTRL, power);
            break;

        default:
            break;
    }

    va_end(ap);

    return;
}

void com_mode_IFACE_RADIO(uint8_t md)
{
    handle_t int_handle;

    switch(md) {
        case IF_OFF:
            int_handle = mos_disable_ints();
            cc2420_disable_fifop_interrupt();
            //turn off radio
            cc2420_strobe(CC2420_SRFOFF);
            mos_enable_ints(int_handle);
            cc2420_com_mode = md;
            break;

        case IF_STANDBY:
            break;

        case IF_IDLE:
            break;

        case IF_LISTEN:
            if (cc2420_com_mode == CC2420_MODE_RX)
                break;

            int_handle = mos_disable_ints();
            cc2420_enable_fifop_interrupt();
            //power up radio
            cc2420_strobe(CC2420_SRXON);
            cc2420_strobe(CC2420_SFLUSHRX);
            mos_enable_ints(int_handle);
            //cc2420_com_mode = md;
            cc2420_com_mode = CC2420_MODE_RX;
            break;
    }

    return;
}


uint8_t com_send_IFACE_RADIO (comBuf *buf)
{
    uint16_t overflow;  

    if(!buf) return 1;
   
    handle_t int_handle;

    overflow = 0;
    while(cc2420_get_sfd() || cc2420_get_fifop())
    {
        if(++overflow == 0)
          return 1;
    }

    int_handle = mos_disable_ints();

    // 11 bytes overhead for header
    send_len = buf->size + 11;

    // no ack frame control field 0x8841
    // should use big edianess if cc2420_write_fifo() is used.
    // ensure that 0x41 is sent before 0x88
    send_fcf = 0x4188; 

    cc2420_strobe(CC2420_SFLUSHTX);

    // turn on receiver if necessary
    if (cc2420_com_mode != CC2420_MODE_RX) {
        cc2420_strobe(CC2420_SRXON);
    }

    overflow = 0;

    // wait for RSSI
    do {
        status = cc2420_update_status();
        {
            if(++overflow == 0)
            {
                mos_enable_ints(int_handle);
                return 1;
            }
        }
    } while(!(status & (1 << CC2420_RSSI_VALID)));

    overflow = 0;

    // wait for transmission
    do {
        cc2420_strobe(CC2420_STXONCCA);
        status = cc2420_update_status();

        //Second trouble spot.  Driver sometimes hangs here.
        //mos_led_blink(2);
        if(++overflow == 0)
        {
            mos_enable_ints(int_handle);
            return 1;
        }

    } while(!(status & (1 << CC2420_TX_ACTIVE)));

    ++send_seq;

    // fill up the fifo with the frame format
    cc2420_write_fifo(&send_len, sizeof(send_len));
    cc2420_write_fifo(&send_fcf, sizeof(send_fcf));
    cc2420_write_fifo(&send_seq, sizeof(send_seq));
    cc2420_write_fifo(&pan_id, sizeof(pan_id));
    cc2420_write_fifo(&dest_addr, sizeof(dest_addr));
    cc2420_write_fifo(&id, sizeof(id));
    // now fill in our packet's data
    cc2420_write_fifo(buf->data, buf->size);

    overflow = 0;

    while(!cc2420_get_sfd())
    {
        if(++overflow == 0)
        {
            mos_enable_ints(int_handle);
            return 1;
        }
        //Third problem spot. Driver sometimes hangs here. 
        //Adding a blink led here hangs the driver causes problems.
    }

    if(cc2420_com_mode != CC2420_MODE_RX) {
        int_handle = mos_disable_ints();  //???
        cc2420_strobe(CC2420_SRFOFF);
    }
   
#ifdef ENABLE_AUTOACK
    last_send_seq = send_seq;
#endif

    mos_enable_ints(int_handle);    
    return 0;
}

#define htons(v)   (((v) >> 8) | (((v) & 0xff) << 8))
uint8_t com_sendto_IFACE_RADIO (comBuf *buf, uint16_t __dest_addr)
{
    uint8_t my_send_seq;
    uint16_t overflow;

    if(__dest_addr == BROADCAST_ADDR)
    {
     // auto-ack or re-transmission won't be used for broadcasting
      return com_send_IFACE_RADIO(buf);
    }
    __dest_addr = htons(__dest_addr);

    handle_t int_handle;
    my_send_seq = (++send_seq);     

#if defined(ENABLE_AUTOACK) && defined(ENABLE_RETRANSMIT)  
  uint8_t tx_attempts = 0;
  while(tx_attempts <= MAX_RETRANSMIT_ATTEMPTS)
  {
#endif
    overflow = 0;
    while(cc2420_get_sfd() || cc2420_get_fifop())
    {
        if(++overflow == 0) 
          return 0xff;           
    }

    int_handle = mos_disable_ints();

    // 11 bytes overhead for header
    send_len = buf->size + 11;

#ifndef ENABLE_AUTOACK
    // noack frame control field 0x8841
    // should use big edianess if cc2420_write_fifo() is used.
    // ensure that 0x41 is sent before 0x88
    send_fcf = 0x4188; 
#else
    // ack frame control field 0x8861
    // should use big edianess if cc2420_write_fifo() is used.
    // ensure that 0x61 is sent before 0x88
    send_fcf = 0x6188; 
#endif

    cc2420_strobe(CC2420_SFLUSHTX);

    // turn on receiver if necessary
    if (cc2420_com_mode != CC2420_MODE_RX) {
        cc2420_strobe(CC2420_SRXON);
    }

    overflow = 0;
    // wait for RSSI
    do {
        status = cc2420_update_status();
        {
            if(++overflow == 0)
            {
                mos_enable_ints(int_handle);
                return 0xff;
            }
        }
    } while(!(status & (1 << CC2420_RSSI_VALID)));

    overflow = 0;
    // wait for transmission
    do {
        cc2420_strobe(CC2420_STXONCCA);
        status = cc2420_update_status();

        //Second trouble spot.  Driver sometimes hangs here.
        //mos_led_blink(2);
        if(++overflow == 0)
        {
            mos_enable_ints(int_handle);
            return 0xff;
        }
    } while(!(status & (1 << CC2420_TX_ACTIVE)));

    // fill up the fifo with the frame format
    cc2420_write_fifo(&send_len, sizeof(send_len));
    cc2420_write_fifo(&send_fcf, sizeof(send_fcf));
    cc2420_write_fifo(&send_seq, sizeof(my_send_seq));
    cc2420_write_fifo(&pan_id, sizeof(pan_id));
    cc2420_write_fifo(&__dest_addr, sizeof(__dest_addr));
    cc2420_write_fifo(&id, sizeof(id));
    // now fill in our packet's data
    cc2420_write_fifo(buf->data, buf->size);

    overflow = 0;
    while(!cc2420_get_sfd())
    {
        if(++overflow == 0)
        {
            mos_enable_ints(int_handle);
            return 0xff;
        }
        //Third problem spot. Driver sometimes hangs here. 
        //Adding a blink led here hangs the driver causes problems.
    }

#ifdef ENABLE_AUTOACK  
#ifdef ENABLE_RETRANSMIT
    if (tx_attempts == 0)
      last_send_seq = send_seq;
#else
    last_send_seq = send_seq;
#endif

    mos_enable_ints(int_handle);  
    /* 
       wait long enough to receive an automatic acknowledgement.
       Delay >= 12 symbol periods (auto ack turnaround, i.e. 192 us) + 
                22 symbol periods (auto ack frame transmission time -- 11-byte physical frame) +
                2 symbol period +
                350 us (safety margin)
    */       
    mos_udelay(1000); 
    int_handle = mos_disable_ints();
#endif

    if(cc2420_com_mode != CC2420_MODE_RX) {
        int_handle = mos_disable_ints();
        cc2420_strobe(CC2420_SRFOFF);
    } 
   
#if defined(ENABLE_AUTOACK) && defined(ENABLE_RETRANSMIT)
    if( last_acked_seq == my_send_seq) {
        mos_enable_ints(int_handle);        
        break;    
    }
    mos_enable_ints(int_handle);

    /* wait a while to avoid self-collision */ 
    mos_thread_sleep((id & 0x0f) + 16); 

    ++tx_attempts;    
  } // end of "while(tx_attempts--) 
  
  /* return the number of re-transmission attempts */
  return (tx_attempts);  

#else       
    mos_enable_ints(int_handle);    
    return 0;        
#endif
        
}

//static uint8_t check_fifo_no_wait_retval;
#if defined(ENABLE_AUTOACK) && defined(ENABLE_RETRANSMIT)
static uint8_t ndup = 0;
uint8_t cc2420_num_suppressed_dup()
{ 
  uint8_t n = ndup;     
  ndup = 0;
  return n;
}
boolean in_cache(uint16_t source, uint8_t seqNo)
{
  static uint8_t ci;
  for(ci = 0; ci < MAX_CACHE_SIZE; ci++)
    if((source == dup_cache[ci].source) && (seqNo == dup_cache[ci].seqNo)) {
      ++ndup;
      return true; 
  }
  return false; 
}

void put_in_cache(uint16_t source, uint8_t seqNo)
{
  dup_cache[cache_next].source = source;
  dup_cache[cache_next].seqNo  = seqNo;
  cache_next = (cache_next + 1) % MAX_CACHE_SIZE;
}
#endif

CC2420_FIFOP_INTERRUPT()
{
#ifdef MOS_DEBUG
    mos_debug_set_trace(DBCODE_INTERRUPT);
#endif

#ifdef PLATFORM_TELOSB
    // clear interrupt flag
    P1IFG &= ~(1 << 0);
#endif

    // exit on overflow, indicated by FIFOP == 1 and FIFO == 0
    if(!cc2420_get_fifo() && !cc2420_get_fifop())
    {
        // not sure why there are two of these... (from example code)
        // twice flusching RX is actually required by cc2420 datasheet
        cc2420_strobe(CC2420_SFLUSHRX);
        cc2420_strobe(CC2420_SFLUSHRX);
        return;
    }

    cc2420_read_fifo(&len, sizeof(len));
    // mask out msb
    len &= 0x7f;
    if(len < 5) {
        cc2420_read_fifo_garbage(len);
        return;
    }

    //2 bytes
    cc2420_read_fifo_no_wait(&fcf, sizeof(fcf));
    //1 byte
    cc2420_read_fifo_no_wait(&seq, sizeof(seq));

     //3 bytes have already been read so far
    len -= 3;

    if(len == 2) {
#ifdef ENABLE_AUTOACK
        // could be an ack packet
      if(fcf == 2) {
        last_acked_seq = seq;
      }
#endif
      cc2420_read_fifo_garbage(len);
      return;
    } else if(len < 9) {
        // packet is too short, discard
        cc2420_read_fifo_garbage(len);
        return;
    } else if(len - 9 > COM_DATA_SIZE)
    {
        // packet is larger than we can handle
        cc2420_read_fifo_garbage(len);
        return;
    } else {	     
	
	// skip the dest PAN addr (taken care of by hw addr recognition)
        cc2420_read_fifo_garbage(2);

        //2 more bytes (dest addr is also taken care of by hw addr recognition)        
        //Do we need to check the return value of cc2420_read_fifo_no_wait() ?
        cc2420_read_fifo_no_wait(&addr, sizeof(addr));
        /*if (addr!= id && addr!= BROADCAST_ADDR) 
        {
          cc2420_read_fifo_garbage(len-4);
          return;
        }*/

        // get source address
        cc2420_read_fifo_no_wait(&rcv_id, sizeof(rcv_id));

        // make sure we have a com buf
        if(!cc2420_recv_buf) {
            com_swap_bufs(IFACE_RADIO, NULL, &cc2420_recv_buf);
            if(!cc2420_recv_buf) {
                cc2420_read_fifo_garbage(len-6);
                return;
            }
        }

#if defined(ENABLE_AUTOACK) && defined(ENABLE_RETRANSMIT)
        if(addr != BROADCAST_ADDR) {
          if(in_cache(rcv_id, seq)) {
             cc2420_read_fifo_garbage(len-6);           
             return;
          }
          else
             put_in_cache(rcv_id, seq);
        }
#endif

        cc2420_recv_buf->source = htons(rcv_id);
        cc2420_recv_buf->size = len - 8;

        // get payload 
        // Is it better: cc2420_recv_buf->size = cc2420_read_fifo_no_wait(cc2420_recv_buf->data, cc2420_recv_buf->size)?       
	cc2420_read_fifo_no_wait(cc2420_recv_buf->data, cc2420_recv_buf->size);

        // store the rssi value for this packet
        // the last two bytes actually contains rssi, crc bit and correlation index
        // format: MSB -- rssi (a signed integer)
        //         LSB -- MS bit: CRC bit
        //                LS 7 bit: correlation index  
        cc2420_read_fifo_no_wait(&last_rssi, 2);

#ifdef GET_RSSI        
        cc2420_recv_buf->signal = *((uint16_t *)&last_rssi);
#endif

        // hand the packet to user space
        com_swap_bufs(IFACE_RADIO, cc2420_recv_buf, &cc2420_recv_buf);
    }
}

#endif
#endif
