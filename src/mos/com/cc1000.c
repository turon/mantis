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


/** @file cc1000.c
 * @brief Driver provides: use of cc1000 state machine, generic to most MACs
 *
 * Driver requires: AVR architecture with cc1000 connected via SPI bus.
 * @authors Jeff Rose, Brian Shucker, Hui Dai
 * @date 03/03/2004
 *
 * This is a conglomeration of cc1000.c and cc1000_defaults.c.  We now integrate the SPI
 * code related to the radio data interface with the configuration code for the radio. [rosejn]
 */

#define INTERRUPT_DRIVEN_SEND 1

#include "mos.h"
#include "com.h"

#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICA2DOT)

#include "led.h"
#include "sem.h"
#include "crc.h"
#include "cc1000.h"
#include "cc1000_params.h"
#include "clock.h"
#include "msched.h"
#include "mutex.h"
#include "stdarg.h"
#include "node_id.h"
#include "avr-rssi.h"
#include "uart.h"

#if defined(CC1000) || !defined(SCONS)

#ifdef RADIO_USE_FEC
#include "fec.h"
#endif

static uint8_t int_handle;

/*driver state info*/
static comBuf *cc1000_recv_buf;  //receive buffer
static comBuf *cc1000_send_buf;  //send buffer
void (*cc1000_state)(void);           //current sync state of receiver
uint8_t cc1000_com_mode;        //current mode

static uint8_t spi_data;        //byte pulled off of SPI bus
static uint8_t prev_data;       //previous byte pulled off bus (for bit shift)
static uint8_t offset;          //recv bit offset
static uint8_t preamble_count;  //preamble bytes received
static uint8_t data_count;      //bytes already received or sent
static uint8_t cc1000_crc_low, cc1000_crc_high; //crc storage

static uint8_t actual_byte;

//error tracking
uint16_t cc1000_success_count;    //completed packets
uint16_t cc1000_crc_error_count;  //dropped packets due to crc error
uint16_t cc1000_mem_error_count;  //dropped packets due to out of memory
uint16_t cc1000_sync_error_count; //dropped packets due to sync failure
uint16_t cc1000_size_error_count; //dropped packets due to size out of range
#ifdef RADIO_USE_FEC
uint16_t cc1000_fec_error_count;  //dropped packets due to fec errors
#endif

//Enable the GET_RSSI and TS_PACKET switches from com.h

//ANS hack:
#ifdef GET_RSSI
uint16_t rssival;
#endif
//ANS hack ends

#ifdef INTERRUPT_DRIVEN_SEND
static mos_sem_t cc1000_sem;          //to synchronize blocking on send
#endif

#ifdef RADIO_REDUNDANT_SIZE
static uint8_t size_bytes[3];
#endif

#ifdef RADIO_USE_FEC
uint8_t data_fec[FEC_DATA_PARITY_COUNT];

static void state_send_data_fec(void);
static void state_recv_data_fec(void);
#endif

static void state_send_pre(void);
static void state_send_sync(void);
static void state_send_size(void);
static void state_send_data(void);
static void state_send_flush(void);
static void state_send_crc_h(void);
static void state_send_crc_l(void);
static void state_send_done(void);
static void state_recv_data(void);
static void state_recv_crc_h(void);
static void state_recv_crc_l(void);
static void state_recv_size(void);
// need reference to this for mac layers
void state_recv_idle(void);
static void state_recv_pre(void);

/** @brief this initializes the error checking/correcting bytes
 * to be used in the mac layer.
 */
inline void cc1000_init_ec (comBuf * sendPkt)
{
   cc1000_send_buf = sendPkt;

#ifdef RADIO_USE_FEC
   fec_init (FEC_DATA_PARITY_COUNT);
   fec_encode (sendPkt->data, sendPkt->size, data_fec);
#else
   //CRC Code
   uint16_t crc;

   //compute crc now, so we don't have to do it in interrupt handler
   crc = crc_compute (sendPkt->data, sendPkt->size);
   cc1000_crc_low = crc & 0x00FF;
   cc1000_crc_high = (crc & 0xFF00) >> 8;
#endif
}

/** @brief routine to start transmission */
inline void cc1000_start_transmit(comBuf * sendPkt)
{
   cc1000_init_ec(sendPkt);

   cc1000_mode(CC1000_MODE_TX); //go to transmit mode

   int_handle = mos_disable_ints();

   cc1000_state = state_send_pre;       //init send state machine

   preamble_count = 1;
   SPDR = PREAMBLE_BYTE;        //start sending first preamble byte

#ifdef INTERRUPT_DRIVEN_SEND
   //INTERRUPT DRIVEN I/O
   //mos_single_threaded();
   SPCR |= (1 << SPIE);         //turn on spi interrupt
   //mos_led_display(1);
   mos_enable_ints(int_handle);
   mos_sem_wait(&cc1000_sem);
   //mos_multi_threaded();
#else
   //POLLING METHOD:
   SPCR &= ~(1 << SPIE);        //turn off spi interrupt
   while(cc1000_state != state_recv_idle) {
      while(!(SPSR & (1 << SPIF)))
         ;
      cc1000_state();
   }
   mos_enable_ints(int_handle);
#endif
   
   if(cc1000_com_mode == IF_LISTEN) {
      SPCR |= (1 << SPIE);
      cc1000_mode(CC1000_MODE_RX);
   }
}

/** @brief Sets mode for this driver.
 * @param md Mode
 * @return New mode set
 */

void com_mode_cc1000(uint8_t md)
{
   switch (md) {
   case IF_OFF:
      //need to take the lock so we don't turn off the radio while
      //it's in use
      SPCR &= ~((1 << SPIE));     //disable spi interrupt
      cc1000_mode (CC1000_MODE_PD);
      cc1000_com_mode = md;
      //TODO: could turn off radio here
      break;

   case IF_STANDBY:
      break;

   case IF_IDLE:
      break;

   case IF_LISTEN:
      cc1000_state = state_recv_idle;   // Start recving in the idle state
      cc1000_com_mode = md;
      cc1000_mode (CC1000_MODE_RX);
      SPCR |= (1 << SPIE);      //turn on spi interrupt
      //TODO: might have to turn on radio here (see above)
      break;
   }
}

void com_mode_IFACE_RADIO (uint8_t md)
{
   mos_mutex_lock(&if_send_mutexes[IFACE_RADIO]);
   com_mode_cc1000(md);
   mos_mutex_unlock(&if_send_mutexes[IFACE_RADIO]);
      
}


/** @brief Generic io control for this driver.
 * @param request i/o request
 * @param ap Arguements
 * @return Always returns 0
*/
void com_ioctl_IFACE_RADIO (uint8_t request, ...)
{
   int arg;
   uint8_t new_freq;
   va_list ap;
   va_start (ap, request);

   switch (request) {
   case CC1000_TX_POWER:       // Adjust transmit power
      arg = va_arg (ap, int);

      cc1000_set_power (arg);
      break;

   case CC1000_GET_TX_POWER: {
      uint8_t* argp = va_arg (ap, uint8_t*);

      *argp = cc1000_get_power();
      break;
   }
   case CC1000_FREQ:           //set frequency
      new_freq = va_arg (ap, int);

      cc1000_change_freq(new_freq);
      break;

   case CC1000_RSSI:           // Get the received signal strength
      // TODO: Get signal strength from dev layer, not as an ioctl
      break;

   }

   va_end (ap);
}

void cc1000_default_init (void)
{
   uint8_t int_handle;
   int_handle = mos_disable_ints();
   
   //get initial receive buffer
   com_swap_bufs(IFACE_RADIO, NULL, &cc1000_recv_buf);
#ifdef GET_RSSI
   cc1000_recv_buf->signal = 0;
#endif
#ifdef TS_PACKET
   cc1000_recv_buf->ts = 0;
#endif
   
#ifdef INTERRUPT_DRIVEN_SEND
   mos_sem_init(&cc1000_sem, 0);
#endif
   
   //init radio
   cc1000_init(FREQ);

   //now init the SPI bus
   DDRB &= ~((1 << DDB0) | (1 << DDB1)); // clock pin to input
   SPCR &= ~((1 << CPOL) | (1 << CPHA)); // Set proper polarity and phase
   DDRB |= (1 << DDB3);                  //miso to output
   DDRB &= ~((1 << DDB0) | (1 << DDB2)); //mosi to input
   cc1000_mode(CC1000_MODE_RX);          //go to recv mode

   SPCR &= ~(1 << SPIE); //disable SPI interrupt
   SPCR |= (1 << SPE);
   
   cc1000_state = state_recv_idle;

   //initially in off mode
   cc1000_com_mode = IF_OFF;

   mos_enable_ints(int_handle);
}

static void state_send_pre(void)
{
   SPDR = PREAMBLE_BYTE;
   preamble_count++;
   if(preamble_count == PREAMBLE_LEN)
      cc1000_state = state_send_sync;
}

static void state_send_sync(void)
{  
   SPDR = 0x33;              //send sync byte
   cc1000_state = state_send_size;
   data_count = 0;
}

static void state_send_size(void)
{
   SPDR = cc1000_send_buf->size;
   #ifdef RADIO_REDUNDANT_SIZE
   // Send redundant size bytes
   if (data_count++ < 2)
   	return;
   #endif
   data_count = 0;
#ifdef RADIO_USE_FEC
   //cc1000_state = STATE_SEND_SIZE_FEC;
   cc1000_state = state_send_data;
#else
   cc1000_state = state_send_data;
#endif   
}

static void state_send_data(void)
{
   SPDR = cc1000_send_buf->data[data_count++];
   if(data_count == cc1000_send_buf->size) {
      //done with data
      data_count = 0;
#ifdef RADIO_USE_FEC
      cc1000_state = state_send_data_fec;
#else
      cc1000_state = state_send_crc_h;
#endif
   }
}

#ifdef RADIO_USE_FEC
static void state_send_data_fec(void)
{
   SPDR = data_fec[data_count++];
   if(data_count == FEC_DATA_PARITY_COUNT)
      cc1000_state = state_send_flush;
}
#endif

static void state_send_crc_h(void)
{
   SPDR = cc1000_crc_high;
   cc1000_state = state_send_crc_l;
}

static void state_send_crc_l(void)
{
   SPDR = cc1000_crc_low;
   cc1000_state = state_send_flush;
}

static void state_send_flush(void)
{
   SPDR = FLUSH_BYTE;
   cc1000_state = state_send_done;
}

static void state_send_done(void)
{
   //go back to recv mode
   cc1000_state = state_recv_idle;
#ifdef INTERRUPT_DRIVEN_SEND

   mos_sem_post(&cc1000_sem);
#endif
}

#ifdef RADIO_USE_FEC
static void state_recv_data_fec(void)
{
   spi_data = SPDR;
   data_fec[data_count++] = (prev_data << offset) | (spi_data >> (8 - offset));
   if(data_count == FEC_DATA_PARITY_COUNT) {
      cc1000_state = state_recv_idle;
      com_swap_bufs(IFACE_RADIO, cc1000_recv_buf, &cc1000_recv_buf);
   }
   prev_data = spi_data;
}
#endif

static void state_recv_crc_l(void)
{
   //read low byte and assemble full crc
   spi_data = SPDR;
   cc1000_crc_low = (prev_data << offset) | (spi_data >> (8 - offset));
   //if crc checks out, swap buffer up to com layer
   if(crc_compute(cc1000_recv_buf->data, cc1000_recv_buf->size) ==
      ((((uint16_t) cc1000_crc_high) << 8) | cc1000_crc_low)) {
      //cc1000_state = state_recv_idle;
      com_swap_bufs(IFACE_RADIO, cc1000_recv_buf, &cc1000_recv_buf);
      cc1000_success_count++;
   } else { //if it's bad, record the error
      //mos_led_toggle(2);
      cc1000_crc_error_count++;
   }
   //either way, we're done and back in idle mode
   cc1000_state = state_recv_idle;
}

static void state_recv_crc_h(void)
{
   spi_data = SPDR;
   cc1000_crc_high = (prev_data << offset) | (spi_data >> (8 - offset));
   //ready to recv low byte
   cc1000_state = state_recv_crc_l;
   prev_data = spi_data;
}

static void state_recv_data(void)
{
   spi_data = SPDR;
   actual_byte = (prev_data << offset) | (spi_data >> (8 - offset));
   cc1000_recv_buf->data[data_count++] = actual_byte;
   
   if (data_count == cc1000_recv_buf->size) {
#ifdef RADIO_USE_FEC
      data_count = 0;
      cc1000_state = state_recv_data_fec;
#else
      cc1000_state = state_recv_crc_h;
#endif
   }
   
   prev_data = spi_data;
}

static void state_recv_size(void)
{
   spi_data = SPDR;
   actual_byte = (prev_data << offset) | (spi_data >> (8 - offset));
#ifdef RADIO_REDUNDANT_SIZE
   // Received redundant size bytes
   size_bytes[data_count] = actual_byte;
   if (data_count++ < 2)
      return;
   // Majority rules
   actual_byte = (size_bytes[0] & size_bytes[1]) |
      (size_bytes[0] & size_bytes[2]) |
      (size_bytes[1] & size_bytes[2]);
#endif
   if (actual_byte > COM_DATA_SIZE || actual_byte == 0) {
      cc1000_size_error_count++;
      cc1000_state = state_recv_idle;
      return;
   } else {
      if (!cc1000_recv_buf) {
	 com_swap_bufs (IFACE_RADIO, NULL, &cc1000_recv_buf);
	 if (!cc1000_recv_buf) {
	    cc1000_mem_error_count++;
	    cc1000_state = state_recv_idle;
	    return;
	 }
      }
   }

   data_count = 0;
   cc1000_recv_buf->size = actual_byte;
   cc1000_state = state_recv_data;
   prev_data = spi_data;
}

static void state_recv_sync(void)
{
   spi_data = SPDR;
   // Figure out the bit offset by shifting until we find the sync byte
   while (prev_data != 0x33) {
      prev_data = (prev_data << 1) | (spi_data >> (7 - offset));
      offset++;
      if (offset >= 8) { //didn't get the sync byte... Something is wrong
	 cc1000_sync_error_count++;
	 cc1000_state = state_recv_idle;
	 return;
      }
   }
   
   // We are synced and ready to start getting the packet
   cc1000_state = state_recv_size;
   prev_data = spi_data;
   data_count = 0;
}

static void state_recv_pre(void)
{
   spi_data = SPDR;
   if (spi_data != PREAMBLE_BYTE && spi_data != 0x55) {
      // not in preamble anymore
      if (preamble_count > PREAMBLE_THRESH) {
	 // reached end of preamble
	 cc1000_state = state_recv_sync;
	 offset = 0;
      } else {
	 // looking at noise
	 cc1000_state = state_recv_idle;
      }
   } else { // seeing more of the preamble
      preamble_count++;
      //ANS hack: Can loose a few preambles since the transmitter
      //sends out 12 preamble bytes and the recviever needs only 6
      
#ifdef GET_RSSI
      //First check whether the signal value is already not filled in
      if (cc1000_recv_buf->signal == 0) {
	 cc1000_rssi_on();
	 rssival = rssi_poll();
	 cc1000_rssi_off();
	 cc1000_recv_buf->signal = rssival;
	 //ANS hack ends
      }
#endif
#ifdef TS_PACKET
      if(cc1000_recv_buf->ts == 0) {
	 cc1000_recv_buf->ts = *real_timer_get_ticks();
	 cc1000_recv_buf->tcnt = TCNT2;
      }
#endif
   }
   prev_data = spi_data;
}

void state_recv_idle(void)
{
   spi_data = SPDR;
   if (spi_data == PREAMBLE_BYTE || spi_data == 0x55) {
#ifdef GET_RSSI
      //ANS hack:
      //We want to initilize the rssival of the cc1000_recv_buf to 0
      cc1000_recv_buf->signal = 0;
      //ANS hack ends
#endif
      
#ifdef TS_PACKET
      cc1000_recv_buf->ts = 0;
#endif
      preamble_count = 0;
      cc1000_state = state_recv_pre;
   }
}

// *****   Radio Configuration Functions *************/

// Global variables for the current frequency and the power level

/** @brief Current frequency. */
uint8_t cur_freq;

/** @brief Current power level. */
uint8_t rf_power;
static uint8_t cc1000Mode;

#define PALE_HIGH() CONFIG_PORT |= (1 << PALE)
#define PALE_LOW() CONFIG_PORT &= ~(1 << PALE)
#define PCLK_HIGH() CONFIG_PORT |= (1 << PCLK)
#define PCLK_LOW() CONFIG_PORT &= ~(1 << PCLK)
#define PDATA_HIGH() CONFIG_PORT |= (1 << PDATA)
#define PDATA_LOW() CONFIG_PORT &= ~(1 << PDATA)

void cc1000_init (uint8_t freq)
{
   uint8_t int_handle;

   int_handle = mos_disable_ints();
   cur_freq = freq; // record the current setting frequency

   // record the current setting power
#ifdef PLATFORM_NYMPH
   rf_power = PA_POW_PARAM;
   // Set the three configuration pin high
   CONFIG_PORT_DIR |= ((1 << PALE) | (1 << PCLK) | (1 << PDATA));
   //PORTC |= ( (1<<PALE) | (1<<PCLK) | (1<<PDATA)); 
#elif defined(PLATFORM_MICA2) || defined(PLATFORM_MICA2DOT)
   rf_power = 0xff;
   //DDRA &= 0xBF;
   //DDRB &= 0x8F;
   //DDRA = 0;
   //DDRB = 0;
   CONFIG_PORT_DIR |= ((1 << PALE) | (1 << PCLK) | (1 << PDATA));
   //CONFIG_PORT &= ~((1 << PALE) | (1 << PCLK) | (1 << PDATA));
#else
   #error "Unimplemented platform"
#endif

   // Make this three pin outputs*/
   //DDRC |= ( (1<<PALE) | (1<<PCLK) | (1<<PDATA) );

   // Set the three configuration pin high
   PALE_HIGH();
   PCLK_HIGH();
   PDATA_HIGH();

   // Reset and turn on the crystal oscillator core
   // 0x3A
   // RXTX = 0 F_REG = 0 RX_PD = 1 TX_PD = 1 FS_PD = 1 CORE_PD = 0
   // BIAS_PD = 1 RESET_N = 0
   cc1000_write(CC1000_MAIN, ((1 << CC1000_RX_PD) |
                              (1 << CC1000_TX_PD) |
                              (1 << CC1000_FS_PD) |
			      (1 << CC1000_BIAS_PD)));

   // RESET
   cc1000_write(CC1000_MAIN, ((1 << CC1000_RX_PD)   |
                              (1 << CC1000_TX_PD)   |
                              (1 << CC1000_FS_PD)   |
                              (1 << CC1000_BIAS_PD) |
			      (1 << CC1000_RESET_N)));
   // wait 2 ms
   // Time to wait depends on crystal frequency and the load capacitance
   mos_udelay(2000);

   // Program all registers, Calibrate VCO and PLL and set the frequency
   cc1000_set(freq);

   //ANS: Start the clock so that we can start timestamping the packets
#ifdef TS_PACKET
   real_timer_init();
   real_timer_clear();
#endif
   
   // Setup default baud rate
   UBRR0H = (uint8_t)(DEFAULT_BAUD_RATE >> 8);
   UBRR0L = (uint8_t)(DEFAULT_BAUD_RATE);

   // enable transmitter and receiver
   UCSR0B = (1 << RXEN0) | (1 << TXEN0);
   
   // no parity, 8 bits
   UCSR0C = (3 << UCSZ00);
   
   mos_enable_ints(int_handle);
}

inline uint8_t cc1000_get_mode (void)
{
   return cc1000Mode;
}

void cc1000_change_freq (uint8_t new_freq)
{
   int i;
   uint8_t j;

   // record the current setting frequency    
   cur_freq = new_freq;

   // write the corresponding FREQ registers
   for(i = 1; i < 7; i++) {
      j = pgm_read_byte(&cc1000_params[cur_freq][i - 1]);
      cc1000_write(i, j);
   }

   //recalibrate after setting the new frequency
   cc1000_calibrate();
}

void cc1000_set (uint8_t new_freq)
{
   int i;
   uint8_t j;

   // record the current setting frequency    
   cur_freq = new_freq;

   // record the current setting power
   rf_power = PA_POW_PARAM;

   // write the corresponding FREQ registers
   for(i = 1; i < 7; i++) {
      j = pgm_read_byte(&cc1000_params[cur_freq][i - 1]);
      cc1000_write(i, j);
   }

   // First write the template to the registers
   //for (i=7;i<29;i++)
   // cc1000_write(i, cc1000_para_template[i] );
   cc1000_write(CC1000_FSEP1, FSEP1_PARAM);
   cc1000_write(CC1000_FSEP0, FSEP0_PARAM);
   cc1000_write(CC1000_CURRENT, RX_CURRENT_PARAM);
   cc1000_write(CC1000_FRONT_END, FRONT_END_PARAM);
   cc1000_write(CC1000_PA_POW, PA_POW_PARAM);
   cc1000_write(CC1000_PLL, PLL_RX_PARAM);
   cc1000_write(CC1000_LOCK, LOCK_PARAM);
   cc1000_write(CC1000_CAL, CAL_PARAM);
   cc1000_write(CC1000_MODEM2, MODEM2_PARAM);
   cc1000_write(CC1000_MODEM1, MODEM1_PARAM);
   cc1000_write(CC1000_MODEM0, MODEM0_PARAM);
   cc1000_write(CC1000_MATCH, MATCH_PARAM);
   cc1000_write(CC1000_FSCTRL, FSCTRL_PARAM);
   cc1000_write(CC1000_FSHAPE7, FSHAPE7_PARAM);
   cc1000_write(CC1000_FSHAPE6, FSHAPE6_PARAM);
   cc1000_write(CC1000_FSHAPE5, FSHAPE5_PARAM);
   cc1000_write(CC1000_FSHAPE4, FSHAPE4_PARAM);
   cc1000_write(CC1000_FSHAPE3, FSHAPE3_PARAM);
   cc1000_write(CC1000_FSHAPE2, FSHAPE2_PARAM);
   cc1000_write(CC1000_FSHAPE1, FSHAPE1_PARAM);
   cc1000_write(CC1000_FSDELAY, FSDELAY_PARAM);
   cc1000_write(CC1000_TEST4, TEST4_PARAM);
   cc1000_write(CC1000_PRESCALER, PRESCALER_PARAM);

   //recalibrate after setting the new frequency
   cc1000_calibrate();

   //set the cc1000 to receive mode
   cc1000_mode(CC1000_MODE_RX);
}

uint8_t cc1000_get_channel (void)
{
   return cur_freq;
}

uint8_t cc1000_get_power (void)
{
   return rf_power;
}


// TODO: Test and debug this function!
uint8_t cc1000_read (uint8_t addr)
{
   uint8_t i;
   uint8_t val = 0;
   uint8_t int_handle;

   int_handle = mos_disable_ints();

   PALE_HIGH();
   
   addr <<= 1; // We only want the lower 7 bits
   PALE_LOW(); 

   // Send 7 address bits
   for(i = 0; i < 7; i++) {
      PCLK_HIGH();
      
      if(addr & 0x80)
         PDATA_HIGH();  
      else
         PDATA_LOW(); 

      addr <<= 1;

      // Now cycle PCLK to write the bit
      PCLK_LOW();     
   }

   // Send read bit
   PCLK_HIGH();
   PDATA_LOW();       
   PCLK_LOW();
   
   PCLK_HIGH();
   PALE_HIGH(); 
   PDATA_HIGH();
   
   CONFIG_PORT_DIR &= ~(1 << PDATA); // Set up PDATA as an input
   
   // Receive data bits
   for(i = 0; i < 8; i++) {
      PCLK_LOW();
      
      if(CONFIG_PORT_PIN & (1 << PDATA))
         val = (val << 1) | 0x01;
      else
         val = (val << 1) & 0xFE;

      PCLK_HIGH();     
   }
   
   PALE_HIGH();
   CONFIG_PORT_DIR |= (1 << PDATA);     // Set up PDATA as an output again
   //CONFIG_PORT |= (1 << PDATA);
   PDATA_HIGH();

   mos_enable_ints(int_handle);

   return val;
}

void cc1000_write (uint8_t addr, uint8_t val)
{
   uint8_t i;
   uint8_t int_handle;

   int_handle = mos_disable_ints();

   addr <<= 1;                  // We only want the lower 7 bits   
   PALE_LOW();

   // Send 7 address bits 
   for(i = 0; i < 7; i++) {
      if(addr & 0x80)
         PDATA_HIGH();   
      else
         PDATA_LOW(); 

      /* Now cycle PCLK to write the bit. */
      PCLK_LOW();  
      PCLK_HIGH(); 
      addr <<= 1;
   }

   // Send write bit 
   PDATA_HIGH();
   PCLK_LOW();
   PCLK_HIGH();
   PALE_HIGH(); 

   // Send 8 data bits    
   for(i = 0; i < 8; i++) {
      if(val & 0x80)
         PDATA_HIGH();   
      else
         PDATA_LOW();

      // Now cycle PCLK to write the bit
      PCLK_LOW(); 
      PCLK_HIGH(); 
      val <<= 1;
   }

   // Set the three configuration pins high 
   PALE_HIGH();
   PDATA_HIGH();
   PCLK_HIGH();

   mos_enable_ints(int_handle);
}

void cc1000_reset (void)
{
   uint8_t mainValue;

   mainValue = cc1000_read(CC1000_MAIN);
   // Reset CC1000 the reset pin to 0
   cc1000_write(CC1000_MAIN, mainValue & 0xFE);
   cc1000_write(CC1000_MAIN, mainValue | 0x01); // Bring CC1000 out of reset
}

void cc1000_calibrate (void)
{
   //the following procedure are follwing the data sheet

   // Write FREQ_A, FREQ_B If DR>=38kBd then write
   // TEST4: L2KIO=3Fh Write CAL: CAL_DUAL = 0
   cc1000_write(CC1000_PA_POW, 0x00);   // turn off radio power
   cc1000_write(CC1000_TEST4, TEST4_PARAM);

   // RX frequency register A is calibrated first
   // Write MAIN: RXTX = 0; F_REG = 0 RX_PD = 0; TX_PD = 1;
   // FS_PD = 0 CORE_PD = 0; BIAS_PD = 0; RESET_N=1
   cc1000_write(CC1000_MAIN, ((1 << CC1000_TX_PD) | (1 << CC1000_RESET_N)));

   //  RX current  is the VCO current to be used in RX mode
   cc1000_write(CC1000_CURRENT, RX_CURRENT_PARAM);

   // Calibration is performed in RX mode, Result is
   // stored in TEST0 and TEST2, RX register
   // Write CAL: CAL_START=1, set the wait
   cc1000_write(CC1000_CAL, ((1 << CC1000_CAL_START) |
                             (1 << CC1000_CAL_WAIT) |
                             (6 << CC1000_CAL_ITERATE)));

   while((cc1000_read(CC1000_CAL) & (1 << CC1000_CAL_COMPLETE)) == 0);

   // Write CAL: CAL_START=0
   cc1000_write(CC1000_CAL, ((1 << CC1000_CAL_WAIT) |
                             (6 << CC1000_CAL_ITERATE)));


   // TX frequency register B is calibrated second
   // Write MAIN: RXTX = 1; F_REG = 1 RX_PD = 1; TX_PD = 0;
   // FS_PD = 0 CORE_PD = 0; BIAS_PD = 0; RESET_N=1
   cc1000_write(CC1000_MAIN, ((1 << CC1000_RXTX) |
                              (1 << CC1000_F_REG) |
                              (1 << CC1000_RX_PD) | (1 << CC1000_RESET_N)));

   //  TX current  is the VCO current to be used in TX mode
   cc1000_write(CC1000_CURRENT, TX_CURRENT_PARAM);

   //PA is turned off to prevent spurious emission
   cc1000_write(CC1000_PA_POW, 0x00);

   // Write CAL: CAL_START=1
   cc1000_write(CC1000_CAL, ((1 << CC1000_CAL_START) |
                             (1 << CC1000_CAL_WAIT) |
                             (6 << CC1000_CAL_ITERATE)));

   while((cc1000_read(CC1000_CAL) & (1 << CC1000_CAL_COMPLETE)) == 0);

   // Write CAL: CAL_START=0
   cc1000_write(CC1000_CAL, ((1 << CC1000_CAL_WAIT) |
                             (6 << CC1000_CAL_ITERATE)));

}

void cc1000_set_power(uint8_t power)
{
   rf_power = power;
   // TODO it would be better to set the TX power only when we're transmitting
   cc1000_write(CC1000_PA_POW, power);
   mos_udelay(250);
}

void cc1000_mode (uint8_t mode)
{
   switch (mode) {
   case CC1000_MODE_RX:
      cc1000Mode = CC1000_MODE_RX;
      // Procedures on the data sheet 
      // Turn on RX: RX_PD = 0, FS_PD = 0 RX_PD = 0, FS_PD = 0
      cc1000_write(CC1000_MAIN, ((1 << CC1000_TX_PD) | (1 << CC1000_RESET_N)));

      cc1000_write(CC1000_PA_POW, 0);

      // Writing current
      cc1000_write(CC1000_CURRENT, RX_CURRENT_PARAM);
      // Wait 250 us
      mos_udelay(250);
      break;
   case CC1000_MODE_TX:
      cc1000Mode = CC1000_MODE_TX;
      // Turn on TX: PA_POW = 00h MAIN: RXTX = 1, F_REG = 1 TX_PD = 0,
      // FS_PD = 0
      cc1000_write(CC1000_MAIN, ((1 << CC1000_RXTX) |
                                 (1 << CC1000_F_REG) |
                                 (1 << CC1000_RX_PD) | (1 << CC1000_RESET_N)));
      // CURRENT =  TX current  
      cc1000_write(CC1000_CURRENT, TX_CURRENT_PARAM);
      // Wait 250 us
      mos_udelay(250);
      // PA_POW =  Output power 
      cc1000_write(CC1000_PA_POW, rf_power);
      // Wait 20 us
      mos_udelay(250);
      //mos_udelay (20);
      break;

   case CC1000_MODE_PD:
      cc1000_write(CC1000_MAIN, 0x3F);  // Put CC1000 into power-down
      cc1000_write(CC1000_PA_POW, 0x00); // Turn off PA to minimise power
      break;
   case CC1000_MODE_SLEEP:
      // put the cc1000 to sleep
      cc1000_write(CC1000_MAIN, ((1 << CC1000_RX_PD) |
                                 (1 << CC1000_TX_PD) |
                                 (1 << CC1000_FS_PD) |
                                 (1 << CC1000_CORE_PD) |
                                 (1 << CC1000_BIAS_PD) |
                                 (1 << CC1000_RESET_N)));
      break;
   }
}

void cc1000_rssi_on (void)
{
   cc1000_write(CC1000_FRONT_END, 0x32);
}

void cc1000_rssi_off (void)
{
   cc1000_write(CC1000_FRONT_END, 0x30);
}

void cc1000_wakeup (void)
{
   cc1000_on();
   mos_udelay(500);
}

void cc1000_on (void)
{
   cc1000_write(CC1000_MAIN, ((1 << CC1000_RX_PD) |
                              (1 << CC1000_TX_PD) |
                              (1 << CC1000_FS_PD) |
                              (1 << CC1000_BIAS_PD) | (1 << CC1000_RESET_N)));

   mos_udelay(2000);
   cc1000_mode(CC1000_MODE_RX);
}

#endif
#endif
