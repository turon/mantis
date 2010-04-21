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

/*
  Project Mantis
  File:   uart.c
  Authors: Jeff Rose
  Date:   03-01-04
  
  Operation in both com mode, using the MOS com layer and a raw mode
    are supported at compilation time. In order to turn on raw mode
    (off by default), #define RAW_MODE_UARTX, where X is the UART you
    want to be in raw mode. This used to be a runtime option, but it
    can be implemented more efficiently as a compile time option.

  In raw mode, if you know you will only be accessing the UART from a
    single thread, you may use the *_nolock functions, which will not
    lock a mutex when accessing the UART.
*/

#include "mos.h"

#ifndef PLATFORM_LINUX

#include "msched.h"
#include "uart.h"
#include "sem.h"
#include "com.h"
#include "mutex.h"
#include "plat_dep.h"
#ifdef PLATFORM_TELOSB
#include "plat_clock.h"
#endif

#ifdef MOS_DEBUG
#include "mos_debugging.h"
#endif

#if defined(UART) || !defined(SCONS)


static uint8_t uart_raw0, uart_raw1;

/* Receive states and state variables. */
#define STATE_SEND_PREAMBLE 0
#define STATE_SEND_SIZE     1
#define STATE_SEND_DATA     2

/* Send states and state variables. */
#define STATE_RECV_IDLE      0
#define STATE_RECV_PREAMBLE  1
#define STATE_RECV_SIZE      2
#define STATE_RECV_DATA      3

static void (*tx_state0)(void);
static void (*rx_state0)(void);
static void (*tx_state1)(void);
static void (*rx_state1)(void);

/* Buf pointers & indices for sending and receiving. */
/** @brief Buf pointer for receiving and sending. */
static comBuf *rx_buf0;
static comBuf *tx_buf0;
static comBuf *rx_buf1;
static comBuf *tx_buf1;

static uint8_t rx_index0;
static uint8_t tx_index0;
static uint8_t tx_index1;
static uint8_t rx_index1;

/** @brief A thread pointer to keep the id of a blocked send thread. */
static mos_sem_t tx_sem0;
static mos_sem_t tx_sem1;
static mos_sem_t rx_sem0;
static mos_sem_t rx_sem1;

#ifdef NEED_RAW_MODE
static uint8_t rx_raw_max0;
static uint8_t *rx_raw_buf0;
static uint8_t tx_raw_max0;
static uint8_t *tx_raw_buf0;
static uint8_t rx_raw_max1;
static uint8_t *tx_raw_buf1;
static uint8_t tx_raw_max1;
static uint8_t *rx_raw_buf1;
#endif

#ifdef ARCH_AVR

#define UART_ENABLE_RX_INT0() (UCSR0B |= (1 << RXCIE0) | (1 << RXEN0))
#define UART_ENABLE_RX_INT1() (UCSR1B |= (1 << RXCIE1) | (1 << RXEN1))
#define UART_DISABLE_RX_INT0() (UCSR0B &= ~((1 << RXCIE0) | (1 << RXEN0)))
#define UART_DISABLE_RX_INT1() (UCSR1B &= ~((1 << RXCIE1) | (1 << RXEN1)))

#define UART_ENABLE_TX_INT0() (UCSR0B |= (1 << TXEN0) | (1 << UDRIE0))
#define UART_ENABLE_TX_INT1() (UCSR1B |= (1 << TXEN1) | (1 << UDRIE1))
#define UART_DISABLE_TX_INT0() (UCSR0B &= ~((1 << TXEN0) | (1 << UDRIE0)))
#define UART_DISABLE_TX_INT1() (UCSR1B &= ~((1 << TXEN1) | (1 << UDRIE1)))


#define UART_WAIT_FOR_TXC(uart_num)                            \
   while(!(UCSR##uart_num##A & (1 << TXC##uart_num)));         \
   UCSR##uart_num##A |= (1 << TXC##uart_num);

#define TX_REG0 UDR0
#define RX_REG0 UDR0
#define TX_REG1 UDR1
#define RX_REG1 UDR1

#elif defined(PLATFORM_TELOSB)

#define UART_ENABLE_RX_INT0() (IE1 |= URXIE0)
#define UART_ENABLE_RX_INT1() (IE2 |= URXIE1)
#define UART_DISABLE_RX_INT0() (IE1 &= ~URXIE0)
#define UART_DISABLE_RX_INT1() (IE2 &= ~URXIE1)

#define UART_ENABLE_TX_INT0() (IE1 |= UTXIE0)
#define UART_ENABLE_TX_INT1() (IE2 |= UTXIE1)

#define UART_DISABLE_TX_INT0() (IE1 &= ~UTXIE0)
#define UART_DISABLE_TX_INT1() (IE2 &= ~UTXIE1)

#define UART_WAIT_FOR_TXC(uart_num)                       \
    while(!(U##uart_num##TCTL & (TXEPT)));                \
    U##uart_num##TCTL |= (TXEPT);
    

#define TX_REG0 U0TXBUF
#define TX_REG1 U1TXBUF
#define RX_REG0 U0RXBUF
#define RX_REG1 U1RXBUF

#else
#error This platform needs a UART implementation
#endif

// here are the prototypes of the functions in the state machines
static void rx_state_preamble0(void);
static void tx_state_preamble0(void);
static void rx_state_size0(void);
static void tx_state_size0(void);
static void rx_state_data0(void);
static void tx_state_data0(void);

static void rx_state_preamble1(void);
static void tx_state_preamble1(void);
static void rx_state_size1(void);
static void tx_state_size1(void);
static void rx_state_data1(void);
static void tx_state_data1(void);

#ifdef NEED_RAW_MODE
static void rx_raw0(void);
static void tx_raw0(void);
static void rx_raw1(void);
static void tx_raw1(void);

#define uart_read_helper(uart_num, buf, count)			\
   uint8_t ret = 0;						\
   mos_mutex_lock(&if_send_mutexes[IFACE_SERIAL + uart_num]);	\
   rx_raw_max##uart_num = count;				\
   rx_index##uart_num = 0;					\
   rx_raw_buf##uart_num = buf;					\
   UART_ENABLE_RX_INT##uart_num();				\
   mos_sem_wait(&rx_sem##uart_num);				\
   UART_DISABLE_RX_INT##uart_num();				\
   ret = rx_index##uart_num;					\
   mos_mutex_unlock(&if_send_mutexes[IFACE_SERIAL + uart_num]);	\
   return ret;

#define uart_read_nolock_helper(uart_num, buf, count)	\
   uint8_t ret = 0;					\
   rx_raw_max##uart_num = count;			\
   rx_index##uart_num = 0;				\
   rx_raw_buf##uart_num = buf;				\
   UART_ENABLE_RX_INT##uart_num();			\
   mos_sem_wait(&rx_sem##uart_num);			\
   UART_DISABLE_RX_INT##uart_num();			\
   ret = rx_index##uart_num;				\
   return ret;

#define uart_write_helper(uart_num, buf, count)			\
   uint8_t ret = 0;						\
   mos_mutex_lock(&if_send_mutexes[IFACE_SERIAL + uart_num]);	\
   tx_raw_max##uart_num = count;				\
   tx_index##uart_num = 0;					\
   tx_raw_buf##uart_num = buf;					\
   UART_ENABLE_TX_INT##uart_num();				\
   mos_sem_wait(&tx_sem##uart_num);				\
   UART_DISABLE_TX_INT##uart_num();				\
   ret = tx_index##uart_num;					\
   mos_mutex_unlock(&if_send_mutexes[IFACE_SERIAL + uart_num]);	\
   return ret;

#define uart_write_nolock_helper(uart_num, buf, count)	\
   uint8_t ret = 0;					\
   tx_raw_max##uart_num = count;			\
   tx_index##uart_num = 0;				\
   tx_raw_buf##uart_num = buf;				\
   UART_ENABLE_TX_INT##uart_num();			\
   mos_sem_wait(&tx_sem##uart_num);			\
   UART_DISABLE_TX_INT##uart_num();			\
   ret = tx_index##uart_num;				\
   return ret;

uint8_t uart_read0(uint8_t *buf, uint8_t count)
{
   uart_read_helper(0, buf, count);
}

uint8_t uart_read_nolock0(uint8_t *buf, uint8_t count)
{
   uart_read_nolock_helper(0, buf, count);
}

uint8_t uart_write0(uint8_t *buf, uint8_t count)
{
   uart_write_helper(0, buf, count);
}

uint8_t uart_write_nolock0(uint8_t *buf, uint8_t count)
{
   uart_write_nolock_helper(0, buf, count);
}

uint8_t uart_read1(uint8_t *buf, uint8_t count)
{
   uart_read_helper(1, buf, count);   
}

uint8_t uart_read_nolock1(uint8_t *buf, uint8_t count)
{
   uart_read_nolock_helper(1, buf, count);
}

uint8_t uart_write1(uint8_t *buf, uint8_t count)
{
   uart_write_helper(1, buf, count);
}

uint8_t uart_write_nolock1(uint8_t *buf, uint8_t count)
{
   uart_write_nolock_helper(1, buf, count);
}

#endif

#ifdef ARCH_AVR
static void uart_set_baud(uint8_t uart_num, uint16_t baud_rate)
{
   if(uart_num == UART0) {
      UBRR0H = (uint8_t)(baud_rate >> 8);
      UBRR0L = (uint8_t)(baud_rate);
   } else {
      UBRR1H = (uint8_t)(baud_rate >> 8);
      UBRR1L = (uint8_t)(baud_rate);
   }
}

void uart_init(void)
{
   // Setup the default baud rate
   uart_set_baud(UART0, DEFAULT_BAUD_RATE);
   uart_set_baud(UART1, B4800);
#ifdef NEED_RAW_MODE  
   uart_raw0 = 1;
   uart_raw1 = 1;
#else
   uart_raw0 = 0;
   uart_raw1 = 0;
#endif
   
   // async, no parity, 8 bits
#ifdef CLOCK_SPEED_4_0
   UCSR0A |= (1 << U2X);
   UCSR1A |= (1 << U2X);
#endif
   UCSR0C = (3 << UCSZ00);
   UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);

   // Turn off the transmitter and receiver by default
   UCSR0B = 0;
   UCSR1B = 0;
   
   mos_sem_init(&tx_sem0, 0);
   mos_sem_init(&tx_sem1, 0);
   mos_sem_init(&rx_sem0, 0);
   mos_sem_init(&rx_sem1, 0);

   rx_state0 = rx_state_preamble0;
   tx_state0 = tx_state_preamble0;
   com_swap_bufs(IFACE_SERIAL, (comBuf *)NULL, &rx_buf0);

   rx_state1 = rx_state_preamble1;
   tx_state1 = tx_state_preamble1;
   com_swap_bufs(IFACE_SERIAL2, (comBuf *)NULL, &rx_buf1);
}

#elif defined(PLATFORM_TELOSB)


#define uart_set_baud(uart_num, baud_rate) {		\
      if(uart_num == UART0) {				\
	 UMCTL0 = UMCTL_INIT;				\
	 UBR10  = UBR1_INIT;				\
	 UBR00  = UBR0_INIT;				\
      } else {						\
	 UMCTL1 = UMCTL_INIT;				\
	 UBR11  = UBR1_INIT;				\
	 UBR01  = UBR0_INIT;				\
      }							\
   }


 
void uart_init(void)
{
   // enter reset mode
   U0CTL = SWRST;
   U1CTL = SWRST;

   // 8 bit data, no parity
   U0CTL |= CHAR;
   U1CTL |= CHAR;

   // source off SMCLK
   U0TCTL = SSEL_2;
   U1TCTL = SSEL_2;
   
   uart_set_baud(UART0, DEFAULT_BAUD_RATE);
   uart_set_baud(UART1, DEFAULT_BAUD_RATE);

   // enable modules
   ME1 |= UTXE0 | URXE0;
   ME2 |= UTXE1 | URXE1;
   
   // init recv control registers
   U0RCTL = 0;
   U1RCTL = 0;
   
   // set peripheral mode for uart tx/rx pins
   P3SEL |= (1 << 4) | (1 << 5);
   P3SEL |= (1 << 6) | (1 << 7);

   // enable interrupts (still in reset state)
   IE1 |= URXIE0;
   IE2 |= URXIE1;
   //printf("uart.c IE1 = %x\nIE2 = %x\n", (int)IE1, (int)IE2);
   
   
   mos_sem_init(&rx_sem0, 0);
   mos_sem_init(&rx_sem1, 0);
   mos_sem_init(&tx_sem0, 0);
   mos_sem_init(&tx_sem1, 0);

   rx_state0 = rx_state_preamble0;
   tx_state0 = tx_state_preamble0;
   com_swap_bufs(IFACE_SERIAL, (comBuf *)NULL, &rx_buf0);

   rx_state1 = rx_state_preamble1;
   tx_state1 = tx_state_preamble1;
   com_swap_bufs(IFACE_SERIAL2, (comBuf *)NULL, &rx_buf1);

   // clear reset state
   U0CTL &= ~SWRST;
   U1CTL &= ~SWRST;
}

#else
#error A UART driver has not been implemented for this platform.
#endif

//#define uart_polling_mode
#ifdef uart_polling_mode

#define com_send_helper(uart_num, buf, raw)			\
   handle_t int_handle;						\
   mos_mutex_lock(&if_send_mutexes[IFACE_SERIAL + uart_num]);	\
   int_handle = mos_disable_ints();				\
   tx_state##uart_num = tx_state_preamble##uart_num;		\
   tx_buf##uart_num = buf;					\
   tx_index##uart_num = 0;					\
   if (!raw){							\
      while(tx_index##uart_num < PREAMBLE_SIZE)			\
      {								\
	 TX_REG##uart_num = PREAMBLE;				\
	 UART_WAIT_FOR_TXC(uart_num);				\
	 tx_index##uart_num++;					\
      }								\
      tx_index##uart_num = 0;					\
      TX_REG##uart_num = tx_buf##uart_num->size;		\
      UART_WAIT_FOR_TXC(uart_num);				\
   }								\
   while(tx_index##uart_num < tx_buf##uart_num->size)		\
   {                                                            \
      TX_REG##uart_num = tx_buf##uart_num->data[tx_index##uart_num]; \
      UART_WAIT_FOR_TXC(uart_num);                              \
      tx_index##uart_num++;                                     \
   }                                                            \
   mos_enable_ints(int_handle);					\
   mos_mutex_unlock(&if_send_mutexes[IFACE_SERIAL + uart_num]); 
#else
#define com_send_helper(uart_num, buf, raw)			\
   handle_t int_handle;						\
   mos_mutex_lock(&if_send_mutexes[IFACE_SERIAL + uart_num]);	\
   int_handle = mos_disable_ints();				\
   tx_buf##uart_num = buf;					\
   tx_index##uart_num = 1;					\
   if (!raw) tx_state##uart_num = tx_state_preamble##uart_num;	\
   else tx_state##uart_num = tx_state_data##uart_num;		\
   if (!raw) TX_REG##uart_num = PREAMBLE;			\
   else TX_REG##uart_num = tx_buf##uart_num->data[0];		\
   UART_ENABLE_TX_INT##uart_num();				\
   mos_enable_ints(int_handle);					\
   mos_sem_wait(&tx_sem##uart_num);				\
   mos_mutex_unlock(&if_send_mutexes[IFACE_SERIAL + uart_num]); 
#endif

/** @brief Uart send function. 
 * @param buf Buffer to use
 * @return Always returns 0
 */
uint8_t com_send_IFACE_SERIAL(comBuf *buf)
{
/*#ifdef MOS_DEBUG
   mos_disable_debugging();
#endif
*/ 
   com_send_helper(0, buf, uart_raw0);

/*#ifdef MOS_DEBUG
   mos_enable_debugging();
#endif
*/ 
   return 0;
}

/** @brief Uart send function. 
 * @param buf Buffer to use
 * @return Always returns 0
 */
uint8_t com_send_IFACE_SERIAL2(comBuf *buf)
{
/*#ifdef MOS_DEBUG
   mos_disable_debugging();
#endif
*/
   com_send_helper(1, buf, uart_raw1);

/*#ifdef MOS_DEBUG
   mos_enable_debugging();
#endif
*/ 
   return 0;
}

#ifdef NEED_RAW_MODE
#define com_ioctl_helper_raw(uart_num)				\
   /*uint8_t baud_rate;*/					\
   va_list ap;							\
   uint16_t arg;						\
   handle_t handle;						\
   va_start(ap, request);					\
   switch(request) {						\
   case UART_IOCTL_BAUD_RATE:					\
      /*baud_rate = (uint8_t)va_arg(ap, int);*/			\
      /*uart_set_baud(uart_num, 57600);*/			\
      break;							\
   case UART_IOCTL_RAW_MODE##uart_num:				\
      arg = va_arg(ap, uint16_t);				\
      handle = mos_disable_ints();				\
      uart_raw##uart_num = arg;					\
      rx_index##uart_num = 0;					\
      mos_enable_ints(handle);					\
      break;							\
   case UART_IOCTL_COM_MODE##uart_num:				\
      arg = va_arg(ap, uint16_t);				\
      handle = mos_disable_ints();				\
      tx_state##uart_num = tx_state_preamble##uart_num;		\
      rx_state##uart_num = rx_state_preamble##uart_num;		\
      rx_index##uart_num = 0;					\
      mos_enable_ints(handle);					\
      break;							\
   }								\
   va_end(ap);

#else

#define com_ioctl_helper(uart_num)			\
   /*uint8_t baud_rate;*/				\
   va_list ap;						\
   va_start(ap, request);				\
   switch(request) {					\
   case UART_IOCTL_BAUD_RATE:				\
      /*baud_rate = (uint8_t)va_arg(ap, int);*/		\
      /*uart_set_baud(uart_num, 57600);*/		\
      break;						\
   }							\
   va_end(ap);

#endif

/** @brief Uart IO control function.
 * @return Always returns 0
 */
void com_ioctl_IFACE_SERIAL(uint8_t request, ...)
{
#ifdef NEED_RAW_MODE
   com_ioctl_helper_raw(0);
#else
   com_ioctl_helper(0);
#endif
}

/** @brief Uart IO control function.
 * @return Always returns 0
 */
void com_ioctl_IFACE_SERIAL2(uint8_t request, ...)
{
#ifdef NEED_RAW_MODE
   com_ioctl_helper_raw(1);
#else
   com_ioctl_helper(1);
#endif
}

#define com_mode_helper(uart_num, mode)				\
   switch(mode) {						\
   case IF_OFF:							\
   case IF_STANDBY:						\
   case IF_IDLE:						\
      UART_DISABLE_RX_INT##uart_num();				\
      rx_state##uart_num = rx_state_preamble##uart_num;		\
      break;							\
   case IF_LISTEN:						\
      /* Start recving in the idle state */			\
      rx_state##uart_num = rx_state_preamble##uart_num;		\
      rx_index##uart_num = 0;					\
      /* Turn on the receiver and rx interrupt*/		\
      UART_ENABLE_RX_INT##uart_num();				\
      break;							\
   default:							\
      break;							\
   }

/** @brief Uart mode function. 
 * @param mode Mode
 * @return Always returns 0
 */
void com_mode_IFACE_SERIAL(uint8_t mode)
{
#ifndef RAW_MODE_UART0
   com_mode_helper(0, mode);
#endif
}

/** @brief Uart mode function. 
 * @param mode Mode
 * @return Always returns 0
 */
void com_mode_IFACE_SERIAL2(uint8_t mode)
{
#ifndef RAW_MODE_UART1
   com_mode_helper(1, mode);
#endif
}

/********************************************************************
 ********************************************************************
 ************  Here begins the state machines ***********************
 ********************************************************************
 ********************************************************************/


#define rx_state_preamble_helper(uart_num, reg)				\
   if(reg == PREAMBLE)							\
      rx_index##uart_num++;						\
   else									\
      rx_index##uart_num = 0;						\
   if(rx_index##uart_num == PREAMBLE_SIZE) {				\
      if(rx_buf##uart_num)						\
	 rx_state##uart_num = rx_state_size##uart_num;			\
      else {								\
	 if(rx_buf##uart_num)						\
	    rx_state##uart_num = rx_state_size##uart_num;		\
	 else								\
	    rx_state##uart_num = rx_state_preamble##uart_num;		\
	 com_swap_bufs(IFACE_SERIAL + uart_num, NULL, &rx_buf##uart_num); \
      }									\
   }

#define rx_state_size_helper(uart_num, reg)			\
   rx_buf##uart_num->size = reg;				\
   if(rx_buf##uart_num->size > COM_DATA_SIZE) {			\
      rx_state##uart_num = rx_state_preamble##uart_num;		\
   } else {							\
      rx_state##uart_num = rx_state_data##uart_num;		\
      rx_index##uart_num = 0;					\
   }

#define rx_state_data_helper(uart_num, reg)			\
   rx_buf##uart_num->data[rx_index##uart_num++] = reg;		\
   if(rx_index##uart_num == rx_buf##uart_num->size) {		\
      rx_state##uart_num = rx_state_preamble##uart_num;		\
      rx_index##uart_num = 0;					\
      com_swap_bufs(IFACE_SERIAL + uart_num, rx_buf##uart_num, &rx_buf##uart_num);	\
   }

#define tx_state_preamble_helper(uart_num, reg)		\
   reg = PREAMBLE;					\
   tx_index##uart_num++;				\
   if(tx_index##uart_num == PREAMBLE_SIZE)		\
      tx_state##uart_num = tx_state_size##uart_num;

#define tx_state_size_helper(uart_num, reg)		\
   reg = tx_buf##uart_num->size;			\
   tx_state##uart_num = tx_state_data##uart_num;	\
   tx_index##uart_num = 0;

#define tx_state_data_helper(uart_num, reg)				\
   if(tx_index##uart_num == tx_buf##uart_num->size) {			\
      UART_DISABLE_TX_INT##uart_num();					\
      tx_index##uart_num = 0;						\
      tx_state##uart_num = tx_state_preamble##uart_num;			\
      mos_sem_post_dispatch(&tx_sem##uart_num);				\
   } else {								\
      reg = tx_buf##uart_num->data[tx_index##uart_num++];		\
   }

#ifndef RAW_MODE_UART0
static void rx_state_preamble0(void)
{
   rx_state_preamble_helper(0, RX_REG0)
}

static void rx_state_size0(void)
{
   rx_state_size_helper(0, RX_REG0)
}

static void rx_state_data0(void)
{
   rx_state_data_helper(0, RX_REG0)
}

static void tx_state_preamble0(void)
{
   tx_state_preamble_helper(0, TX_REG0)
}

static void tx_state_size0(void)
{
   tx_state_size_helper(0, TX_REG0)
}

static void tx_state_data0(void)
{
   tx_state_data_helper(0, TX_REG0)
}
#endif

#ifndef RAW_MODE_UART1
static void rx_state_preamble1(void)
{
   rx_state_preamble_helper(1, RX_REG1)
}

static void rx_state_size1(void)
{
   rx_state_size_helper(1, RX_REG1)
}

static void rx_state_data1(void)
{
   rx_state_data_helper(1, RX_REG1)
}

static void tx_state_preamble1(void)
{
   tx_state_preamble_helper(1, TX_REG1)
}

static void tx_state_size1(void)
{
   tx_state_size_helper(1, TX_REG1)
}

static void tx_state_data1(void)
{
   tx_state_data_helper(1, TX_REG1)
}
#endif

#ifdef NEED_RAW_MODE
#define rx_raw_state_helper(uart_num, reg)			\
   *(rx_raw_buf##uart_num + rx_index##uart_num++) = reg;	\
   if(rx_index##uart_num >= rx_raw_max##uart_num) {		\
      mos_sem_post(&rx_sem##uart_num);				\
   }

#define tx_raw_state_helper(uart_num, reg)			\
   reg = *(tx_raw_buf##uart_num + tx_index##uart_num++);	\
   if(tx_index##uart_num >= tx_raw_max##uart_num) {		\
      mos_sem_post(&tx_sem##uart_num);				\
   }

static void rx_raw0(void)
{
   rx_raw_state_helper(0, RX_REG0);
}

static void tx_raw0(void)
{
   tx_raw_state_helper(0, TX_REG0);
}

static void rx_raw1(void)
{
   rx_raw_state_helper(1, RX_REG1);
}

static void tx_raw1(void)
{
   tx_raw_state_helper(1, TX_REG1);
}

#endif

#ifdef ARCH_AVR

SIGNAL(SIG_UART0_DATA)
{
#ifdef MOS_DEBUG
   mos_debug_set_trace(DBCODE_INTERRUPT);
#endif
   tx_state0();
}

SIGNAL(SIG_UART0_RECV)
{
#ifdef MOS_DEBUG
   mos_debug_set_trace(DBCODE_INTERRUPT);
#endif
   rx_state0();
}

SIGNAL(SIG_UART1_DATA)
{
#ifdef MOS_DEBUG
   mos_debug_set_trace(DBCODE_INTERRUPT);
#endif
   tx_state1();
}
SIGNAL(SIG_UART1_RECV)
{
#ifdef MOS_DEBUG
   mos_debug_set_trace(DBCODE_INTERRUPT);
#endif
   rx_state1();
}

#endif

#ifdef PLATFORM_TELOSB
/* USART0 is used in SPI mode to communicate with the radio and m25p80 flash;
 * these interrupts are implemented in src/mos/dev/spi.c
  interrupt (UART0TX_VECTOR) uart0_tx_int(void)
  {
  
  tx_state0();
  }

  interrupt (UART0RX_VECTOR) uart0_rx_int(void)
  {
  
  rx_state0();
  }
*/

interrupt (UART1TX_VECTOR) uart1_tx_int(void)
{
   tx_state1();
}

interrupt (UART1RX_VECTOR) uart1_rx_int(void)
{
   rx_state1();
}

#endif

#endif //if defined(UART) || !defined(SCONS)
#endif //ifndef platform linux

