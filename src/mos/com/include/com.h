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

/**
  Project Mantis
  File:   com.h
  Authors: Jeff Rose & Brian Shucker
  Date:   01-16-04
  
**/

/** @file com.h
 * @brief Primary com functions
 * @author Jeff Rose, Brian Shucker
 * @date 01/18/2004
 */

#ifndef _COM_H_
#define _COM_H_

#include "mos.h"
#include "mutex.h"
#include <stdarg.h>

#ifdef SCONS
#include "optsconfig.h"
#else

/* Defines which MAC layer to use */
//#define CC1000_RAW
#define CC1000_CSMA
//#define CC1000_CSMA_ACK
//#define CC1000_TDMA
//#define CC1000_BMAC


//ANS: Enable signal strength measurement

#define GET_RSSI
//#define TS_PACKET

#endif


#ifndef PLATFORM_LINUX
// enable FEC code
//#define RADIO_USE_FEC 1
// send three size bytes instead of one, compute packet size from majority bits
//#define RADIO_REDUNDANT_SIZE
// Send 2 CRC bytes after the data that will be checked after FEC is done
//#define RADIO_EXTRA_CRC
#endif

#ifdef RADIO_USE_FEC
//#define FEC_SIZE_PARITY_COUNT 2
#define FEC_DATA_PARITY_COUNT 8
#endif

#ifndef RADIO_EXTRA_CRC
#define COM_DATA_SIZE 84 /* Size of com buffer. */
#else
// Add two more bytes to include a CRC that is checked AFTER error-correction
// FIXME this is an ugly way to do this, since some protocols may use
// this number to determine their payload sizes
#define COM_DATA_SIZE 86
#endif

#ifdef PLATFORM_LINUX
#define NUM_BUFS 255 /* Size of buffer pool. */

#else
#define NUM_BUFS 5
#endif


#ifdef TS_PACKET
#ifdef ARCH_AVR
#include "realtime.h"
#endif
#endif

/** @addtogroup com Communications Interface
 * This group contains functions associated with communications devices,
 * Such as the radio and UART.
 */
//@{

/** @brief The standard comBuf data structure is designed to be used
 * across multiple different communication devices.  A comBuf contains
 * an array of bytes, the number of bytes, and possibly some other
 * fields, depending on the specific platform.
*/
typedef struct comBuf_t {
   /** @internal A pointer to the next comBuf in the pool.
    * This should not be used by an application. */
   struct comBuf_t *next;
   /** @brief An array of bytes representing the contents
    * of the comBuf. The maximum number of bytes that may be
    * placed in a comBuf is represented by COM_DATA_SIZE. */
   uint8_t data[COM_DATA_SIZE];

#ifdef GET_RSSI
/** @brief For a comBuf received from the radio driver,
 * The RSSI value (signal strength indicator).  This member is
 * only available of GET_RSSI has been defined in com.h */
   uint16_t signal;
#endif
#ifdef TS_PACKET
   /** @internal if TS_PACKET is defined, this field contains a 32-bit
    * timestamp. */
   uint32_t ts;
   /** @internal if TS_PACKET is defined, this field contains an 8-bit
    * count. */
   uint8_t tcnt;
#endif
   /** @brief The number of data bytes located in the data array
    * of this comBuf. */
   uint8_t size;
   uint16_t source;
} comBuf;


/** @name Available Interfaces
 * Each of the following can be used with the
 * com_send(), com_recv(), com_mode(), and com_ioctl()
 * functions. */
//@{
/** @brief the UART0 serial interface. */
#define IFACE_SERIAL   0 
/** @brief the UART1 serial interface. */
#define IFACE_SERIAL2  1
/** @brief the CC2420 or CC1000 radio interface. */
#define IFACE_RADIO    2 
/** @brief the loopback interface. */
#define IFACE_LOOPBACK 3
/** @internal The maximum number of interfaces. */
#define MAX_IFS        7
//@}

#ifdef PLATFORM_LINUX
#define IFACE_UDP      4 // udp/ip sockets
#define IFACE_TCP      5 // tcp/ip sockets
#define IFACE_TERMINAL 6 // terminal (input reading)
#endif

#ifdef PLATFORM_TELOSB
#define com_send_IFACE_STDIO com_send_IFACE_SERIAL2
#define com_ioctl_IFACE_STDIO com_ioctl_IFACE_SERIAL2
#define com_mode_IFACE_STDIO com_mode_IFACE_SERIAL2
#else
/** @brief for cross platform compatibilty, IFACE_STDIO resolves to either
 * IFACE_SERIAL or IFACE_SERIAL2 -- whichever interface will output to the terminal --
 * depending on the platform. This macro aliases the com_send function.
 * @ingroup com_serial */
#define com_send_IFACE_STDIO com_send_IFACE_SERIAL
/** @brief for cross platform compatibilty, IFACE_STDIO resolves to either
 * IFACE_SERIAL or IFACE_SERIAL2 -- whichever interface will output to the terminal --
 * depending on the platform. This macro aliases the com_ioctl function.
 * @ingroup com_serial */
 
#define com_ioctl_IFACE_STDIO com_ioctl_IFACE_SERIAL
/** @brief for cross platform compatibilty, IFACE_STDIO resolves to either
 * IFACE_SERIAL or IFACE_SERIAL2 -- whichever interface will output to the terminal --
 * depending on the platform. This macro aliases the com_mode function.
  * @ingroup com_serial */
#define com_mode_IFACE_STDIO com_mode_IFACE_SERIAL
#endif

/********* Device-specific function prototypes ************/
/** @addtogroup com_serial Serial Interfaces
 * The IFACE_SERIAL and IFACE_SERIAL2 interfaces are used to
 * relay data to the terminal and/or any devices which
 * communicate via the UART.
 */
//@{
/** @brief transmit the data in buf over UART0.
 * @param buf a pointer to the comBuf to transmit.
 * @return Always returns 0.
 */
uint8_t com_send_IFACE_SERIAL(comBuf *buf);
/** @brief change the mode of the UART0 interface.
 * Accepted parameters are:
 *    -# IF_OFF
 *    -# IF_LISTEN
 * @param mode the new mode.
 */
void com_mode_IFACE_SERIAL(uint8_t mode);
/** @brief currently unsupported for the serial interface.
 * @param request ignored.
 * @param ... ignored.
 */
void com_ioctl_IFACE_SERIAL(uint8_t request, ...);

/** @brief transmit the data in buf over UART1.
 * @param buf a pointer to the comBuf to transmit.
 * @return Always returns 0.
 */
uint8_t com_send_IFACE_SERIAL2(comBuf *buf);
/** @brief change the mode of the UART1 interface.
 * Accepted parameters are:
 *    -# IF_OFF
 *    -# IF_LISTEN
 * @param mode the new mode.
 */
void com_mode_IFACE_SERIAL2(uint8_t mode);
/** @brief currently unsupported for the serial interface.
 * @param request ignored.
 * @param ... ignored.
 */
void com_ioctl_IFACE_SERIAL2(uint8_t request, ...);
//@}

/** @addtogroup com_radio Radio Interface
 * The IFACE_RADIO interface allows programs to
 * interact directly with the onboard radio, which
 * is either the CC1000 (Mica2) or CC2420 (MicaZ/TELOSb)
 */
//@{
/** @brief transmit the contents of buf over the radio.
 * @param buf a pointer to the comBuf to transmit.
 * @return returns 0 on success.
 */
uint8_t com_send_IFACE_RADIO(comBuf *buf);
/** @brief change the power status of the radio.
 * Valid parameters are:
 *    -# IF_OFF
 *    -# IF_LISTEN
 * @param mode the new mode.
 */
void com_mode_IFACE_RADIO(uint8_t mode);
/** @brief send an device-specific IO Control request to the radio.
 * Valid parameters are:
 *   -# CC2420_LOW_POWER_MODE
 *   -# CC2420_HIGH_POWER_MODE
 *   -# CC2420_TX_POWER, [int power]
 *   -# CC1000_TX_POWER, [int power]
 *   -# CC1000_GET_TX_POWER, [uint8_t* power]
 *   -# CC1000_FREQ, [int frequency]
 * @param request the IO Control to send.
 * @param ... any required data for the IOCTL.
 */
void com_ioctl_IFACE_RADIO(uint8_t request, ...);
//@}

/** @addtogroup com_lin_fun Linux COM Functions
 * These functions can be used when building for linux.
 * They do not exist when targeting motes.
 */
//@{
uint8_t com_send_IFACE_TERMINAL(comBuf *buf);
void com_mode_IFACE_TERMINAL(uint8_t mode);
void com_ioctl_IFACE_TERMINAL(uint8_t request, ...);

uint8_t com_send_IFACE_UDP(comBuf *buf);
void com_mode_IFACE_UDP(uint8_t mode);
void com_ioctl_IFACE_UDP(uint8_t request, ...);
//@}

/** @addtogroup com_loop Loopback Interface
 * The loopback interface allows programs to
 * transmit and receive data locally, for example
 * in a producer-consumer thread situation. */
//@{

uint8_t com_send_IFACE_LOOPBACK(comBuf *buf);
void com_mode_IFACE_LOOPBACK(uint8_t mode);
void com_ioctl_IFACE_LOOPBACK(uint8_t request, ...);
//@}


/** @name COM Modes
 * These values can be passed to the com_mode()
 * function for each com interface.  Note that
 * not all interfaces support every mode.
 */
//@{

/** @brief The possible mode settings for interface devices. */
enum {
   /** @brief Interface is off. */
   IF_OFF = 0, 
   /** @brief Interface is in low power mode with wakeup. */
   IF_STANDBY,
   /** @brief Interface is on, but not listening. */
   IF_IDLE,
   /** @brief Interface is on and listening for data. */
   IF_LISTEN,
#ifdef RADIO_USE_FEC
   /** @brief Interface has FEC enabled. */
   IF_FEC_ENABLE,
   /** @brief Interface has FEC disabled. */
   IF_FEC_DISABLE 
#endif
};
//@}

#define PREAMBLE            0x53
#define PREAMBLE_SIZE       2


/** @name COM Error Messages
 * These values can be returned from com layer functions
 * when an error occurs.
 */
//@{
/** @brief The requested interface is busy. */
#define SELECT_IFACE_BUSY 150
/** @brief The requested interface does not exist. */
#define IFACE_NOT_REGISTERED 160
//@}

/** @name COM Select
 * com_select can be used to check if there is data available to be read
 * from a specified interface.  This function is similar to the berkley socket
 * select() function.
 */
//@{
/** @brief represents a set of interfaces */
typedef uint8_t IF_SET;

/* NOTE: These implementations limit the number of devices to 8. */

/** @brief Add the specified interface to the IF_SET pointed to by set. */
#define IF_SET(iface, set) (*set |= 1 << iface) // TODO: check shift direction
/** @brief Remove the specified interface from the IF_SET pointed to by set */
#define IF_CLEAR(iface, set) (*set &= ~(1 << iface))
/** @brief Remove all interfaces from the IF_SET pointed to by set. */
#define IF_ZERO(set) (*set = 0)
/** @brief Returns non-zero if the specified interface is set in the IF_SET pointed to by set. */
#define IF_ISSET(iface, set) (*set & (1 << iface))

/** @brief Check a set of interfaces for valid data.  
 * @param iset Interface set to check.  After the function returns,
 * iset contains only interfaces with data available. Use IF_ISSET() to determine
 * which interfaces are set.
 * @param msec the number of milliseconds to wait for data before returning.
 * @return The number of interfaces that are set in iset, or a COM Error Code.
 */


uint8_t com_select (IF_SET *iset, uint32_t msec);

/** @brief pass to com_select() to block until an interface becomes readable */
#define COM_BLOCK -1
/** @brief pass to com_select() to return immediately */
#define COM_NOBLOCK 0

//@}



/** @internal Init com layer.  Must be called before any ifaces
 * try to register.
 * @return Always returns 0
 */
uint8_t com_init();
/** @internal Used by the com layer to synchronize access to devices when transmiting data. */
extern mos_mutex_t if_send_mutexes[MAX_IFS];

/** @addtogroup com_funcs Generic COM Functions
 * These functions can be used with all interfaces.
 */
//@{

/** @brief Flush the packets on a particular interface.
 * @param iface the interface to flush.
 */
void com_flush (uint8_t iface);


/** @brief Send data over specified interface.
 *  Blocks until the data has been transmitted.
 * @param iface Interface to use
 * @param buf A pointer to the buffer containing data to send.
 * @return Retval, else IFACE_NOT_REGISTERED
 */
#define com_send(iface,buf) com_send_##iface (buf)
#define com_sendto(iface,buf,dest) com_sendto_##iface(buf,dest)

/** @brief Receive data from a specified interface.
 * Blocks until a packet is available.
 * It is important that once the received comBuf has been
 * handled that the application call com_free_buf(buf)
 * to release the buffer back to the system.
 * @param iface Interface to use
 * @return A buffer containing the received data.
 */
comBuf *com_recv(uint8_t iface);

/** @brief Receive data from a specified interface.
 * If no data is received in the specified amount of time, the call will return.
 * @param iface Interface to use
 * @param msecs Milliseconds to wait before timing out
 * @return A buffer containing the received data, or NULL if no
 * data was received.
 */
comBuf *com_recv_timed(uint8_t iface, uint32_t msecs);

/** @brief Pass ioctl flag and data down to the given interface.
 * @param iface Interface used
 * @param Request The IOCTL to pass down to the interface.
 * @return Retval, else IFACE_NOT_REGISTERED
 */
#define com_ioctl(iface,request,args...) com_ioctl_##iface (request, ##args)

/** @brief Pass the mode setting down to the given interface. 
 * @param iface Interface used
 * @param mode The new mode.
 * @return Retval, else IFACE_NOT_REGISTERED
 */
#define com_mode(iface,mode) com_mode_##iface (mode)

/** @brief Release a buffer returned by com_recv or com_recv_timed
 * back to the system.  If buffers are not released with this method,
 * The system will run out of buffers and not be able to receive data.
 * @param buf A pointer to the buffer to free.
 */
void com_free_buf (comBuf *buf);
//@}

/** @internal Swaps full receive buffer with a fresh buffer.  
 * @brief This is called by a driver when it has a full recv buffer.
 * Also called by a driver to just get another buffer, in
 * which case buf is null.
 *
 *  Drivers must register before they call swap_bufs!
 * @param iface Interface to use
 * @param buf Buffer to use
 * @return New buffer
 */
void com_swap_bufs (uint8_t iface, comBuf *buf, comBuf **ret);

/** @addtogroup com_help COM Helper Functions
 * These functions do not directly relate to the COM Layer,
 * but are useful when dealing with comBufs on specific platforms.
 *
 * Specifically, the buf_insert and buf_extract functions can be used instead
 * of casting a byte pointer to a uint16_t* or uint32_t*.  On some platforms such
 * as the TELOSb, undefined behavior occurs if you read or write a 16-bit or 32-bit
 * value to a location in memory that is not word aligned.  These functions do
 * not have such an issue.
 */
//@{
/** @brief Insert a uint16_t into an array of bytes in little endian byte order.
 *  @param p A pointer to an array of uint8_ts..
 *  @param pos The index at which to insert the uint16_t.
 *  @param word The 16-bit number to insert.
 */
void buf_insert16(uint8_t* p, uint16_t pos, uint16_t word);

/** @brief Insert a uint32_t into an array of bytes in little endian byte order.
 *  @param p A pointer to an array of uint8_ts.
 *  @param pos The index at which to insert the uint32_t.
 *  @param dword The 32-bit number to insert.
 */
void buf_insert32(uint8_t* p, uint16_t pos, uint32_t dword);

/** @brief Extracts a little endian uint16_t from an array of bytes.
 *  @param p A pointer to a byte array
 *  @param pos The index from which to extract the uint16_t.
 *  @return the extracted 16-bit value.
 */
uint16_t buf_extract16(uint8_t* p, uint16_t pos);

/** @brief Extracts a little endian uint32_t from an array of bytes.
 *  @param p A pointer to a byte array
 *  @param pos The index from which to extract the uint32_t.
 *  @return the extracted 32-bit value.
 */
uint32_t buf_extract32(uint8_t* p, uint16_t pos);
//@}




/* ELF object propagation support */
/** @internal used for ELF propogation */
char is_elf_packet(comBuf *buf);
/** @internal used for ELF propogation */
#define ELFPKT_BYTE0 65
/** @internal used for ELF propogation */
#define ELFPKT_BYTE1 75
/** @internal used for ELF propogation */
#define ELFPKT_BYTE2 85
/** @internal used for ELF propogation */
#define ELFPKT_BYTE3 95

#ifdef USING_ELF

#include "msched.h"
#include "com.h"
/** @internal used for ELF propogation */
mos_thread_t *elfthread;
/** @internal used for ELF propogation */
comBuf *elfbuf;
/** @internal used for ELF propogation */
char is_elf_packet(comBuf *buf);

//@}
#endif


#endif
