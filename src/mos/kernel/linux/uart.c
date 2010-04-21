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

/**************************************************************************/
/* File:      mos.h                                                       */
/* Author     Jeff Rose & Brian Shucker                                   */
/*   Date: 05/12/03                                                       */
/*                                                                        */
/**************************************************************************/

#include "mos.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "uart.h"


#define BAUDRATE B57600    // 19200 now matches nymph/mica2 default uart baud
#define NUM_UARTS 2       // number of uarts

char *device [NUM_UARTS] = {"/dev/ttyS0","/dev/ttyS1"}; // serial device locations

int uartFd [NUM_UARTS];
struct termios oldtio,newtio; //terminal settings

/** Startup the uart with default settings
 */
int8_t mos_uart_open(uint8_t uart)
{  
  if(uart > NUM_UARTS) return -1;

    uartFd[uart] = open(device[uart], O_RDWR | O_NOCTTY); 

  if (uartFd <0) {perror(device[uart]); exit(-1); }
  
  tcgetattr(uartFd[uart],&oldtio); /* save current port settings */
  
  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR | IXANY;
  newtio.c_oflag = 0;
  
  /* set input mode (non-canonical, no echo,...) */
  newtio.c_lflag = 0;
  
  newtio.c_cc[VTIME]    = 0;   /* min time between chars (*.1sec)*/
  newtio.c_cc[VMIN]     = 0;   /* min number of chars for read */
  
  tcflush(uartFd[uart], TCIFLUSH);
  tcsetattr(uartFd[uart],TCSANOW,&newtio);

  return 0;
}

int mos_uart_getfd(uint8_t uart)
{
  return uartFd[uart];
}

/** Send a byte to the uart
 * @param uart uart to send the byte on
 * @param byte Byte to send
 */
void mos_uart_send(uint8_t uart, uint8_t byte)
{
  write(uartFd[uart], &byte, 1);
  tcdrain(uartFd[uart]);
}

/** Get a byte from the uart
 * @return Byte received
 */
uint8_t mos_uart_recv(uint8_t uart)
{
  uint8_t byte;
  while(!read(uartFd[uart], &byte, 1))
    ;
  return byte;
}

/** Turn off the uart
 */
void mos_uart_close(uint8_t uart)
{
  tcsetattr(uartFd[uart],TCSANOW,&oldtio);
}
