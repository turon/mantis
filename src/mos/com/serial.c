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
  File:   serial.c
  Authors: Jeff Rose
  Date:   02-01-04
  
  Mac layer built on top of serial uart devices.
*/

#include "mos.h"

#ifdef PLATFORM_LINUX

#include <stdlib.h>
#include <strings.h>
#include <sys/time.h>

#include "serial.h"
#include "com.h"
#include "msched.h"
#include "mutex.h"
#include "arg.h"
#include <pthread.h>
#include <signal.h>
#include <sys/ioctl.h>
#if defined(SERIAL) || !defined(SCONS)

/** @brief Terminal settings. */
struct termios oldtio, newtio; 

static comBuf *packetBuf;
static pthread_t serial_tid = 0;

int serialfd = -1;
uint8_t current_mode;

void serial_listen ();

void serial_init_fd ()
{
   uint8_t *serial_dev;
   uint8_t *new_baud;
   int baud;
   // Check to see if someone specified a different serial port
   if (!mos_arg_check ("--sdev", &serial_dev))
      serial_dev = DEFAULT_SERIAL_DEV;
      
   if (!mos_arg_check ("--sbaud", &new_baud))
   {
      baud = DEFAULT_BAUDRATE;
   } else {
      if(strcmp(new_baud, "B2400") == 0) {
	 baud = B2400;
      } else if(strcmp(new_baud, "B4800") == 0) {
	 baud = B4800;
      } else if(strcmp(new_baud, "B9600") == 0) {
	 baud = B9600;
      } else if(strcmp(new_baud, "B19200") == 0) {
	 baud = B19200;
      } else if(strcmp(new_baud, "B38400") == 0) {
	 baud = B38400;
      } else if(strcmp(new_baud, "B57600") == 0) {
	 baud = B57600;
      } else if(strcmp(new_baud, "B115200") == 0) {
	 baud = B115200;
      } else if(strcmp(new_baud, "B230400") == 0) {
	 baud = B230400;
      } else {
	 printf("Error in baud setting, try '--sbaud B57600'\n");
	 baud = DEFAULT_BAUDRATE;
      }
      printf("specified baud: %s\n", new_baud);
      
   }

   printf("opening serial\n");
   // Init the actual serial device.  The NOCTTY means this
   // terminal won't have any process control.  (That way if a
   // control-C happens to come down the line we don't get dropped.)
   if (serialfd == -1) {
//      serialfd = open (serial_dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
      serialfd = open (serial_dev, O_RDWR | O_NOCTTY);
      if (serialfd < 0) {
	 printf("ERROR OPENING SERIAL\n");
	 perror (serial_dev);
	 exit (-1);
      }
   } else {
      printf ("Serial port is already open\n");
      return;
   }

/*
   
   // Note that open() follows POSIX semantics: multiple open() calls to the same file will succeed
   // unless the TIOCEXCL ioctl is issued. This will prevent additional opens except by root-owned
   // processes.
   // See tty(4) ("man 4 tty") and ioctl(2) ("man 2 ioctl") for details.

   if (ioctl(serialfd, TIOCEXCL) == -1)
     {
	printf("Error setting TIOCEXCL on %s.\n", serial_dev);
       return;
     }

   // Now that the device is open, clear the O_NONBLOCK flag so subsequent I/O will block.
   // See fcntl(2) ("man 2 fcntl") for details.

   if (fcntl(serialfd, F_SETFL, 0) == -1)
     {
	printf("Error clearing O_NONBLOCK %s.\n", serial_dev);
       return;
     }        

*/     
     
   tcgetattr (serialfd, &oldtio); // save current port settings
   
   // Setup new serial port settings
   bzero (&newtio, sizeof (newtio));
   // manually do what cfmakeraw () does since cygwin doesn't support it
   newtio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
		       INLCR | IGNCR | ICRNL | IXON);
   newtio.c_oflag &= ~OPOST;
   newtio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
   newtio.c_cflag &= ~(CSIZE | PARENB);
   newtio.c_cflag |= CS8;
//   newtio.c_cflag |= O_NOCTTY;

   newtio.c_iflag &= ~(INPCK | IXOFF | IXON);
   newtio.c_cflag &= ~(HUPCL | CSTOPB | CRTSCTS);
   newtio.c_cflag |= (CLOCAL | CREAD);
   
   // set input mode (non-canonical, no echo,...)
   newtio.c_lflag = 0;  
   newtio.c_cc[VTIME] = 0;   // min time between chars (*.1sec)
   newtio.c_cc[VMIN] = 1;   // min number of chars for read 

   cfsetispeed (&newtio, baud);
   cfsetospeed (&newtio, baud);
   tcflush (serialfd, TCIFLUSH);
   tcsetattr (serialfd,TCSANOW,&newtio);

   printf("done.\n");
}

uint8_t serial_init ()
{
   serial_init_fd();
   //initialize the serial mutex
   //mos_mutex_init(&serial_mutex);
   
   current_mode = IF_OFF;
   
   // get initial buffer
   com_swap_bufs (IFACE_SERIAL, NULL, &packetBuf);
   packetBuf->size = 0;

   // Create a thread to constantly listen on the serial port and
   // save packets if we are in listen mode
   if(serial_tid == 0)
      mos_thread_new (serial_listen, 128, PRIORITY_NORMAL);
   
   return 0;
}

void serial_close ()
{
   if (serialfd != -1) { //if the file descriptor has been set...
      close (serialfd); //close the file descriptor
      serialfd = -1;    //set the value to initial known value
   }
}

void serial_block()
{
   //mos_mutex_lock(&serial_mutex);
}

void serial_unblock()
{
   //mos_mutex_unlock(&serial_mutex);
}

void serial_flush ()
{
   tcflush (serialfd, TCIFLUSH);
}

uint8_t com_send_IFACE_SERIAL (comBuf *buf)
{
   uint8_t byte = PREAMBLE;
   uint8_t i;
   int retval;

   mos_mutex_lock (&if_send_mutexes[IFACE_SERIAL]);
   
   // Send the preamble then the size and packet
   for(i = 0; i < PREAMBLE_SIZE; i++) {
      retval = write (serialfd, &byte, 1);
      if (retval == -1)
	 goto serial_send_error;
   }
   retval = write (serialfd, &buf->size, 1);
   if (retval == -1)
      goto serial_send_error;
   retval = write (serialfd, &buf->data, buf->size);
   if (retval == -1)
      goto serial_send_error;
   retval = tcdrain (serialfd);

serial_send_error:
   if (retval == -1)
      perror("com_send_IFACE_SERIAL");
			
   mos_mutex_unlock (&if_send_mutexes[IFACE_SERIAL]);
   
   return 0;
}

uint8_t serial_get_mode ()
{
   return current_mode;
}

void com_mode_IFACE_SERIAL (uint8_t mode)
{
   if (mode != current_mode) {
      switch (mode) {
      case IF_OFF:
	 break;
      case IF_STANDBY:
      case IF_IDLE:
      case IF_LISTEN:
	 break;
      default:
	 break;
      }
      current_mode = mode;
   }
}

void com_ioctl_IFACE_SERIAL (uint8_t request, ...)
{
   int baud_rate;
   va_list ap;
   va_start(ap, request);

   switch (request) {
   case BAUD_RATE:
      baud_rate = va_arg (ap, int);
      printf ("Setting new baud rate: %d\n", baud_rate);
      newtio.c_cflag = baud_rate | CS8 | CLOCAL | CREAD;
      tcflush (serialfd, TCIFLUSH);
      tcsetattr (serialfd, TCSANOW, &newtio);
      break;
   }
   va_end(ap);
}

/** @brief See full description.
 * 
 * This function is meant to be run as a separate thread that is
 * constantly reading bytes off the serial port.  In order to make
 * the behavior the same in xmos and amos I am dropping packets
 * unless the user has explicitly turned on the serial device. 
 */
void serial_listen ()
{
   uint8_t byte, preamble_count, bytesRead;
   uint8_t *dataPtr;
   int retval;
   
   preamble_count = 0;
   
   serial_tid = pthread_self ();
   
   while (1) {
      retval = read (serialfd, &byte, 1);
      if (retval > 0) {
	 // Check for the preamble
	 if (byte == PREAMBLE) {
	    preamble_count++;
	    
	    // After the preamble we can get the size and read the packet
	    if (preamble_count == PREAMBLE_SIZE) {
	       retval = 0;
	       while (retval == 0) {
		  retval = read (serialfd, &packetBuf->size, 1);
	       }
	       if (retval == -1) {
		  perror("serial_listen: read");
		  continue;
	       }

	       dataPtr = packetBuf->data;
	       bytesRead = 0;
	       while (bytesRead < packetBuf->size) {
		  retval = read (serialfd, dataPtr, packetBuf->size - bytesRead);
		  if (retval == -1) {
		     perror("serial_listen: read");
		     break;
		  }
		  bytesRead += retval;
		  dataPtr = packetBuf->data + bytesRead;
	       }
	       // Only send the packet up to the com layer if we are actually
	       //  in listen mode
	       if (current_mode == IF_LISTEN) {
		  com_swap_bufs (IFACE_SERIAL, packetBuf, &packetBuf);
		  while (packetBuf == NULL) {
		     fprintf (stderr, "Ran out of packets...\n");
		     usleep (250);
		     com_swap_bufs (IFACE_SERIAL, packetBuf, &packetBuf);
		  }
	       }

	       preamble_count = 0;
	    }
	 } else // Only a valid preamble if all the bytes show up in a row
	    preamble_count = 0;
      }
      else if (retval == -1) {
	 perror("serial_listen: read");
      }
   }
}

#endif
#endif
