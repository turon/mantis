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
  File:   loopback.c
  Authors: Brian Shucker
  Date:   01-16-04
  
  Implementation of the local loopback network device
  (Useful for network generality and for IPC)
*/

#include "mos.h"
#include "com.h"
#include "sem.h"
#include "mutex.h"
#include "loopback.h"

#if defined(LOOPBACK) || !defined(SCONS)

static comBuf *managed_buf;
static mos_sem_t recv_sem;
static mos_sem_t send_sem;

void com_loopback_init(void)
{
  //get initial buffer
   com_swap_bufs(IFACE_LOOPBACK, NULL, &managed_buf);

   mos_sem_init(&recv_sem, 0);
   mos_sem_init(&send_sem, 0);
   
   return;
}

uint8_t com_send_IFACE_LOOPBACK(comBuf *buf)
{
  uint8_t i;

  mos_mutex_lock(&if_send_mutexes[IFACE_LOOPBACK]);

  if(managed_buf != NULL) {
     for(i = 0; i < buf->size; i++)
	managed_buf->data[i] = buf->data[i];
     
     managed_buf->size = buf->size;
  }

  com_swap_bufs(IFACE_LOOPBACK, managed_buf, &managed_buf);
  
  mos_sem_post(&send_sem);
  mos_sem_wait(&send_sem);

  mos_mutex_unlock(&if_send_mutexes[IFACE_LOOPBACK]);

  return 0;
}

void com_ioctl_IFACE_LOOPBACK(uint8_t request, ...)
{
   return;
}

void com_mode_IFACE_LOOPBACK(uint8_t mode)
{
   return;
}

#endif
