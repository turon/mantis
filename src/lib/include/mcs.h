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

/** @file mcs.h
 * @brief Defines used for comm. between the comp and node
 *
 * Note: Many of these are obsolete as they are not implemented.
 */

/** @brief 128k bytes of flash. */
#define FLASH_SIZE 131072      
/** @brief 128 words per page of flash. */
#define PAGE_SIZE_BYTES 256           // 128 words per page of flash

#define NOOP 0
#define LOAD_IMAGE 1
#define READ_FLASH 2
#define READ_FUSES 3
#define START 4
#define PEEK 5
#define POKE 6
#define HEAP_DUMP 7
#define STACK_DUMP 8
#define THREAD_DUMP 9
#define CALL 10
#define SPAWN 11
#define QUIT 12
#define RESET 13
#define WRITE_EEPROM 15
#define READ_EEPROM 16

/*return codes*/
#define UNKNOWN 100
#define ADDRESS_RECVD 101
#define PAGE_RECVD 102
#define PAGE_WRITTEN 103
#define FLASH_READ_COMPLETE 104
#define ADDR_TOO_HIGH 105
#define FLASH_FULL 106
#define NOT_IN_XMOS 107
#define BAD_EEPROM 108
#define EEPROM_DONE 109
#define ACK 110
#define LOAD_ACK 111
#define PACKET_ACK 112
#define PAGE_ACK 113
#define IMAGE_ACK 114

/*other codes*/
#define LOADER_PRESENT 200 
#define SHELL_PRESENT 201  
#define APP_PRESENT 202  
#define REBOOT 203
#define CLEAR_CB 204
#define LOADER_PRESENT_PONG 205 
#define CLEAR_CB_PONG 206
#define WORD_PRINT 245
