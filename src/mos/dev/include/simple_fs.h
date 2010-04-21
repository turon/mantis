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

/** @file simple_fs.h
 * @brief Super-simple file system
 */

#ifndef _SIMPLE_FS_H_
#define _SIMPLE_FS_H_

/* The following file names are already in use:
 * 
 * prg - program image propagated by Aqueduct or Deluge
 * aqX - cache file used by Aqueduct or Deluge, X is a letter starting from '1'
 * rrp - temporary program image stored by repro_reprogram function before it
 *       gets renamed to 'prg' or 'bkX'
 * bkX - backup image, X is a letter starting from '1'
 */
 
#include "mos.h"

typedef struct {
	// This field was originally supposed to mean something more,
	// but now is just being used as a flag that the file is allocated. 
	int8_t index;
	uint8_t name[3];
	uint32_t start;
	uint32_t length;
} mos_file;

void simple_fs_init();
void simple_fs_format();
mos_file* mos_file_create(const char* name, uint32_t length);
mos_file* mos_file_open(const char* name);
void mos_file_rename(const char* src, const char* dst);
uint16_t mos_file_read(uint8_t* buf, mos_file* file, uint32_t offset, uint16_t length);
uint16_t mos_file_write(uint8_t* buf, mos_file* file, uint32_t offset, uint16_t length);
uint16_t mos_file_crc(mos_file* file, uint32_t offset, uint32_t length);
void mos_file_flush(mos_file* file);
int8_t mos_file_close(mos_file* file);
void simple_fs_ls();

#endif /* _SIMPLE_FS_H_ */
