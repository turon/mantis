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

/** @file simple_fs.c
 * @brief Super-simple file system
 */

#include "mos.h"

#ifndef PLATFORM_LINUX

#include "simple_fs.h"
#include "dev.h"
#include "atmel-flash.h"
#include "mutex.h"
#include "printf.h"
#include "command_daemon.h"
#include "avr-eeprom.h"

static struct {
	uint16_t nextPage;
	mos_file files[SIMPLE_FS_MAX_FILES];
} fss;

static mos_mutex_t fs_lock;

static void saveState()
{
	dev_ioctl (DEV_AVR_EEPROM, DEV_LOCK);
	dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, SIMPLE_FS_ADDR);
	dev_write (DEV_AVR_EEPROM, (uint8_t *)&fss, sizeof (fss));
	dev_ioctl (DEV_AVR_EEPROM, DEV_UNLOCK);
}

static void loadState()
{
	dev_ioctl (DEV_AVR_EEPROM, DEV_LOCK);
	dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, SIMPLE_FS_ADDR);
	dev_read (DEV_AVR_EEPROM, (uint8_t *)&fss, sizeof (fss));
	dev_ioctl (DEV_AVR_EEPROM, DEV_UNLOCK);
}

static int8_t findFile(const char* name)
{
	int8_t i;
	for (i=0; i<SIMPLE_FS_MAX_FILES; i++) {
		if (fss.files[i].index != -1 &&
			name[0]==fss.files[i].name[0] &&
			name[1]==fss.files[i].name[1] &&
			name[2]==fss.files[i].name[2])
		{
			return i;
		}
	}
	return -1;
}

// TODO this hasn't actually been tested in a situation where files could conflict
static uint32_t findFreePages(uint32_t start, uint32_t length)
{
	uint32_t ret = start;
	uint8_t i;
	while (ret + length <= ATMEL_FLASH_SIZE)
	{	
		for (i=0; i<SIMPLE_FS_MAX_FILES; i++) {
			if (fss.files[i].index == -1)
				continue;
			if (fss.files[i].start < ret+length &&
				fss.files[i].start+fss.files[i].length > ret)
					break;
		}
		if (i==SIMPLE_FS_MAX_FILES)
			return ret;
		// skip to the end of the conflicting file
		ret = ((fss.files[i].start + fss.files[i].length + 
			(uint32_t)ATMEL_FLASH_PAGE_SIZE-1)/(uint32_t)ATMEL_FLASH_PAGE_SIZE)*
			(uint32_t)ATMEL_FLASH_PAGE_SIZE;
	}
	// try again from the beginning
	ret = (uint32_t)0;
	while (ret < start)
	{	
		for (i=0; i<SIMPLE_FS_MAX_FILES; i++) {
			if (fss.files[i].index == -1)
				continue;
			if (fss.files[i].start <= ret+length &&
				fss.files[i].start+fss.files[i].length >= ret)
					break;
		}
		if (i==SIMPLE_FS_MAX_FILES)
			return ret;
		// skip to the end of the conflicting file
		ret = ((fss.files[i].start + fss.files[i].length + 
			(uint32_t)ATMEL_FLASH_PAGE_SIZE-1)/(uint32_t)ATMEL_FLASH_PAGE_SIZE)*
			(uint32_t)ATMEL_FLASH_PAGE_SIZE;
	}
	// Return an address that is way out of range
	return (uint32_t)-1;
}

mos_file* mos_file_create(const char* name, uint32_t length)
{
	// Name must be exactly 3 printable characters
	if (name[0]<32||name[0]>=127||
		name[1]<32||name[1]>=127||
		name[2]<32||name[2]>=127||
		name[3] != 0)
			return NULL;
			
	mos_mutex_lock(&fs_lock);
	int8_t i = findFile(name);
	
	// Allocate a file if needed
	if (i == -1) {
		for (i=0; i<SIMPLE_FS_MAX_FILES; i++)
			if (fss.files[i].index == -1)
				break;
		if (i==SIMPLE_FS_MAX_FILES) {
			mos_mutex_unlock(&fs_lock);
			return NULL;
		}
		fss.files[i].name[0] = name[0];
		fss.files[i].name[1] = name[1];
		fss.files[i].name[2] = name[2];
	}
	
	// Find free space
	uint32_t start = fss.nextPage * (uint32_t)ATMEL_FLASH_PAGE_SIZE;
	start = findFreePages(start, length);
	if (start == (uint32_t)-1)
		return NULL;
	
	// Mark the file as allocated
	mos_file* file = &fss.files[i];
	file->index = i;
	
	// Find where the file should start
	/*file->start = fss.nextPage * (uint32_t)ATMEL_FLASH_PAGE_SIZE;
	if (file->start + length > (uint32_t)ATMEL_FLASH_SIZE) {
		// There is not enough room on flash to store this image,
		// start again at 0
		// TODO we really ought to check that we're not overwriting
		// existing files
		file->start = 0;
	}*/
	file->start = start;
	
	// Find the next page after this file
	fss.nextPage = (file->start + length
		+ (uint32_t)ATMEL_FLASH_PAGE_SIZE-1)/(uint32_t)ATMEL_FLASH_PAGE_SIZE;
	file->length = length;
	saveState();
	
	mos_mutex_unlock(&fs_lock);
	return file;
}

mos_file* mos_file_open(const char* name)
{
	mos_mutex_lock(&fs_lock);
	int8_t i = findFile(name);
	mos_file* file = NULL;
	if (i != -1) {
		file = &fss.files[i];
		/*file->index = i;
		saveState();*/
	}
	mos_mutex_unlock(&fs_lock);
	return file;
}

void mos_file_rename(const char* src, const char* dst)
{
	// New name must be exactly 3 printable characters
	if (dst[0]<32||dst[0]>=127||
		dst[1]<32||dst[1]>=127||
		dst[2]<32||dst[2]>=127||
		dst[3] != 0)
			return;

	mos_mutex_lock(&fs_lock);
	// Find the file being renamed
	int8_t src_idx = findFile(src);
	if (src_idx != -1)
	{
		// Find if a file with the destination name already exists
		int8_t dst_idx = findFile(dst);
		if (dst_idx != -1)
			// Deallocate it
			fss.files[dst_idx].index = -1;
			
		// Copy the new name
		fss.files[src_idx].name[0] = dst[0];
		fss.files[src_idx].name[1] = dst[1];
		fss.files[src_idx].name[2] = dst[2];
		
		saveState();
	}
	mos_mutex_unlock(&fs_lock);
}

uint16_t mos_file_read(uint8_t* buf, mos_file* file, uint32_t offset, uint16_t length)
{
	mos_mutex_lock(&fs_lock);
	uint16_t count = length;
	if (offset >= file->length) {
		count = 0;
	} else if (offset + count > file->length) {
		count = file->length - offset;
	}
	if (count>0) {
		dev_ioctl(DEV_ATMEL_FLASH, DEV_LOCK);
		dev_ioctl(DEV_ATMEL_FLASH, DEV_SEEK, file->start + offset);
		dev_read(DEV_ATMEL_FLASH, buf, count);
		dev_ioctl(DEV_ATMEL_FLASH, DEV_UNLOCK);
	}
	mos_mutex_unlock(&fs_lock);
	return count;
}

uint16_t mos_file_write(uint8_t* buf, mos_file* file, uint32_t offset, uint16_t length)
{
	mos_mutex_lock(&fs_lock);
	uint16_t count = length;
	if (offset >= file->length) {
		count = 0;
	} else if (offset + count > file->length) {
		count = file->length - offset;
	}
	if (count>0) {
		dev_ioctl(DEV_ATMEL_FLASH, DEV_LOCK);
		dev_ioctl(DEV_ATMEL_FLASH, DEV_SEEK, file->start + offset);
		dev_write(DEV_ATMEL_FLASH, buf, count);
		dev_ioctl(DEV_ATMEL_FLASH, DEV_UNLOCK);
	}
	mos_mutex_unlock(&fs_lock);
	return count;
}

/*int8_t mos_file_close(mos_file* file)
{
	mos_mutex_lock(&fs_lock);
	file->index = -1;
	saveState();
	mos_mutex_unlock(&fs_lock);
	return 0;
}*/

uint16_t mos_file_crc(mos_file* file, uint32_t offset, uint32_t length)
{
	uint16_t crc = 0;
	mos_mutex_lock(&fs_lock);
	uint32_t count = length;
	if (offset >= file->length) {
		count = (uint32_t)0;
	} else if (offset + count > file->length) {
		count = file->length - offset;
	}
	dev_ioctl(DEV_ATMEL_FLASH, DEV_LOCK);
	dev_ioctl(DEV_ATMEL_FLASH, DEV_SEEK, file->start + offset);
	crc = atmel_flash_crc(count);
	dev_ioctl(DEV_ATMEL_FLASH, DEV_UNLOCK);
	mos_mutex_unlock(&fs_lock);
	return crc;
}

void mos_file_flush(mos_file* file)
{
	mos_mutex_lock(&fs_lock);
	dev_ioctl(DEV_ATMEL_FLASH, DEV_LOCK);
	dev_ioctl(DEV_ATMEL_FLASH, DEV_FLUSH);
	dev_ioctl(DEV_ATMEL_FLASH, DEV_UNLOCK);
	mos_mutex_unlock(&fs_lock);
}

void simple_fs_ls()
{
	mos_mutex_lock(&fs_lock);
	printf("nextPage: %d\n", fss.nextPage);
	uint8_t i;
	for (i=0; i<SIMPLE_FS_MAX_FILES; i++)
	{
		if (fss.files[i].index == -1) {
			printf("EMPTY\n");
			continue;
		}
		printf("%C %c%c%c %l %l\n", fss.files[i].index, fss.files[i].name[0], 
			fss.files[i].name[1], fss.files[i].name[2], fss.files[i].start, 
			fss.files[i].length);
	}
	mos_mutex_unlock(&fs_lock);
}

void simple_fs_format()
{
	mos_mutex_lock(&fs_lock);
	//fss.nextPage = 0;
	uint8_t i;
	for (i=0; i<SIMPLE_FS_MAX_FILES; i++)
		fss.files[i].index = -1;
	saveState();
	mos_mutex_unlock(&fs_lock);
}
void simple_fs_init()
{
	mos_mutex_init(&fs_lock);
	mos_mutex_lock(&fs_lock);
	dev_mode(DEV_ATMEL_FLASH, ATMEL_FLASH_MODE_BUFFERED);
	loadState();
	mos_mutex_unlock(&fs_lock);
}

#endif
