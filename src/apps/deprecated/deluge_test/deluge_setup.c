/*
  This file is part of MANTIS OS, Operating System for Nymph.
  See http://mantis.cs.colorado.edu/

  Copyright (C) 2003 University of Colorado, Boulder

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  (See http://www.gnu.org/copyleft/gpl.html)
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307,
  USA, or send email to mantis-users@cs.colorado.edu.
*/

#include <inttypes.h>

#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "command_daemon.h"
#include "deluge_impl.h"
#include "boot.h"
#include "dev.h"
#include "com.h"
#include "net.h"
#include "printf.h"
#include "reprogram_commands.h"
#include "simple_fs.h"
#include "node_id.h"
#include <string.h>
#include <avr/pgmspace.h>
#include "plat_dep.h"

const char sMapRow[] ARCH_PROGMEM = "%C: port %C\n";
const char sChooseMap[] ARCH_PROGMEM = "Choose an entry (0 to exit): ";
const char sBadChoice[] ARCH_PROGMEM = "Bad choice %C\n";
const char sPort[] ARCH_PROGMEM = "port #: ";
const char sPortWarning[] ARCH_PROGMEM = "Warning: %C other entries also use port %C, "
			"these must be updated.\n";
const char sVerGoal[] ARCH_PROGMEM = "version %C, goalPage %C,\n";
const char sPages[] ARCH_PROGMEM = "highPage %C, incomingPage %C, program CRC %h\n";
const char sNeeded[] ARCH_PROGMEM = "pages needed: ";
const char sImageSize[] ARCH_PROGMEM = "Image size: %l bytes, %d pages.\n";
const char sVersion[] ARCH_PROGMEM = "version #: ";
const char sClearMem[] ARCH_PROGMEM = "Clearing Deluge memory...";
const char sDone[] ARCH_PROGMEM = "done.\n";
const char sFormat[] ARCH_PROGMEM = "Formatting the file system...";
const char sStatsTitle[] ARCH_PROGMEM = "Node %d:\n";
const char sStatsHead[] ARCH_PROGMEM = " Node Summary Profile Request Data Total\n";
const char sStatsBar[] ARCH_PROGMEM = "----- ------- ------- ------- ---- -----\n";
const char sStatsRow[] ARCH_PROGMEM = "%5C %7d %7d %7d %4d %5d\n";
const char sStatsTot[] ARCH_PROGMEM = "Total %7d %7d %7d %4d %5d\n";
const char sImage[] ARCH_PROGMEM = "Select the image you want to reprogram:\n"
		"\t0: Default image.  Will be loaded on reboot.\n"
		"\t1 and up:\tBackup image that has this index.\n";
const char sDummy[] ARCH_PROGMEM = "number of pages in dummy program: ";
/*const char s[] ARCH_PROGMEM = ;
const char s[] ARCH_PROGMEM = ;
const char s[] ARCH_PROGMEM = ;
const char s[] ARCH_PROGMEM = ;
const char s[] ARCH_PROGMEM = ;
const char s[] ARCH_PROGMEM = ;
const char s[] ARCH_PROGMEM = ;
const char s[] ARCH_PROGMEM = ;
const char s[] ARCH_PROGMEM = ;
const char s[] ARCH_PROGMEM = ;*/

static uint8_t deluge_portmap[DELUGE_INSTANCE_COUNT];
static deluge_control_block dcb;

static void loadPortmap()
{
	dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, DELUGE_DIRECTORY_ADDR);
	dev_read (DEV_AVR_EEPROM, deluge_portmap, sizeof (deluge_portmap));
}

static void savePortmap()
{
	dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, DELUGE_DIRECTORY_ADDR);
	dev_write(DEV_AVR_EEPROM, deluge_portmap, sizeof (deluge_portmap));
}

static void saveState()
{
	dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, DELUGE_CONTROL_BLOCK_ADDR);
	dev_write (DEV_AVR_EEPROM, (uint8_t *)&dcb, DELUGE_CONTROL_BLOCK_SIZE);
}

/*static void loadState()
{
	dev_ioctl (DEV_AVR_EEPROM, DEV_SEEK, DELUGE_CONTROL_BLOCK_ADDR);
	dev_read (DEV_AVR_EEPROM, (uint8_t *)&dcb, DELUGE_CONTROL_BLOCK_SIZE);
}*/

static void deluge_map()
{
	uint8_t i;
	for (i=0; i<DELUGE_INSTANCE_COUNT; i++) {
		printf_P(sMapRow, i+1, deluge_portmap[i]);
	}
}

/*static void deluge_apps()
{
	uint8_t i;
	for (i=0; i<N_LOADED; i++) {
		printf("\nDeluge %C:\n", i+1);
		deluge_print(&apps[i]);
	}
}*/

/*
 * Edit the table that maps Aqueduct instances to port numbers.
 * 
 * The first entry is the home instance.  When a packet is received on this
 * port, the node will behave as a "member" node according to the Aqueduct
 * protocol and will actively try to keep the corresponding image updated.
 * 
 * When a packet arrives on a port for the other entries, the node behaves
 * as a "forwarding" node, and acts as a proxy between member nodes.
 */
static void edit_deluge_map()
{
	loadPortmap();
	while (1)
	{
		deluge_map();
		printf_P(sChooseMap);
		uint8_t i = prompt_uint8("");
		if (i==0) break;
		if (i>DELUGE_INSTANCE_COUNT) {
			printf_P(sBadChoice, i);
			continue;
		}
		i--;
		
		printf_P(sPort);
		deluge_portmap[i] = prompt_uint8("");
		savePortmap();
		
		uint8_t same = 0;
		uint8_t j;
		for (j=0; j<DELUGE_INSTANCE_COUNT; j++) {
			if (i==j) continue;
			if (deluge_portmap[i] == deluge_portmap[j])
				same++;
		}
		if (same) printf_P(sPortWarning, same, deluge_portmap[i]);
	}
}

static void deluge_print4loader()
{
	printf_P(sVerGoal,
		dcb.version, dcb.goalPage);
	printf_P(sPages, 
		dcb.highPage, dcb.incomingPage, dcb.programcrc);

	uint8_t i;
	printf_P(sNeeded);
	for (i=0; i<sizeof(dcb.pagesNeeded); i++)
		printf("%b ", dcb.pagesNeeded[i]);
	printf("\n");
}

/*
 * This command examines the current code image in the file 'prg' and
 * initializes the Aqueduct protocol state for the home instance accordingly.
 * 
 * TODO: This command only initialilizes the home instance.  There probably 
 * should be a command for the other instances, besides format_deluge,
 * which just sets everything to zero.
 */
static void set_deluge_version()
{
	mos_file* image_file = mos_file_open("prg");
	uint16_t pageCount = (image_file->length+DELUGE_PAGE_SIZE-1)/DELUGE_PAGE_SIZE;
	printf_P(sImageSize, image_file->length, pageCount);
	
	printf_P(sVersion);
	dcb.version = prompt_uint8("");
	dcb.programcrc = mos_file_crc(image_file, (uint32_t)0, image_file->length);
	dcb.goalPage = dcb.incomingPage = dcb.highPage = (uint8_t)pageCount;
	dcb.codeSize = image_file->length;
	
	uint8_t i;
	for (i=0; i<sizeof(dcb.pagesNeeded); i++)
		dcb.pagesNeeded[i] = 0;

	saveState();
	
	deluge_print4loader();
}

/*
 * This command erases the Aqueduct state from EEPROM and formats the file
 * system.
 */
void format_deluge()
{
	printf_P(sClearMem);
	uint16_t zero = 0;
	uint16_t addr;
	for (addr = DELUGE_CONTROL_BLOCK_ADDR; addr < SIMPLE_FS_ADDR; addr += 2)
	{
		dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, addr);
		dev_write(DEV_AVR_EEPROM, (uint8_t*)&zero, 2);
	}
	printf_P(sDone);
	
	printf_P(sFormat);
	simple_fs_format();
	printf_P(sDone);

	uint8_t default_portmap[DELUGE_INSTANCE_COUNT];
	uint8_t i;
	for (i=0; i<DELUGE_INSTANCE_COUNT; i++)
		default_portmap[i] = i+1;
	dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, DELUGE_DIRECTORY_ADDR);
	dev_write(DEV_AVR_EEPROM, default_portmap, sizeof (default_portmap));
}

#ifdef DELUGE_KEEP_STATS
static void clear_deluge_stats()
{
	uint16_t row[4] = { 0, 0, 0, 0 };
	uint16_t addr = DELUGE_STATS_ADDR;
	uint8_t i;
	for (i = 0; i < DELUGE_STATS_COUNT; i++)
	{
		dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, addr);
		dev_write(DEV_AVR_EEPROM, (uint8_t*)row, sizeof(row));
		addr += sizeof(row);
	}
}

static void print_deluge_stats()
{
	uint16_t row[4] = { 0, 0, 0, 0 };
	uint16_t total[4] = { 0, 0, 0, 0 };
	uint16_t addr = DELUGE_STATS_ADDR;
	uint8_t i;
	
	printf_P(sStatsTitle, mos_node_id_get());
	printf_P(sStatsHead);
	printf_P(sStatsBar);
	for (i = 0; i < DELUGE_STATS_COUNT; i++)
	{
		dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, addr);
		dev_read(DEV_AVR_EEPROM, (uint8_t*)row, sizeof(row));
		addr += sizeof(row);
		
		printf_P(sStatsRow, i, row[0], row[1], row[2], row[3],
			(row[0]+row[1]+row[2]+row[3]));
		total[0] += row[0];
		total[1] += row[1];
		total[2] += row[2];
		total[3] += row[3]; 
	}
	printf_P(sStatsBar);
	printf_P(sStatsTot, total[0], total[1], total[2], total[3],
		(total[0]+total[1]+total[2]+total[3]));
}
#endif

void deluge_suspend() {
}

void deluge_resume() {
}

static uint8_t target_image = 0;

/*
 * Run this command before running reprogram to store a backup code image.
 * Selecting 0 (or not running this command at all) will cause reprogram
 * to store the image in the default file 'prg', which will then be loaded
 * on reboot.  Storing the image in a backup file will not cause anything to be
 * loaded.  See reboot_net in deluge_command to see how to boot a backup image.
 */
void set_image()
{
	printf_P(sImage);
	target_image = prompt_uint8(":");
}

/*
 * Typing 'reprogram' in mos_shell causes the shell to call this function
 * and store a code image in the file 'rrp'.  Depending on what you entered 
 * for set_image, the file will be renamed to 'prg', which will be loaded on
 * reboot, or to one of the backup files, which will not be loaded.
 */
void reprogram()
{
	// Call the original reprogramming function
	repro_reprogram();
	// Rename the image file
	if (target_image == 0) {
		mos_file_rename("rrp", "prg");
	} else {
		char name[4] = { 'b', 'k', '0'+target_image, 0 };
		mos_file_rename("rrp", name);
		repro_clear_cb();
	}
}

/*
 * Create a fake program image of the specified size.  The image consists of a
 * sequential list of 4-byte integers.  The control block is cleared to ensure
 * the image doesn't get loaded into program flash.  This is useful in doing
 * experiments, since real code images with Aqueduct are getting as big as
 * 60 KB.  This should only be used if Aqueduct was compiled with
 * DELUGE_NO_REPROGRAM defined, otherwise nodes would attempt to execute this.
 */
void dummy()
{
	uint16_t size;
	printf_P(sDummy);
	size = prompt_long("");
	size *= DELUGE_PAGE_SIZE;
	
	mos_file* prg = mos_file_create("prg", (uint32_t)size);
	
	uint32_t i;
	for (i = (uint32_t)0; i < size; i += (uint32_t)4)
		mos_file_write((uint8_t*)&i, prg, i, (uint32_t)4);
	
	// We don't want to accidentally execute this thing
	repro_clear_cb();
}

void default_setup()
{
	format_deluge();

	mos_file* prg = mos_file_create("prg", (uint32_t)0);
	
	// We don't want to accidentally execute this thing
	repro_clear_cb();

	uint16_t pageCount = (prg->length+DELUGE_PAGE_SIZE-1)/DELUGE_PAGE_SIZE;
	printf_P(sImageSize, prg->length, pageCount);
	
	printf_P(sVersion);
	dcb.version = 1;
	dcb.programcrc = mos_file_crc(prg, (uint32_t)0, prg->length);
	dcb.goalPage = dcb.incomingPage = dcb.highPage = (uint8_t)pageCount;
	dcb.codeSize = prg->length;
	
	uint8_t j;
	for (j=0; j<sizeof(dcb.pagesNeeded); j++)
		dcb.pagesNeeded[j] = 0;

	saveState();
	
	deluge_print4loader();
}

void start(void)
{
	simple_fs_init();
	
	mos_command_daemon_init();
	mos_register_function("ls", simple_fs_ls);
	mos_register_function("format", simple_fs_format);
	//mos_register_function("deluge_apps", deluge_apps);
	//mos_register_function("deluge_map", deluge_map);
	mos_register_function("format_deluge", format_deluge);
	mos_register_function("edit_deluge_map", edit_deluge_map);
	mos_register_function("set_deluge_version", set_deluge_version);
	mos_register_function("set_image", set_image);
	mos_register_function("DR", reprogram);
	mos_register_function("clear_cb", repro_clear_cb);
	mos_register_function("get_cb", repro_get_cb);
	mos_register_function("dummy", dummy);
	mos_register_function("default", default_setup);
	#ifdef DELUGE_KEEP_STATS
	mos_register_function("clear_deluge_stats", clear_deluge_stats);
	mos_register_function("print_deluge_stats", print_deluge_stats);
	#endif
	mos_thread_new(mos_command_daemon, MOS_COMMANDER_STACK_SIZE, PRIORITY_NORMAL);
}
