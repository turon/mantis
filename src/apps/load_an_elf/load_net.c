#ifndef USING_ELF
	ERROR, DEPENDS ON ELF RADIO THREAD (gcc -DUSING_ELF)
#endif

#include <inttypes.h>
#include "mos.h"
#include "msched.h"
#include "clock.h"
#include "com.h"  /* Contains ELF variables */
#include "loader/elfloader.h"
#include "prog2load_buf.h"
#include "symbols.h"

#define PACKET_NR_OFFSET 4 
#define DATA_OFFSET 6 
#define DATASIZE (COM_DATA_SIZE-DATA_OFFSET)
#define SLEEPTIME 2000

short last_packet_nr;
char received_termination;

void elf_netload (void) {
     int elfloader_err;
     uint32_t sleep_time = 4000;
     uint16_t packet_nr = 0;
     uint16_t packet_nrfake = 111;

	/* Store thread handle for later from com.c */
	elfthread = mos_thread_current();

	/* Make sure packets will not be dropped before comparison */
    com_mode(IFACE_RADIO, IF_LISTEN);

  printf("elf_netload started\n");

  while (1) {

	last_packet_nr = -1;
	received_termination = 0;
	
   while (!received_termination) {
	 /* Suspend thread waiting for radio data - similar to com_recv() */
	mos_thread_suspend();

   	  /* Analyze packet data */
   	  if (is_elf_packet(elfbuf)) {
   	      packet_nr = -1;

		// Check header for packet number
		memcpy(&packet_nr, &elfbuf->data[PACKET_NR_OFFSET], sizeof(uint16_t));
		
		if (last_packet_nr + 1 >= packet_nr) {
			if (elfbuf->size == DATA_OFFSET) {
				// Received termination packet...
			    printf("received termination packet: %d\n", packet_nr);
			    received_termination = 1;
			} else {
	   	  		last_packet_nr = packet_nr;
			    printf("writing data to elfstore: %d\n", packet_nr);
   				elfstore_write(&elfbuf->data[DATA_OFFSET], (DATASIZE)*packet_nr, DATASIZE);
   			}		    
		   
   	  	} else {
			// ignore out of synch packets
     	  printf("received out of sync elf packet %d\n", packet_nr);
   	  	}
    
   	  } else {
     	  printf("error, elf_netload() received non-ELF data\n");
   	  }

      com_free_buf(elfbuf);
      mos_led_toggle (2);
   }

  printf("loading elf now\n");
  
  /* Load and unload forever */
    if (elfloader_loaded_unload != NULL) {
      printf("shutting down already started application...\n");
      elfloader_loaded_unload();
      elfloader_loaded_unload == NULL;
    }
    
    printf("sleeping...\n");
    mos_thread_sleep (SLEEPTIME);

    /* Load ELF */
    printf("loading ELF now...");
    elfloader_err = elfloader_load();
    if (elfloader_err != ELFLOADER_OK) {
      printf("failed\n");
      printf("ELF loader returned error %C, elfloader_unknown = %s\n", elfloader_err, elfloader_unknown);
      printf("symbols size is %C\n", sizeof(symbols));
    } else {
      /* Start loaded application */
      printf("success\n");
      printf("sleeping...\n");
      mos_thread_sleep (SLEEPTIME);
      
      printf("starting loaded application now...\n");
      elfloader_loaded_load();
    }
    
    printf("waiting until duplicate packets gone\n");
    mos_thread_sleep (SLEEPTIME);
    
  }
}
   
void start(void) {
  mos_thread_new (elf_netload, 128, PRIORITY_NORMAL);
}
