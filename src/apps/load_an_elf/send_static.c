#include <inttypes.h>

#include "mos.h"
#include "msched.h"
#include "clock.h"
#include "loader/elfloader.h"
#include "prog2load_buf.h"
#include "symbols.h"
#include "com.h"

#define PACKET_NR_OFFSET 4 
#define DATA_OFFSET 6 
#define DATASIZE (COM_DATA_SIZE-DATA_OFFSET)
#define SLEEPTIME 2000

#define RESEND_SLEEP 30
#define RESEND_ALL_SLEEP 1000*20
static comBuf pkt_to_send;

/* Sends static ELF data located in the prog2load buffer
   (prog2load_buf.c). */
void start(void) {
  uint16_t packet_nr = 0;

  com_mode(IFACE_RADIO, IF_LISTEN);

  printf("send_static started\n\n");
  
  /* Send forever */
  while (1) {
	int program_size = sizeof(prog2load);
	int nr_packets_to_send = program_size / DATASIZE + 1;
	
	while (packet_nr < nr_packets_to_send) {
		int left_to_send = program_size - packet_nr*DATASIZE;
		mos_led_toggle (0);
	    
	    pkt_to_send.size = COM_DATA_SIZE;
	    
	    // ID
	    pkt_to_send.data[0] = ELFPKT_BYTE0;
	    pkt_to_send.data[1] = ELFPKT_BYTE1;
	    pkt_to_send.data[2] = ELFPKT_BYTE2;
	    pkt_to_send.data[3] = ELFPKT_BYTE3;

		// Packet number
		memcpy(&pkt_to_send.data[PACKET_NR_OFFSET], &packet_nr, sizeof(uint16_t));
		
		// Data
		if (left_to_send < DATASIZE) {
			printf("padding zeroes (reached end of program)\n");
			memset(&pkt_to_send.data[DATA_OFFSET], 0, DATASIZE);
			memcpy(&pkt_to_send.data[DATA_OFFSET], &prog2load[DATASIZE*packet_nr], left_to_send);
		} else {
			memcpy(&pkt_to_send.data[DATA_OFFSET], &prog2load[DATASIZE*packet_nr], DATASIZE);
		}
		
		// Send data packet
	    printf("sending data: %d\n", packet_nr);
	    com_send(IFACE_RADIO, &pkt_to_send);
		packet_nr++;
	    mos_thread_sleep(RESEND_SLEEP);
	}


	// Send no data termination packet
	pkt_to_send.size = DATA_OFFSET;

    // ID
    pkt_to_send.data[0] = ELFPKT_BYTE0;
    pkt_to_send.data[1] = ELFPKT_BYTE1;
    pkt_to_send.data[2] = ELFPKT_BYTE2;
    pkt_to_send.data[3] = ELFPKT_BYTE3;

	// Packet number
	memcpy(&pkt_to_send.data[PACKET_NR_OFFSET], &packet_nr, sizeof(uint16_t));
		
	// Send data packet
    printf("sending termination packet: %d\n", packet_nr);
    com_send(IFACE_RADIO, &pkt_to_send);
	packet_nr++;
    mos_thread_sleep(30);

	packet_nr = 0;

  mos_thread_sleep (RESEND_ALL_SLEEP);

  }
}
