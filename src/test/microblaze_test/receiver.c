//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "mos.h"
#include "sem.h"
#include "com.h"
#include "dev.h"
#include "printf.h"
#include "fix_fft.h"
#include "show_amp.h"
#include "mb-timer.h"

#include "command_daemon.h"

#include "clock.h"


static uint8_t receiver_continue;

void receiver();

void start_receiver()
{
    receiver_continue = true;
    if(mos_thread_new (receiver, 1024, PRIORITY_NORMAL) != THREAD_OK)
        show_thread_err();
}

void stop_receiver()
{
    receiver_continue = false;
}

//receiver thread
#define PACKETS_PER_CYCLE 250

#ifdef RADIO_USE_FEC
extern uint16_t cc1000_fec_error_count;
#endif

static uint16_t num_samples;

void receiver()
{
    comBuf send_pkt;
    uint16_t total_pkts = 0;
    uint32_t total_packet_count;
    uint16_t total_packets_dropped;
    float packet_percent;
    uint16_t packet_int;
    uint16_t packet_frac;
    uint16_t num_pkts_dropped = 0;
    uint16_t fec_errs;
    uint16_t last_seq = 0;
    uint16_t seq;
    comBuf *recv_pkt;
    bool first_run = true;


    printf("Radio tester initialized....\n");
    com_mode(IFACE_RADIO,IF_LISTEN);

    //use this instead of printf for efficiency
    send_pkt.size = 2;
    send_pkt.data[1] = '\0';

    while(1) {     
        mos_led_toggle(2);

        //printf("about to recv\n");
        recv_pkt = com_recv(IFACE_RADIO);


        total_pkts++;
        seq = *(uint8_t *)(&recv_pkt->data[1]);
        seq <<= 8;
        seq |= *(uint8_t *)(&recv_pkt->data[0]);

        if(first_run == true) {
            first_run = false;
            last_seq = seq;
        } else {
            if(seq != last_seq + 1) {
             //   printf("recvd size: %d seq: %x\n", recv_pkt->size, seq);
                uint16_t dropped;

                if(seq > last_seq)
                    dropped = seq - last_seq - 1;
                else
                    dropped = 0xffff - last_seq + seq;

                //printf("dropped: %d, seq: %d, last_seq: %d\n", dropped, seq, last_seq);

                num_pkts_dropped += dropped;
                total_pkts += dropped;
                send_pkt.data[0] = 'm';
                com_send(IFACE_SERIAL, &send_pkt);
            } else {
#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICA2DOT)
#ifdef RADIO_USE_FEC
                if(fec_errs != cc1000_fec_error_count) { //a new fec error
                    //printf("%s", "f");
                    fec_errs = cc1000_fec_error_count;
                }
#endif
#endif
                send_pkt.data[0] = '.';
                com_send(IFACE_SERIAL, &send_pkt);
            }
            last_seq = seq;
        }

        com_free_buf(recv_pkt);
        //printf("buf freed\n");

        if(total_pkts >= PACKETS_PER_CYCLE) {
            total_packet_count += total_pkts;
            total_packets_dropped += num_pkts_dropped;

            packet_percent = (float)num_pkts_dropped / (float)total_pkts;

            packet_int = (uint16_t)(packet_percent * 1000.0);
            packet_frac = packet_int % 10;
            packet_int /= 10;

//            printf("Dropped: %d, Total: %d, Percent: %d.%d\n", num_pkts_dropped,
//                    total_pkts, packet_int, packet_frac);

            //eeprom logging

            /* 	 dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, DATA_INDEX + num_samples * */
            /* 		   sizeof(num_samples) * 2); */
            /* 	 dev_write(DEV_AVR_EEPROM, &packet_int, sizeof(packet_int)); */
            /* 	 dev_write(DEV_AVR_EEPROM, &packet_frac, sizeof(packet_frac)); */

            /* 	 num_samples++; */
            /* 	 dev_ioctl(DEV_AVR_EEPROM, DEV_SEEK, SIZE_INDEX); */
            /* 	 dev_write(DEV_AVR_EEPROM, &num_samples, sizeof(num_samples)); */

#if defined(PLATFORM_MICA2) || defined(PLATFORM_MICA2DOT)
            printf("crc errs: %d ",cc1000_crc_error_count);
#ifdef RADIO_USE_FEC
            printf("fec errs: %d ",cc1000_fec_error_count);
#endif
            printf ("size errs: %d ", cc1000_size_error_count);
            printf ("mem errs: %d ", cc1000_mem_error_count);
            printf("sync errs: %d \n",cc1000_sync_error_count);
#endif

            num_pkts_dropped = 0;
            total_pkts = 0;
        }      
    }

}
