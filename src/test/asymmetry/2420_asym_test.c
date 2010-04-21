// Simple program to send and record ping packets from neighbors over time.
// No complicated MAC or timing is used.  Rahter, this is meant to discover
// asymmetric paths present in the environment.
//
// Temporary results are stored into flash.  Writting to flash is known to be
// expensive, but with fresh batteries this application should still run long
// enough to give good results.
//
// Written By: Cyrus Hall <hallc@lu.unisi.ch>
// Date:       March 6th 2006
//

#include <string.h>

#include "mos.h"
#include "msched.h"
#include "clock.h"
#include "printf.h"
#include "node_id.h"
#include "clock.h"
#include "com.h"
#include "command_daemon.h"
#include "led.h"
#include "mutex.h"
#include "cc2420.h"

#include "log.h"

#define RECORD_SIZE 8

static comBuf send_buf;

void ping_generator(void) {
    //com_ioctl(IFACE_RADIO, CC2420_TX_POWER, 0x70);
    //com_ioctl(IFACE_RADIO, CC2420_HIGH_POWER_MODE);
    com_ioctl(IFACE_RADIO, CC2420_LOW_POWER_MODE);

    //high byte first
    buf_insert16(send_buf.data, 0, mos_node_id_get());
    send_buf.data[2] = '\0'; //padding to get the radio to send
    send_buf.size = 3;

    mos_thread_sleep(1000);

    while(1) {
        com_send(IFACE_RADIO, &send_buf);
        mos_led_toggle(0);
        mos_thread_sleep(30000);  //once every 60 seconds
    }
}

void ping_listener(void) {
    comBuf   *recv_buf;
    uint8_t   buf[RECORD_SIZE];
    uint16_t  rssi;

    com_mode(IFACE_RADIO, IF_LISTEN);

    while(1) {
        recv_buf = com_recv(IFACE_RADIO);
        rssi = cc2420_get_last_rssi();
        
        if(recv_buf != NULL) {
            mos_led_toggle(1);

            printf("recved pkt from %d, rssi %d.\n", buf_extract16(recv_buf->data, 0), rssi);

            //this procedure uses RECORD_SIZE bytes each write
            buf_insert32(buf, 0, mos_get_realtime());
            buf[4] = recv_buf->data[0];
            buf[5] = recv_buf->data[1];
            buf_insert32(buf, 6, rssi);

            if(append_log(buf) == 0) {
                printf("log is full, quiting.\n");
                com_free_buf(recv_buf);

                return;
            }

            com_free_buf(recv_buf);
        }
    }
}

// Print the recorded results to the serial
void query_flash() {
    uint8_t  *buf;
    uint32_t  time;
    uint16_t  i, node, rssi;

    i = 0;
    printf("---B(%d)---\n", mos_node_id_get());
    
    mos_mutex_lock(&open_log.file_lock);
    while((buf = read_log_index(i++, NULL)) != NULL) {
        time = buf_extract32(buf, 0);
        node = buf_extract16(buf, 4);
        rssi = buf_extract16(buf, 6);

        if(time == 0 && node == 0) {
            printf("reset\n");
        } else {
            //printf("@%l, ping <- %d\n", time, node);
            printf("%l:%d %d\n", time, node, rssi);
        }
    }
    printf("---E---\n");
    mos_mutex_unlock(&open_log.file_lock);
}

uint8_t read_buf[RECORD_SIZE];

// MOS main init function - initialize devices, filesystem, start threads
void start(void) {
    //clear_log();
    
    mos_command_daemon_init();
    mos_register_function("dump", query_flash);
    mos_register_function("clear", clear_log);

    reopen_log("log", 0xFFFF, RECORD_SIZE, 0xFF, &read_buf[0]);
    
    printf("Starting threads.\n");
    mos_thread_new(mos_command_daemon, 192, PRIORITY_NORMAL);
    printf("\t...command daemon\n");
    mos_thread_new(ping_generator, 192, PRIORITY_NORMAL);
    printf("\t...ping_generator\n");
    mos_thread_new(ping_listener, 192, PRIORITY_NORMAL);
    printf("\t...ping_listener\n");
}
