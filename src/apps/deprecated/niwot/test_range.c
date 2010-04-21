#include <inttypes.h>
#include "led.h"
#include "dev.h"
#include "com.h"
#include "msched.h"
#include "clock.h"
#include "cc1000.h"

// float floating;
// // static comBuf send_pkt;
// 
// // void send();
// 
void start()
 {
// //     floating = 1.23456;
// //     if(floating == 1.23456)
// //     {printf("it worked\n");}
// //     if(floating == 1)
// //     {printf("it sucked\n");}
// //     floating += 0.12;
// //     if(floating == 1)
// //     {printf("it worked\n");}
//    com_mode(IFACE_RADIO, IF_LISTEN);
//    com_ioctl_IFACE_RADIO(CC1000_TX_POWER, 0x50);
//    
//    mos_thread_new (send, 128, PRIORITY_NORMAL);
//     
 }
// 
// void send()
// {
//    uint8_t seqNo = 1;
//    uint16_t volt = 0;
//    send_pkt.size = 4; //4 bytes
//    
//    while(1/*seqNo<=100*/) //only send 100 packets, then die
//    {  
//       mos_led_toggle(0);
//       
//       dev_open(DEV_MICA2_BATTERY); //--------------------------------battery----------------------------
//       dev_read(DEV_MICA2_BATTERY, &volt, sizeof(volt)); //-----------voltage----------------------------
//       dev_close(DEV_MICA2_BATTERY); //--------------------------------read------------------------------
//       
//       send_pkt.data[0] = seqNo; //sequence number loaded
//       send_pkt.data[1] = volt; //least significant byte loaded
//       send_pkt.data[2] = volt/256; //most significant byte loaded
//       send_pkt.data[3] = 'q'; //packet marker.  this is our packet
// 
//       com_send(IFACE_RADIO, &send_pkt);
//       
//       seqNo++;
//       
//       mos_thread_sleep(500);
//    }
//    
//    while(1)
//    {
//    send_pkt.data[0] = 0;
//    com_send(IFACE_RADIO, &send_pkt);
//    }
// }

  