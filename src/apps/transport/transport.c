//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

// A simple stop-and-wait transport protocol with end-to-end retransmits

#include <inttypes.h>

#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "mutex.h"
#include "string.h"
#include "com.h"
#include "printf.h"
#include "clock.h"

#define NODE_ID 4
#define BASE_NODE_ID 1
#define LAST_NODE_ID 4
#define ACK_TIMEOUT 10000 // Wait for up to 10 seconds for an ACK
#define COM_RECV_TIMEOUT 100
#define ACK_FLAG 1

// Array indices into packet for data members
#define SIZE_INDEX    0
#define SEQ_NUM_INDEX 1
#define NODE_ID_INDEX 5
#define ACK_INDEX     6

// Sizes in bytes for packets and ACK packets 
#define PKT_SIZE      7
#define ACK_PKT_SIZE  4

// Max number of retransmits attempted for each packet sent
#define MAX_RETRANSMITS 4

// Flag used to indicate when a packet is received by the receive thread
uint8_t packetRecvd = 0; 

// Flag used to indicate when an ACK packet is received by the receive thread
uint8_t ackRecvd = 0;

uint32_t sendSeqNum = 1;
uint32_t recvSeqNum = 0;
comBuf sendPkt;
comBuf* recvPkt;

// Function prototypes
void sendThread();
void receiveThread();
void sendPacket(uint32_t);
void ackTimeoutCallBack(void*);

// Counter for current number of retransmits sent
uint8_t numRetransmits = 0;

uint8_t recvdPktNodeId = 0;

// Mutexes for access to sendPkt and recvPkt
mos_mutex_t sendPktMutex;
mos_mutex_t recvPktMutex;

// Alarm for ACK timeout
mos_alarm_t ackAlarm;
uint8_t ackTimeout = 0;

void sendThread() {
    uint8_t nextNodeId = 0;
    uint32_t ackSeqNum = 0;
    uint8_t recvPktSize;
    
    sendPkt.size = PKT_SIZE; // 6 bytes

    com_mode(IFACE_RADIO, IF_LISTEN);
   
    while (1) {
	    // Base station
	    if (NODE_ID == BASE_NODE_ID) {
            /*
	        // Store sequence number in packet
	        memcpy(&(sendPkt.data[SEQ_NUM_INDEX]), &sendSeqNum, 
                sizeof(sendSeqNum));
	        
	        // Store node id for next hop in packet
	        nextNodeId = BASE_NODE_ID + 1;
	        memcpy(&(sendPkt.data[NODE_ID_INDEX]), &nextNodeId, 
                sizeof(nextNodeId));
	        
	        // Send packet
	        com_send(IFACE_RADIO, &sendPkt);
            */
            // printf("Locking recvPktMutex... \n");
            // mos_mutex_lock(&recvPktMutex);
            // printf("Locked recvPktMutex \n");
            
            // Send packet
            sendPacket(sendSeqNum);
            
            // Set alarm for ACK timeout
            ackAlarm.msecs = ACK_TIMEOUT;
            ackAlarm.reset_to = 0;
            mos_alarm(&ackAlarm); 
	        
            while (ackTimeout == 0) {
                /*
    	        // Receive ACK for this sequence number
    	        recvPkt = com_recv_timed(IFACE_RADIO, COM_RECV_TIMEOUT);
                
                if (recvPkt == NULL) {
                    continue;
                }
                
                // Packet received.  Check that it's a valid packet
                // Get packet size from packet data
                memcpy(&recvPktSize, &(recvPkt->data[SIZE_INDEX]), 
                    sizeof(recvPktSize));
                */
                    
                // Valid packet received
                
                /*
                if (recvPktSize == PKT_SIZE) {
                    // Get seq num from received ACK packet
                    memcpy(&ackSeqNum, &(recvPkt->data[SEQ_NUM_INDEX]), 
                        sizeof(ackSeqNum));
                    
                    // Get node ID field value from received packet
                    memcpy(&recvdPktNodeId, &(recvPkt->data[NODE_ID_INDEX]), 
                        sizeof(recvdPktNodeId));
                        
                    // Get ACK flag from received ACK packet
                    memcpy(&ackRecvd, &(recvPkt->data[ACK_INDEX]), 
                        sizeof(ackRecvd));
                 */
                    
                // Check that received packet is the ACK for this seqNum
                mos_mutex_lock(&recvPktMutex);
                if ((recvSeqNum == sendSeqNum) && (ackRecvd == 1) && 
                    (recvdPktNodeId == NODE_ID)) {
                    printf("ACK received for seq num = %l \n", recvSeqNum);
                    
                    // Expected ACK successfully received
                    // Seq num for next packet
                    sendSeqNum++;
                    
                    // ACK has been received, so clear ackRecvd flag
                    ackRecvd = 0;
                    
                    com_free_buf(recvPkt);
                    mos_mutex_unlock(&recvPktMutex);
                    
                    // Cancel (remove) ACK timeout alarm
                    mos_remove_alarm(&ackAlarm);
                    
                    break;
                }
                else if (packetRecvd == 1) {
                    com_free_buf(recvPkt);
                }
                mos_mutex_unlock(&recvPktMutex);
                // }
                
                // com_free_buf(recvPkt);
            }
            
            printf("ackTimeout = %d \n", ackTimeout);
            
            // Process ACK timeout
            if (ackTimeout == 1) {
                // ACK timeout alarm fired
                if (numRetransmits < MAX_RETRANSMITS) {
                    numRetransmits++;
                    
                    // Clear ackTimeout flag, since packet will be resent
                    ackTimeout = 0;
                    
                    // Cancel (remove) ACK timeout alarm
                    mos_remove_alarm(&ackAlarm);
                    
                    // Resend packet
                }
                else {
                    // MAX_RETRANSMITS attempted, move on to next packet
                    sendSeqNum++;
                    numRetransmits = 0;
                }    
            }
                  
            // mos_mutex_unlock(&recvPktMutex);
            
            printf("Continuing loop... \n");                		
	    }
	    // Node is not base station and is not last node 
	    else if (NODE_ID > BASE_NODE_ID && NODE_ID < LAST_NODE_ID) {
            mos_mutex_lock(&recvPktMutex);
            // Forward packet to next hop, if a packet has been received
            if ((packetRecvd == 1) && (recvdPktNodeId == NODE_ID) && 
                (ackRecvd != 1)) {    
		        // Store node id for next hop in packet
		        nextNodeId = NODE_ID + 1;
		        memcpy(&(recvPkt->data[NODE_ID_INDEX]), &nextNodeId, 
                    sizeof(nextNodeId));
                    
                printf("Forwarding received packet to node %d \n", nextNodeId);
		        
		        // Forward (send) received packet to next hop
		        com_send(IFACE_RADIO, recvPkt);
                
                com_free_buf(recvPkt);
                
                // Received packet has been forwarded, so clear packetRecvd 
                // flag
                packetRecvd = 0;
            } 
            // Forward ACK packet to next hop, if ACK was received
            else if (ackRecvd == 1) {
                // Store node id for next hop in packet
                nextNodeId = NODE_ID - 1;
                memcpy(&(recvPkt->data[NODE_ID_INDEX]), &nextNodeId, 
                    sizeof(nextNodeId));
                
                printf("Forwarding received ACK packet to node %d \n", nextNodeId);
                    
                // Forward (send) received ACK packet to next hop
                com_send(IFACE_RADIO, recvPkt);
                
                com_free_buf(recvPkt);
                
                // Received ACK has been forwarded, so clear packetRecvd 
                // and ackRecvd flags
                packetRecvd = 0;
                ackRecvd = 0;    
            }
            mos_mutex_unlock(&recvPktMutex);    
	    }
	    // Last node
	    else if (NODE_ID == LAST_NODE_ID) {
            mos_mutex_lock(&recvPktMutex);
            // Send ACK back to base station, if a packet has been received
            if ((packetRecvd == 1) && (recvdPktNodeId == NODE_ID)) {
                printf("Sending ACK... \n");
                
		        // uint32_t recvSeqNum = 0;
		        
		        // Get seq num for received packet
                /*
		        memcpy(&recvSeqNum, &(recvPkt->data[SEQ_NUM_INDEX]), 
                    sizeof(recvSeqNum)); */
		        
                // Done with recvPkt, so free it
                com_free_buf(recvPkt);
                
                // Store packet size in packet
                memcpy(&(sendPkt.data[SIZE_INDEX]), &(sendPkt.size), 
                    sizeof(sendPkt.size));
                
		        // Store received seq num in ACK packet to send
		        memcpy(&(sendPkt.data[SEQ_NUM_INDEX]), &recvSeqNum, 
                    sizeof(recvSeqNum));
		         
		        // Store node id for next hop in ACK packet to send
		        nextNodeId = LAST_NODE_ID - 1;
		        memcpy(&(sendPkt.data[NODE_ID_INDEX]), &nextNodeId, 
                   sizeof(nextNodeId));
		        
                // Set ACK flag in packet to send
		        memset(&(sendPkt.data[ACK_INDEX]), ACK_FLAG, sizeof(ACK_FLAG));
		        
		        // Send ACK for received packet
		        com_send(IFACE_RADIO, &sendPkt);
                
                printf("Sent ACK for packet with seq num = %l \n", recvSeqNum);
                
                // Received packet has been processed, so clear packetRecvd 
                // flag
                packetRecvd = 0;
            }
            mos_mutex_unlock(&recvPktMutex);    
	    }
	    
	    mos_thread_sleep(1000);
    }
}

void receiveThread(){
    com_mode(IFACE_RADIO, IF_LISTEN);
    uint8_t recvPktSize;
    // uint32_t recvSeqNum = 0;
   
    while (1) {    
        // Receive packet
        // recvPkt = com_recv_timed(IFACE_RADIO, COM_RECV_TIMEOUT);
        recvPkt = com_recv(IFACE_RADIO);
        
        printf("In receiveThread(), locking recvPktMutex... \n");
        mos_mutex_lock(&recvPktMutex);
        printf("In receiveThread(), locked recvPktMutex... \n");
        
        /*
        if (recvPkt == NULL) {
            mos_mutex_unlock(&recvPktMutex);
            mos_thread_sleep(100);
            continue;
        }
        */
        
        printf("Received data \n");
        
        // Get packet size from packet data
        memcpy(&recvPktSize, &(recvPkt->data[SIZE_INDEX]), 
            sizeof(recvPktSize));
            
        // Get node ID field value from received packet
        memcpy(&recvdPktNodeId, &(recvPkt->data[NODE_ID_INDEX]), 
            sizeof(recvdPktNodeId));
            
        // Valid packet for this node received
        if ((recvPktSize == PKT_SIZE) && (recvdPktNodeId == NODE_ID)) {
            // Get seq num for received packet
            memcpy(&recvSeqNum, &(recvPkt->data[SEQ_NUM_INDEX]), 
                sizeof(recvSeqNum));
            
            // Print packet's seq num
            printf("Received seqNum = %l\n", recvSeqNum);
            
            printf("Received packet for recvdPktNodeId = %d \n", 
                recvdPktNodeId); 
            
            // Set packetRecvd flag to indicate that a packet was received
            packetRecvd = 1;       
            
            // Determine if an ACK packet was received
            // Set ackRecvd flag to indicate that an ACK was received    
            memcpy(&ackRecvd, &(recvPkt->data[ACK_INDEX]), sizeof(ackRecvd)); 
            
            mos_led_toggle(1);
            // mos_thread_sleep(500);
        }
        else {
            // Free invalid/unused packet
            com_free_buf(recvPkt);    
        }
        
        // mos_thread_sleep(100);
        mos_mutex_unlock(&recvPktMutex);      
    }
}

// Send a packet from the base station to it's next hop
void sendPacket(uint32_t seqNum) {
    uint8_t nextNodeId = 0;
    
    sendPkt.size = PKT_SIZE; // 7 bytes
    
    // Store packet size in packet
    memcpy(&(sendPkt.data[SIZE_INDEX]), &(sendPkt.size), 
        sizeof(sendPkt.size));
    
    // Store sequence number in packet
    memcpy(&(sendPkt.data[SEQ_NUM_INDEX]), &sendSeqNum, 
        sizeof(sendSeqNum));
    
    // Store node id for next hop in packet
    nextNodeId = BASE_NODE_ID + 1;
    memcpy(&(sendPkt.data[NODE_ID_INDEX]), &nextNodeId, 
        sizeof(nextNodeId));
    
    // Send packet
    com_send(IFACE_RADIO, &sendPkt);
    
    printf("Sent packet with seq num = %l \n", sendSeqNum);
    mos_led_toggle(0); 
}

// Callback function called when an ACK receive has timed out
void ackTimeoutCallBack(void* data) {
    ackTimeout = 1;
    
    mos_led_toggle(2);
}
   
/* The start function is automatically called by the operating system,
   for single-threaded applications, treat start() like main(). for multi-
   threaded applications, start each thread in the start() function.*/
void start(void) {
    mos_mutex_init(&recvPktMutex);
    mos_mutex_init(&sendPktMutex);
    
    // Initialize ACK timeout alarm
    ackAlarm.func = ackTimeoutCallBack;
    ackAlarm.msecs = ACK_TIMEOUT;
    ackAlarm.reset_to = 0;
    
    mos_thread_new(sendThread, 128, PRIORITY_NORMAL);
    mos_thread_new(receiveThread, 128, PRIORITY_NORMAL);
}
