//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

// A simple stop-and-wait transport protocol with end-to-end retransmits.
//
// - Added support for timeout estimation using Jacobsen/Karels timeout 
//   algorithm.
// - Added support for port numbers to multiplex/demultiplex the connection. 

#include <inttypes.h>

#include <stdlib.h>
#include <string.h>
#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "mutex.h"
// #include "string.h"
#include "com.h"
#include "printf.h"
#include "clock.h"
#include "queue.h"

#define NODE_ID 2
#define BASE_NODE_ID 1
#define LAST_NODE_ID 2
#define ACK_TIMEOUT 10000 // Wait for up to 10 seconds for an ACK
#define COM_RECV_TIMEOUT 100
#define ACK_FLAG 1

// Array indices into packet header for data members
#define SIZE_INDEX    0
#define SEQ_NUM_INDEX 1
#define NODE_ID_INDEX 5
#define ACK_INDEX     6
#define PORT_INDEX    7
#define DATA_INDEX    8 // Starting index for packet data

// Each packet contains up to 16 bytes of data
#define PKT_DATA_SIZE 16

// Sizes in bytes for packets and ACK packets 
#define PKT_SIZE      DATA_INDEX + PKT_DATA_SIZE
#define ACK_PKT_SIZE  PORT_INDEX + 1

// Max number of retransmits attempted for each packet sent
#define MAX_RETRANSMITS 4

// Max port number
#define MAX_PORT_NUM 15

// Max data elements in each port queue.  Each data element contains the data
// payload for one packet.
#define MAX_QUEUE_LEN 15

// Flag used to indicate when a packet is received by the receive thread
uint8_t packetRecvd = 0; 

// Flag used to indicate when an ACK packet is received by the receive thread
uint8_t ackRecvd = 0;

uint32_t sendSeqNum = 1;
uint32_t recvSeqNum = 0;
comBuf sendPkt;
comBuf* recvPkt;

// Array of packet receive queues, one for each port
queue_t receiveQueues[MAX_PORT_NUM + 1];

// Array of buffers, one for each receive queue
uint8_t receiveBufs[MAX_PORT_NUM + 1][PKT_DATA_SIZE * MAX_QUEUE_LEN];

// Function prototypes
void sendThread();
void receiveThread();
void ackTimeoutCallBack(void*);
void addToReceiveQueue(uint8_t*, int, int);
void receiveData(uint8_t, uint8_t*, int);
void sendPacket(uint8_t, uint8_t*, uint8_t);
void sendData(uint8_t, uint8_t*, uint8_t);

// Counter for current number of retransmits sent
uint8_t numRetransmits = 0;

uint8_t recvdPktNodeId = 0;
uint8_t recvdPktPortNum = 0;

// Mutexes for access to sendPkt and recvPkt
mos_mutex_t sendPktMutex;
mos_mutex_t recvPktMutex;

// Alarm for ACK timeout
mos_alarm_t ackAlarm;
uint8_t ackTimeoutFlag = 0;

void sendThread() {
    uint8_t nextNodeId = 0;
    /* uint32_t ackSeqNum = 0;
    uint8_t recvPktSize;
    uint32_t startTime = 0;
    uint32_t endTime = 0;
    int32_t sampleRtt = 0;
    uint32_t estimatedRtt = ACK_TIMEOUT / 2;
    uint32_t deviation = estimatedRtt / 2;
    uint32_t timeOut = ACK_TIMEOUT;
    */
    // float timeoutAlpha = 1.5; // Alpha, used for timeout calculation
    uint8_t sendPortNum = 0;
    char testStr[] = "testdatamorefiller";
    char sendPortNumStr[3];
    char sendSeqNumStr[4];
    char testData[strlen(testStr) + sizeof(sendPortNumStr) + 
        sizeof(sendSeqNumStr) + 3 + 1];
    char recvdDataBuf[sizeof(testData)];
    
    // Used to skip printing of received data, for debugging
    uint8_t skipRecv = 1; 
    
    // sendPkt.size = PKT_SIZE;

    com_mode(IFACE_RADIO, IF_LISTEN);
   
    while (1) {
	    // Base station
	    if (NODE_ID == BASE_NODE_ID) {
            // Build testData string, of form 
            // "testdata${sendPortNumStr},s:{seqNum}"
            memset(testData, 0, sizeof(testData));
            memset(sendSeqNumStr, 0, sizeof(sendSeqNumStr));
            itoa(sendPortNum, sendPortNumStr, 10);
            strcpy(testData, testStr);
            strcat(testData, sendPortNumStr);
            strcat(testData, ",s:");
            itoa(sendSeqNum, sendSeqNumStr, 10);
            strcat(testData, sendSeqNumStr);
            
            printf("Calling sendData() for: %s \n", testData);
            sendData(sendPortNum, testData, sizeof(testData));
            
            // Send five packets to a port before incrementing port num
            if (sendSeqNum % 5 == 0) {
                sendPortNum++;
            }
            // Reset port number (for testing)
            if (sendPortNum == MAX_PORT_NUM) {
                sendPortNum = 0;
            }
            
//            /*
//	        // Store sequence number in packet
//	        memcpy(&(sendPkt.data[SEQ_NUM_INDEX]), &sendSeqNum, 
//                sizeof(sendSeqNum));
//	        
//	        // Store node id for next hop in packet
//	        nextNodeId = BASE_NODE_ID + 1;
//	        memcpy(&(sendPkt.data[NODE_ID_INDEX]), &nextNodeId, 
//                sizeof(nextNodeId));
//	        
//	        // Send packet
//	        com_send(IFACE_RADIO, &sendPkt);
//            */
//            // printf("Locking recvPktMutex... \n");
//            // mos_mutex_lock(&recvPktMutex);
//            // printf("Locked recvPktMutex \n");
//            
//            // Get current elapsed time, for use in determining RTT for ACK
//            // received for sent packet
//            startTime = mos_get_realtime();
//            
//            // Send packet
//            sendPacket(sendSeqNum, sendPortNum);
//            
//            // Set alarm for ACK timeout
//            ackAlarm.msecs = timeOut;
//            ackAlarm.reset_to = 0;
//            mos_alarm(&ackAlarm); 
//	        
//            while (ackTimeoutFlag == 0) {
//                /*
//    	        // Receive ACK for this sequence number
//    	        recvPkt = com_recv_timed(IFACE_RADIO, COM_RECV_TIMEOUT);
//                
//                if (recvPkt == NULL) {
//                    continue;
//                }
//                
//                // Packet received.  Check that it's a valid packet
//                // Get packet size from packet data
//                memcpy(&recvPktSize, &(recvPkt->data[SIZE_INDEX]), 
//                    sizeof(recvPktSize));
//                */
//                    
//                // Valid packet received
//                
//                /*
//                if (recvPktSize == PKT_SIZE) {
//                    // Get seq num from received ACK packet
//                    memcpy(&ackSeqNum, &(recvPkt->data[SEQ_NUM_INDEX]), 
//                        sizeof(ackSeqNum));
//                    
//                    // Get node ID field value from received packet
//                    memcpy(&recvdPktNodeId, &(recvPkt->data[NODE_ID_INDEX]), 
//                        sizeof(recvdPktNodeId));
//                        
//                    // Get ACK flag from received ACK packet
//                    memcpy(&ackRecvd, &(recvPkt->data[ACK_INDEX]), 
//                        sizeof(ackRecvd));
//                 */
//                    
//                // Check that received packet is the ACK for this seqNum
//                mos_mutex_lock(&recvPktMutex);
//                if ((recvSeqNum == sendSeqNum) && (ackRecvd == 1) && 
//                    (recvdPktNodeId == NODE_ID)) {
//                    // Get current elapsed time, for use in determining RTT for
//                    // received ACK.  Only determine RTT for packets that
//                    // have not been resent
//                    if (numRetransmits == 0) {    
//                        endTime = mos_get_realtime();
//                        sampleRtt = endTime - startTime;
//                    }
//                    
//                    printf("sampleRtt measured = %l ms \n", sampleRtt); 
//                    // Update timeout
//                    // Use Jacobsen/Karels timeout algorithm
//                    sampleRtt -= (estimatedRtt >> 3);
//                    estimatedRtt += sampleRtt;
//                    if (sampleRtt < 0) {
//                        sampleRtt = -sampleRtt;
//                    }
//                    sampleRtt -= (deviation >> 3);
//                    deviation += sampleRtt;
//                    timeOut = (estimatedRtt >> 3) + (deviation >> 1);
//                    
//                    /*
//                    if (timeoutAlpha <= 1.5) {
//                        timeoutAlpha = 1;
//                    }
//                    else {
//                        timeoutAlpha -= 0.5;
//                    }
//                    ackTimeout = (uint32_t)(timeoutAlpha * ackRtt);
//                    */
//                    
//                    printf("For new packet: \n");
//                    if (sampleRtt < 0) {
//                        printf("sampleRtt = -%l ms \n", (uint32_t)(-sampleRtt));
//                    }
//                    else {
//                        printf("sampleRtt = %l ms \n", sampleRtt);
//                    }
//                    printf("deviation = %l ms \n", deviation);
//                    // printf("timeoutAlpa = %l \n", (uint32_t)timeoutAlpha);
//                    printf("timeOut = %l ms \n", timeOut); 
//                    
//                    printf("ACK received for seq num = %l \n", recvSeqNum);
//                    
//                    // Expected ACK successfully received
//                    // Seq num for next packet
//                    sendSeqNum++;
//                    
//                    sendPortNum++;
//                    // Reset port number (for testing)
//                    if (sendPortNum == MAX_PORT_NUM) {
//                        sendPortNum = 0;
//                    }
//                    
//                    // ACK has been processed, so clear ackRecvd flag
//                    ackRecvd = 0;
//                    
//                    com_free_buf(recvPkt);
//                    mos_mutex_unlock(&recvPktMutex);
//                    
//                    // Cancel (remove) ACK timeout alarm
//                    mos_remove_alarm(&ackAlarm);
//                    
//                    numRetransmits = 0;
//                    
//                    break;
//                }
//                else if (packetRecvd == 1) {
//                    com_free_buf(recvPkt);
//                }
//                mos_mutex_unlock(&recvPktMutex);
//                // }
//                
//                // com_free_buf(recvPkt);
//            }
//            
//            printf("ackTimeoutFlag = %d \n", ackTimeoutFlag);
//            
//            // Process ACK timeout (ACK timeout alarm fired)
//            if (ackTimeoutFlag == 1) {
//                // Update timeout
//                // Use Jacobsen/Karels timeout algorithm
//                sampleRtt -= (estimatedRtt >> 3);
//                estimatedRtt += sampleRtt;
//                if (sampleRtt < 0) {
//                    sampleRtt = -sampleRtt;
//                }
//                sampleRtt -= (deviation >> 3);
//                deviation += sampleRtt;
//                timeOut = (estimatedRtt >> 3) + (deviation >> 1);
//                
//                /*
//                ackRtt = ackTimeout;
//                timeoutAlpha += 0.5;
//                ackTimeout = (uint32_t)(timeoutAlpha * ackRtt);
//                */
//                
//                printf("For retransmit: \n");
//                if (sampleRtt < 0) {
//                        printf("sampleRtt = -%l ms \n", (uint32_t)(-sampleRtt));
//                }
//                else {
//                    printf("sampleRtt = %l ms \n", sampleRtt);
//                }
//                printf("deviation = %l ms \n", deviation);
//                // printf("timeoutAlpa = %l \n", (uint32_t)timeoutAlpha);
//                printf("timeOut = %l ms \n", timeOut);
//                
//                if (numRetransmits < MAX_RETRANSMITS) {
//                    numRetransmits++;
//                    
//                    // Clear ackTimeoutFlag, since packet will be resent
//                    ackTimeoutFlag = 0;
//                    
//                    // Cancel (remove) ACK timeout alarm
//                    mos_remove_alarm(&ackAlarm);
//                    
//                    // Resend packet
//                }
//                else {
//                    // MAX_RETRANSMITS attempted, move on to next packet
//                    sendSeqNum++;
//                    numRetransmits = 0;
//                }    
//            }
//                  
//            // mos_mutex_unlock(&recvPktMutex);
//            
//            printf("Continuing loop... \n");                		
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
                // Get data from received packet
                if (skipRecv == 0) {
                    // Skip printing of received data, for debugging
                    memset(recvdDataBuf, 0, sizeof(recvdDataBuf));
                    receiveData(recvdPktPortNum, recvdDataBuf, 
                        sizeof(recvdDataBuf));
                    printf("Received data: %s \n", recvdDataBuf);
                    
                    skipRecv = 1; // Skip printing of next recv
                }
                else {
                    // Print reeceived data on next packet recv
                    skipRecv = 0;
                }
                
                // printf("Sending ACK... \n");
                
		        // uint32_t recvSeqNum = 0;
		        
		        // Get seq num for received packet
                /*
		        memcpy(&recvSeqNum, &(recvPkt->data[SEQ_NUM_INDEX]), 
                    sizeof(recvSeqNum)); */
		        
                // Done with recvPkt, so free it
                com_free_buf(recvPkt);
                
                // Store packet size in ACK packet
                sendPkt.size = ACK_PKT_SIZE;
                memcpy(&(sendPkt.data[SIZE_INDEX]), &(sendPkt.size), 
                    sizeof(sendPkt.size));
                
		        // Store received seq num in ACK packet to send
		        memcpy(&(sendPkt.data[SEQ_NUM_INDEX]), &recvSeqNum, 
                    sizeof(recvSeqNum));
		         
		        // Store node id for next hop in ACK packet to send
		        nextNodeId = LAST_NODE_ID - 1;
		        memcpy(&(sendPkt.data[NODE_ID_INDEX]), &nextNodeId, 
                   sizeof(nextNodeId));
                   
                // Store received port num in ACK packet
                memcpy(&(sendPkt.data[PORT_INDEX]), &(recvdPktPortNum), 
                    sizeof(recvdPktPortNum));
		        
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
	    
	    // mos_thread_sleep(1000);
    }
}

void receiveThread(){
    com_mode(IFACE_RADIO, IF_LISTEN);
    uint8_t recvPktSize;
    // uint32_t recvSeqNum = 0;
   char tempPacketDataStr[PKT_DATA_SIZE + 1];
   
    while (1) {    
        // Receive packet
        // recvPkt = com_recv_timed(IFACE_RADIO, COM_RECV_TIMEOUT);
        // printf("In receiveThread(), calling com_recv()... \n");
        recvPkt = com_recv(IFACE_RADIO);
        
        //printf("In receiveThread(), locking recvPktMutex... \n");
        mos_mutex_lock(&recvPktMutex);
        // printf("In receiveThread(), locked recvPktMutex... \n");
        
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
        if ((recvPktSize <= PKT_SIZE) && (recvdPktNodeId == NODE_ID)) {
            // Get seq num for received packet
            memcpy(&recvSeqNum, &(recvPkt->data[SEQ_NUM_INDEX]), 
                sizeof(recvSeqNum));
                
            // Get port number field value from received packet
            memcpy(&recvdPktPortNum, &(recvPkt->data[PORT_INDEX]), 
                sizeof(recvdPktPortNum));
            
            // Print packet's seq num
            printf("Received seqNum = %l\n", recvSeqNum);
            
            // Print packet's port num
            printf("Received packet for port = %C\n", recvdPktPortNum);
            
            printf("Received packet for recvdPktNodeId = %d \n", 
                recvdPktNodeId); 
            
            // Set packetRecvd flag to indicate that a packet was received
            packetRecvd = 1;       
            
            // Determine if an ACK packet was received
            // Set ackRecvd flag to indicate that an ACK was received    
            memcpy(&ackRecvd, &(recvPkt->data[ACK_INDEX]), sizeof(ackRecvd)); 
            
            if (ackRecvd != 1) {
                // Print packet's data
                memset(tempPacketDataStr, 0, sizeof(tempPacketDataStr));
                memcpy(tempPacketDataStr, &(recvPkt->data[DATA_INDEX]), 
                    recvPktSize - DATA_INDEX);
                tempPacketDataStr[recvPktSize - DATA_INDEX + 1] = '\0';
                printf("In receiveThread(), received packet data = %s \n", 
                    tempPacketDataStr);
            }
            
            // Add received packet data to queue for this port, if not an ACK
            // packet and this node is the last node (target)
            if ((ackRecvd != 1) && (NODE_ID == LAST_NODE_ID)) {
                addToReceiveQueue(&(recvPkt->data[DATA_INDEX]), 
                    recvPktSize - DATA_INDEX, recvdPktPortNum);
            }
            
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

// Gets packet data for a port from the receive queue for that port, up to 
// bufferLen bytes
void receiveData(uint8_t portNum, uint8_t* buffer, int bufferLen) {
    int i;
    uint8_t retVal;
    
    // Get packet data from the receiveQueue for this port
    for (i = 0; i < bufferLen; i++) {
        /*
        if (i >= PKT_DATA_SIZE) {
            break;
        }
        */
        
        retVal = mos_queue_remove(&(receiveQueues[portNum]), &(buffer[i]));
        if (retVal == Q_UNDERFLOW) {
            printf("In receieveData(): queue is empty \n");
            
            // receive queue is empty
            break;
        } 
    }
    
    return;
}

// Send a packet from the base station to it's next hop.  Packet data
// is populated with contents of buffer (up to PKT_DATA_SIZE)
void sendPacket(uint8_t portNum, uint8_t *buffer, uint8_t bufferLen) {
    uint8_t nextNodeId = 0;
    uint8_t dataLen = bufferLen;
    
    uint32_t startTime = 0;
    uint32_t endTime = 0;
    static int32_t sampleRtt = 0;
    static uint32_t estimatedRtt = ACK_TIMEOUT / 2;
    static uint32_t deviation = ACK_TIMEOUT / 4;
    static uint32_t timeOut = ACK_TIMEOUT;
    
    // Used to print contents of buffer, for debugging
    char tempBuffer[bufferLen + 1];
    memset(tempBuffer, 0, sizeof(tempBuffer));
    
    // Store sequence number in packet
    memcpy(&(sendPkt.data[SEQ_NUM_INDEX]), &sendSeqNum, sizeof(sendSeqNum));
    
    // Store node id for next hop in packet
    nextNodeId = BASE_NODE_ID + 1;
    memcpy(&(sendPkt.data[NODE_ID_INDEX]), &nextNodeId, sizeof(nextNodeId));
        
    // Store port number in packet
    memcpy(&(sendPkt.data[PORT_INDEX]), &portNum, sizeof(portNum));
    
    // Store data in packet
    if (bufferLen > PKT_DATA_SIZE) {
        dataLen = PKT_DATA_SIZE;
    }
    memcpy(&(sendPkt.data[DATA_INDEX]), buffer, dataLen);
    
    strncpy(tempBuffer, buffer, dataLen);
    tempBuffer[dataLen] = '\0';
    printf("In sendPacket, dataLen = %d \n", dataLen); 
    printf("Sending packet with data = %s \n", tempBuffer);
    
    // Store packet size in packet
    sendPkt.size = DATA_INDEX + dataLen;
    memcpy(&(sendPkt.data[SIZE_INDEX]), &(sendPkt.size), 
        sizeof(sendPkt.size));
    
    // Get current elapsed time, for use in determining RTT for ACK
    // received for sent packet
    startTime = mos_get_realtime();
    
    // Send packet
    com_send(IFACE_RADIO, &sendPkt);
    
    printf("Sent packet with seq num = %l \n", sendSeqNum);
    mos_led_toggle(0);
    
    // TBD: Add ACK processing
    // Set alarm for ACK timeout
    ackAlarm.msecs = timeOut;
    ackAlarm.reset_to = 0;
    mos_alarm(&ackAlarm); 
    
    while (ackTimeoutFlag == 0) {
        // Check that received packet is the ACK for this seqNum
        mos_mutex_lock(&recvPktMutex);
        if ((recvSeqNum == sendSeqNum) && (ackRecvd == 1) && 
            (recvdPktNodeId == NODE_ID)) {
            // Get current elapsed time, for use in determining RTT for
            // received ACK.  Only determine RTT for packets that
            // have not been resent
            if (numRetransmits == 0) {    
                endTime = mos_get_realtime();
                sampleRtt = endTime - startTime;
            }
            
            printf("sampleRtt measured = %l ms \n", sampleRtt); 
            // Update timeout
            // Use Jacobsen/Karels timeout algorithm
            sampleRtt -= (estimatedRtt >> 3);
            estimatedRtt += sampleRtt;
            if (sampleRtt < 0) {
                sampleRtt = -sampleRtt;
            }
            sampleRtt -= (deviation >> 3);
            deviation += sampleRtt;
            timeOut = (estimatedRtt >> 3) + (deviation >> 1);
            
            printf("For new packet: \n");
            if (sampleRtt < 0) {
                printf("sampleRtt = -%l ms \n", (uint32_t)(-sampleRtt));
            }
            else {
                printf("sampleRtt = %l ms \n", sampleRtt);
            }
            printf("deviation = %l ms \n", deviation);
            // printf("timeoutAlpa = %l \n", (uint32_t)timeoutAlpha);
            printf("timeOut = %l ms \n", timeOut); 
            
            printf("ACK received for seq num = %l \n", recvSeqNum);
            
            // Expected ACK successfully received
            // Seq num for next packet
            sendSeqNum++;
            
            // ACK has been processed, so clear ackRecvd flag
            ackRecvd = 0;
            
            com_free_buf(recvPkt);
            mos_mutex_unlock(&recvPktMutex);
            
            // Cancel (remove) ACK timeout alarm
            mos_remove_alarm(&ackAlarm);
            
            numRetransmits = 0;
            
            break;
        }
        else if (packetRecvd == 1) {
            com_free_buf(recvPkt);
        }
        mos_mutex_unlock(&recvPktMutex);
    }
    
    printf("ackTimeoutFlag = %d \n", ackTimeoutFlag);
            
    // Process ACK timeout (ACK timeout alarm fired)
    if (ackTimeoutFlag == 1) {
        // Update timeout
        // Use Jacobsen/Karels timeout algorithm
        sampleRtt -= (estimatedRtt >> 3);
        estimatedRtt += sampleRtt;
        if (sampleRtt < 0) {
            sampleRtt = -sampleRtt;
        }
        sampleRtt -= (deviation >> 3);
        deviation += sampleRtt;
        timeOut = (estimatedRtt >> 3) + (deviation >> 1);
        
        /*
        ackRtt = ackTimeout;
        timeoutAlpha += 0.5;
        ackTimeout = (uint32_t)(timeoutAlpha * ackRtt);
        */
        
        printf("For retransmit: \n");
        if (sampleRtt < 0) {
                printf("sampleRtt = -%l ms \n", (uint32_t)(-sampleRtt));
        }
        else {
            printf("sampleRtt = %l ms \n", sampleRtt);
        }
        printf("deviation = %l ms \n", deviation);
        // printf("timeoutAlpa = %l \n", (uint32_t)timeoutAlpha);
        printf("timeOut = %l ms \n", timeOut);
        
        if (numRetransmits < MAX_RETRANSMITS) {
            numRetransmits++;
            
            // Clear ackTimeoutFlag, since packet will be resent
            ackTimeoutFlag = 0;
            
            // Cancel (remove) ACK timeout alarm
            mos_remove_alarm(&ackAlarm);
            
            // Resend packet
            sendPacket(portNum, buffer, bufferLen);
        }
        else {
            // MAX_RETRANSMITS attempted, move on to next packet
            sendSeqNum++;
            numRetransmits = 0;
        }    
    }
          
    // mos_mutex_unlock(&recvPktMutex);
}

// Sends data contained in buffer.  Buffer sizes larger than PKT_DATA_SIZE are
// segmented into multiple packets.
void sendData(uint8_t portNum, uint8_t *buffer, uint8_t bufferLen) {
    uint8_t bufferIndex = 0;
    
    if (bufferLen > PKT_DATA_SIZE) {
        // uint8_t remainingBufferLen = bufferLen; 
        
        // Segment buffer into multiple packets
        while (bufferLen > PKT_DATA_SIZE) {
            sendPacket(portNum, &(buffer[bufferIndex]), PKT_DATA_SIZE);
            bufferLen -= PKT_DATA_SIZE;
            bufferIndex += PKT_DATA_SIZE;       
        }
        
        if (bufferLen > 0) {
            // Send last segment of buffer
            sendPacket(portNum, &(buffer[bufferIndex]), bufferLen);
        }    
    }
    else {
        sendPacket(portNum, &(buffer[bufferIndex]), bufferLen);        
    }
}

// Callback function called when an ACK receive has timed out
void ackTimeoutCallBack(void* data) {
    ackTimeoutFlag = 1;
    
    mos_led_toggle(2);
}

// Adds packet data to receive queue for a port
void addToReceiveQueue(uint8_t *packetData, int packetDataLen, int port) {
    int i;
    int retVal;
    
    for (i = 0; i < packetDataLen; i++) {
        retVal = mos_queue_add(&(receiveQueues[port]), packetData[i]);
        
        if (retVal == Q_FULL) {
            printf("In addToReceiveQueue(): queue full \n");
            
            // receive queue is full
            break;    
        }
    }
}
   
/* The start function is automatically called by the operating system,
   for single-threaded applications, treat start() like main(). for multi-
   threaded applications, start each thread in the start() function.*/
void start(void) {
    int i;
    
    mos_mutex_init(&recvPktMutex);
    mos_mutex_init(&sendPktMutex);
    
    // Initialize ACK timeout alarm
    ackAlarm.func = ackTimeoutCallBack;
    ackAlarm.msecs = ACK_TIMEOUT;
    ackAlarm.reset_to = 0;
    
    // Initialize packet receive queues
    for (i = 0; i < MAX_PORT_NUM + 1; i++) {
        mos_queue_init(&(receiveQueues[i]), receiveBufs[i], 
            (uint8_t)(PKT_DATA_SIZE * MAX_QUEUE_LEN));           
    }
    
    mos_thread_new(receiveThread, 128, PRIORITY_HIGH);
    mos_thread_new(sendThread, 128, PRIORITY_NORMAL);
}
