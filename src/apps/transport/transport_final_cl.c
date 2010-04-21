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
// - Added support for concurrent logical connections.  Only one connection
//   per node id and port number combination is permitted.
// Each connection is uniquely identified by the combination of source node id,
// source port number, destination node id, and destination port number.

#include <inttypes.h>

#include <stdlib.h>
#include <string.h>
#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "mutex.h"
#include "sem.h"
// #include "string.h"
#include "com.h"
#include "printf.h"
#include "clock.h"
#include "queue.h"
#include "transport.h"
#include "net.h"
#include "ctp_plus.h"

// Flag used to indicate when a packet is received by the receive thread
// uint8_t packetRecvd = 0; 

// Flag used to indicate when an ACK packet is received by the receive thread
uint8_t ackRecvd = 0;

uint32_t sendSeqNum = 0;
uint32_t recvSeqNum = 0;
comBuf sendPkt;
comBuf* recvPkt;

// Array of packet receive queues, one for each port
queue_t receiveQueues[MAX_PORT_NUM + 1];

// Array of packet data receive queues, one for each port
// queue_t receiveDataQueues[MAX_PORT_NUM + 1];

// Array of packet header receive queues, one for each port
// queue_t receiveHeaderQueues[MAX_PORT_NUM + 1];

// Array of buffers, one for each receive data queue
uint8_t receiveBufs[MAX_PORT_NUM + 1][PKT_DATA_SIZE * MAX_QUEUE_LEN];

// Values from received packet header
// uint8_t recvdPktNodeId = 0;
uint8_t recvdPktSrcPortNum = 0;
uint8_t recvdPktDestPortNum = 0;
uint8_t recvdConnId = 0;

// Mutexes for access to sendPkt, recvPkt, connections[], and sendConnReplyFlag
mos_mutex_t sendPktMutex;
mos_mutex_t recvPktMutex;
mos_mutex_t connectionsMutex;

// Semaphore for ACK processing
// mos_sem_t ackSem;

mos_mutex_t sendConnReplyFlagMutex;

// Mutexes for access to each receiveQueue
mos_mutex_t receiveQueueMutexes[MAX_PORT_NUM + 1];

// Alarm for ACK timeout
mos_alarm_t ackAlarm;
// uint8_t ackTimeoutFlag = 0;

// Flag set by receiveThread() to indicate that a connection management reply
// message should be sent by sendThread()
uint8_t sendConnReplyFlag = 0;

// Buffer for message to send in connection management reply
char connReplyMessBuff[PKT_DATA_SIZE];

// Array of connection state information for each logical connection, used for
// sending side of the connection
connectState connections[MAX_PORT_NUM + 1];

// Array of server connection state information for each logical connection,
// used for the receiving side of the connection
connectState serverConnections[MAX_PORT_NUM + 1];

// Array of connection ids, used as data parameter values for 
// ackTimeoutCallBack() function
/* uint8_t connectionIds[MAX_PORT_NUM + 1] = 
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}; */
uint8_t connectionIds[MAX_PORT_NUM + 1] = {0, 1, 2, 3};

// Connection ID for which connection reply message should be send
uint8_t connReplyConnId = 0;

// Queue to hold connections IDs for which connection reply messages should be
// sent
queue_t connReplyQueue;

uint8_t connReplyQueueBuf[MAX_PORT_NUM + 1];

// Counters to record number of packets sent and received
uint32_t numPktsSent = 0;
uint32_t numPktsRecvd = 0;

// Thread to send a reply to a connection message
void sendConnReplyThread() {
	uint8_t i = 0;
	uint8_t currConnId = 0;
	
	mos_mutex_lock(&sendConnReplyFlagMutex);
	// Send connection reply messages for any pending connection replies
	for (i = 0; i < MAX_PORT_NUM + 1; i++) {
		currConnId = connectionIds[i];
		
	    if (connections[currConnId].sendConnReplyFlag == 1) {
	    	// Send connection reply message to originator (sender of connection 
	    	// message)
	    	sendPacket(connections[currConnId].srcPortNum, 
	    			connections[currConnId].connReplyMessBuff, 
	    			strlen(connections[currConnId].connReplyMessBuff) + 1, 
	    			connections[currConnId].originator);
	        
	    	if (strcmp(connections[currConnId].connReplyMessBuff, 
	    			"ACCEPTED") == 0) {
	    		// Set connection state to open if an ACCEPTED message was sent
	    		// (opening a connection)
	    		serverConnections[connections[currConnId].
	    		                  destPortNum].connectionOpen = 1;
	    	}
	    	else if (strcmp(connections[currConnId].connReplyMessBuff, 
	    			"CLOSED") == 0) {
	    		// Set connection state to open if CLOSED message was sent
	    		// (closed a connection)
	    		serverConnections[connections[currConnId].
	    		                  destPortNum].connectionOpen = 0;
	    	}
	    	
	        // Connection management reply message sent, so clear 
	        // sendConnReplyFlag for connection
	    	connections[currConnId].sendConnReplyFlag = 0;  
	    }
	}
    mos_mutex_unlock(&sendConnReplyFlagMutex);
    // mos_mutex_unlock(&recvPktMutex);
}

// Receive a packet on the specified port.  Received packets are placed on the
// receive queue for the port.
void receivePacket(uint8_t portNum) {
    uint8_t recvPktSize;
    // uint32_t recvSeqNum = 0;
    char tempPacketDataStr[PKT_DATA_SIZE + 1];
    char pktDataStr[PKT_DATA_SIZE + 1];
    
    uint16_t originator = 0;
    uint8_t  headerSize;
    
    // char messageBuff[PKT_DATA_SIZE];
    // memset(connReplyMessBuff, 0, sizeof(connReplyMessBuff));
    
    // uint32_t lastSeqNumRecvd = 0;
    uint8_t connectMessRecvd = 0;
    
#ifdef TRANSPORT_DEBUG   
    printf("In receivePacket(), calling net_recv()... \n");
#endif    
    
    // Get received data packet from net layer (CTP++)
    recvPkt = net_recv(TRANSPORT_LISTENING_PORT);

#ifdef TRANSPORT_DEBUG
    printf ("In receivePacket(), locking recvPktMutex... \n");
#endif
    mos_mutex_lock(&recvPktMutex);
#ifdef TRANSPORT_DEBUG
    printf("In receivePacket(), locked recvPktMutex... \n");
#endif
    
    /*
    if (recvPkt == NULL) {
        mos_mutex_unlock(&recvPktMutex);
        mos_thread_sleep(100);
        continue;
    }
    */

#ifdef TRANSPORT_DEBUG || defined(TRANSPORT_RECV_RATIO)
    printf("Received data \n");
    
    numPktsRecvd++;
    printf("Num packets recvd = %l \n", numPktsRecvd);
#endif
    
    // Get orignator (sender) of this packet
    // Extract originator from CTP++ header
    ctp_data_header_t* dh = (ctp_data_header_t*) recvPkt->data;
    originator = (dh->originHigh << 8) | (dh->originLow & 0xff);
    
    // Remove CTP++ header from packet
    headerSize = getHeaderSize(dh->type);
    memcpy(recvPkt->data, &(recvPkt->data[headerSize]), 
    		recvPkt->size-headerSize);
    recvPkt->size -= headerSize;
    
    // Get packet size from packet data
    memcpy(&recvPktSize, &(recvPkt->data[SIZE_INDEX]), 
        sizeof(recvPktSize));
        
    // Get node ID field value from received packet
    /*
    memcpy(&recvdPktNodeId, &(recvPkt->data[NODE_ID_INDEX]), 
        sizeof(recvdPktNodeId));
    */
        
    // Valid packet for this node received
    // if ((recvPktSize <= PKT_SIZE) && (recvdPktNodeId == NODE_ID)) {
    if (recvPktSize <= PKT_SIZE) {
        // Get seq num for received packet
        memcpy(&recvSeqNum, &(recvPkt->data[SEQ_NUM_INDEX]), 
            sizeof(recvSeqNum));
        
        // Get dest port number field value from received packet
        memcpy(&recvdPktDestPortNum, &(recvPkt->data[DEST_PORT_INDEX]), 
            sizeof(recvdPktDestPortNum));    
        
        // Get src port number field value from received packet
        memcpy(&recvdPktSrcPortNum, &(recvPkt->data[SRC_PORT_INDEX]), 
            sizeof(recvdPktSrcPortNum));
        
        // Get connection id field value from received packet
        memcpy(&recvdConnId, &(recvPkt->data[CONN_ID_INDEX]), 
            sizeof(recvdConnId));
        
        connections[recvdConnId].originator = originator;
        connections[recvdConnId].recvSeqNum = recvSeqNum;
        connections[recvdConnId].srcPortNum = recvdPktSrcPortNum;
        connections[recvdConnId].destPortNum = recvdPktDestPortNum;
        
#ifdef TRANSPORT_DEBUG
        printf("Received packet from originator = %d \n", originator);
        
        printf("Received connId = %d \n", recvdConnId);
        
        printf("Received packet size = %d \n", recvPktSize);
        
        // Print packet's seq num
        printf("Received seqNum = %l\n", recvSeqNum);
        
        // Print packet's port num
        printf("Received packet for port = %C\n", recvdPktDestPortNum);
#endif
        
        /* printf("Received packet for recvdPktNodeId = %d \n", 
            recvdPktNodeId); */ 
        
        // Set packetRecvd flags to indicate that a packet was received
        // packetRecvd = 1;
        // connections[recvdConnId].packetRecvd = 1;
        
        // Determine if an ACK packet was received
        // Set ackRecvd flag to indicate that an ACK was received for this
        // connection    
        // memcpy(&ackRecvd, &(recvPkt->data[ACK_INDEX]), sizeof(ackRecvd)); 
        memcpy(&(connections[recvdConnId].ackRecvd), 
            &(recvPkt->data[ACK_INDEX]), 
            sizeof(connections[recvdConnId].ackRecvd));
        memcpy(&ackRecvd, &(recvPkt->data[ACK_INDEX]), sizeof(ackRecvd));

#ifdef TRANSPORT_DEBUG
        if (ackRecvd == 1) {
            printf("Received ACK packet \n");
        }
        
        if (connections[recvdConnId].ackRecvd != 1) {
            // Print packet's data
            memset(tempPacketDataStr, 0, sizeof(tempPacketDataStr));
            memcpy(tempPacketDataStr, &(recvPkt->data[DATA_INDEX]), 
                recvPktSize - DATA_INDEX);
            tempPacketDataStr[recvPktSize - DATA_INDEX + 1] = '\0';
            printf("In receivePacket(), received packet data = %s \n", 
                tempPacketDataStr);
        }
#endif
        
        // Process connection management request messages: CONNECT or
        // CLOSE
        memset(pktDataStr, 0, sizeof(pktDataStr));
        memcpy(pktDataStr, &(recvPkt->data[DATA_INDEX]), 
            recvPktSize - DATA_INDEX);
        pktDataStr[recvPktSize - DATA_INDEX + 1] = '\0';
        
        // Process CONNECT message
        // TBD: Spawn a new thread to use for sending connection reply messages
        // (needed to handle ACK timeouts)
        if (strcmp(pktDataStr, "CONNECT") == 0) {
        	
#ifdef TRANSPORT_DEBUG
            printf("Processing CONNECT message \n");
            
            // Send ACK for received CONNECT message
            printf("Sending ACK to node %d \n", originator);
#endif
            
            sendAck(recvdPktSrcPortNum, originator);
            
            connectMessRecvd = 1;
            
            // Check serverConnections to determine if a connection
            // is already open on this port
            if (serverConnections[recvdPktDestPortNum].connectionOpen == 1) {
                // Send BUSY message to indicate that a connection
                // is already open
#ifdef TRANSPORT_DEBUG
            	printf("Sending BUSY message ... \n");
#endif
            	
            	// Can only send a connection reply message if there is no
            	// pending connection reply message to send for this connection
            	if (connections[recvdConnId].sendConnReplyFlag == 0) {
	                mos_mutex_lock(&sendConnReplyFlagMutex);
	                memset(connections[recvdConnId].connReplyMessBuff, 0, 
	                		sizeof(connections[recvdConnId].connReplyMessBuff));
	                char connectMessage[] = "BUSY";
	                strcpy(connections[recvdConnId].connReplyMessBuff, 
	                		connectMessage);
	                // connReplyConnId = recvdConnId;
	                connections[recvdConnId].sendConnReplyFlag = 1;
	                // sendConnReplyFlag = 1;
	                mos_mutex_unlock(&sendConnReplyFlagMutex);
            	}
                
                // Unlock recvPktMutex to allow ACK to be received
                // mos_mutex_unlock(&recvPktMutex);
                /* sendPacket(recvdPktSrcPortNum, connReplyMessBuff, 
                		strlen(connReplyMessBuff) + 1, originator); */
                /*
                sendPacket(recvdPktSrcPortNum, messageBuff, 
                    strlen(messageBuff) + 1);
                    */
                
            }
            else {
                // Send ACCEPTED message
#ifdef TRANSPORT_DEBUG
                printf("Sending ACCEPTED message ... \n");
#endif
                
                // Can only send a connection reply message if there is no
            	// pending connection reply message to send
            	if (connections[recvdConnId].sendConnReplyFlag == 0) {
	                mos_mutex_lock(&sendConnReplyFlagMutex);
	                memset(connections[recvdConnId].connReplyMessBuff, 0, 
	                		sizeof(connections[recvdConnId].connReplyMessBuff));
	                char connectMessage[] = "ACCEPTED";
	                strcpy(connections[recvdConnId].connReplyMessBuff, 
	                		connectMessage);
	                // connReplyConnId = recvdConnId;
	                connections[recvdConnId].sendConnReplyFlag = 1;
	                // sendConnReplyFlag = 1;
	                mos_mutex_unlock(&sendConnReplyFlagMutex);
	                
#ifdef TRANSPORT_DEBUG
	                printf("Scheduled ACCEPTED message \n");
#endif
            	}
                
                // Unlock recvPktMutex to allow ACK to be received
                /*
                mos_mutex_unlock(&recvPktMutex);
                mos_thread_sleep(1000);
                sendPacket(recvdPktSrcPortNum, connReplyMessBuff, 
                		strlen(connReplyMessBuff) + 1, originator);
                */
                /*
                sendPacket(recvdPktSrcPortNum, messageBuff, 
                    strlen(messageBuff) + 1);
                    */
            }
            
        }
        // Process CLOSE message
        else if (strcmp(pktDataStr, "CLOSE") == 0) {
#ifdef TRANSPORT_DEBUG
            printf("Processing CLOSE message \n");
#endif
            
            // Send ACK for received CLOSE message
            sendAck(recvdPktSrcPortNum, originator);
            
            connectMessRecvd = 1;
            
            // Send CLOSED message
            // Can only send a connection reply message if there is no
        	// pending connection reply message to send
        	if (connections[recvdConnId].sendConnReplyFlag == 0) {
	            mos_mutex_lock(&sendConnReplyFlagMutex);
                memset(connections[recvdConnId].connReplyMessBuff, 0, 
                		sizeof(connections[recvdConnId].connReplyMessBuff));
	            char closeMessage[] = "CLOSED";
	            strcpy(connections[recvdConnId].connReplyMessBuff, 
	            		closeMessage);
	            connections[recvdConnId].sendConnReplyFlag = 1;
	            // sendConnReplyFlag = 1;
	            // connReplyConnId = recvdConnId;
	            mos_mutex_unlock(&sendConnReplyFlagMutex);
        	}
            
            // Unlock recvPktMutex to allow ACK to be received
            // mos_mutex_unlock(&recvPktMutex);
            /*
            sendPacket(recvdPktSrcPortNum, connReplyMessBuff, 
            		strlen(connReplyMessBuff) + 1, originator);
            */
            /*
            sendPacket(recvdPktSrcPortNum, messageBuff, 
                strlen(messageBuff) + 1);
                */
        }
        
        // TBD: Add received packet (header + data) to queue for this port,
        // if not an ACK packet and not a resent packet
        // printf("In receiveThread(), recvSeqNum = %d \n", recvSeqNum);
        // printf("In receiveThread(), lastSeqNumRecvd = %d \n", lastSeqNumRecvd);
        if ((connections[recvdConnId].ackRecvd != 1) && 
            (connectMessRecvd != 1) && (recvSeqNum > 
            serverConnections[recvdPktDestPortNum].currSeqNum)) {
            serverConnections[recvdPktDestPortNum].currSeqNum = recvSeqNum;    

#ifdef TRANSPORT_DEBUG
            printf("In receievePacket(), adding packet to receive queue \n");
#endif
            
            addToReceiveQueue(&(recvPkt->data[0]), recvPktSize, 
            		recvdPktDestPortNum);
            
        }
        if ((connections[recvdConnId].ackRecvd != 1) && 
        		(connectMessRecvd != 1)) {
#ifdef TRANSPORT_DEBUG
        	printf("In receivePacket, sending ACK to originator ... \n");
#endif
        	
			// Send ACK to originator of received packet
			sendAck(recvdPktSrcPortNum, connections[recvdConnId].originator);
		}
        
        // Add received packet data to queue for this port, if not an ACK
        // packet, not a resent packet, and this node is the last node 
        // (target)
        // TBD: Add source node id and source port num "header" to 
        // packet header queue
        /*
        if ((ackRecvd != 1) && (recvSeqNum > lastSeqNumRecvd) && 
            (NODE_ID == LAST_NODE_ID)) {
            addToReceiveQueue(&(recvPkt->data[DATA_INDEX]), 
                recvPktSize - DATA_INDEX, recvdPktDestPortNum);
        }
        */
        
        /*
        if (ackRecvd == 0) {
            // Non-ACK packet received, so update sequence num of last
            // packet received
            lastSeqNumRecvd = recvSeqNum;
        }
        */

#ifdef TRANSPORT_DEBUG
        mos_led_toggle(1);
#endif
        // mos_thread_sleep(500);
    }
        
    // Free packet (contents have been stored in receiveQueue)
    com_free_buf(recvPkt);    
    
    mos_mutex_unlock(&recvPktMutex);
    
    if (connections[recvdConnId].sendConnReplyFlag == 1) {
    	// Spawn new thread to send connection message reply
    	mos_thread_new(sendConnReplyThread, 384, PRIORITY_HIGH);
    	
    	// Don't unlock recvPktMutex until connection reply message is sent,
    	// to prevent disruption of connection state
    }
    
    // mos_thread_sleep(100);
    // Don't unlock recvPktMutex if an ACK is received - allows ACK to be
    // processed
    /*
    if (ackRecvd != 1) {
    	mos_mutex_unlock(&recvPktMutex);
    }
    
    if (ackRecvd == 1) {
    	mos_sem_post(&ackSem);
    }
    */
#ifdef TRANSPORT_DEBUG
        printf("Completed receivePacket() \n");
#endif
}

// Gets packet data for a port from the receive queue for that port, up to 
// bufferLen bytes
int receiveData(uint8_t portNum, uint8_t* buffer, uint8_t bufferLen, 
    uint8_t blocking) {
    int i;
    uint8_t retVal;
    uint8_t packetNum;
    uint8_t tempByte = 0;
    uint8_t packetSize = 0;
    
    uint8_t packetBuf[PKT_SIZE];
    
    // Populate buffer with packet data from receiveQueue
    for (packetNum = 0; (packetNum * PKT_DATA_SIZE) < bufferLen; packetNum++) {
        memset(packetBuf, 0, sizeof(packetBuf));
        
        // Block until receieve queue has data
        if (blocking == 1) {
            while (1) {
            	// receivePacket(portNum);
            	
                retVal = mos_queue_peek(&(receiveQueues[portNum]), 0, 
                    &tempByte);
                
                if (retVal == Q_EMPTY) {
                    // Receive queue is empty, so continue looking for data
                    continue;
                }
                else if (retVal == Q_OK) {
                    // Data available in receive queue
                    break;
                } 
            }
        }
        
        // Check if receieve queue is empty before attempting to get packet 
        // data from receive queue
        retVal = mos_queue_peek(&(receiveQueues[portNum]), 0, &tempByte);          
        if (retVal == Q_EMPTY) {
#ifdef TRANSPORT_DEBUG
        	printf("In receiveData(): queue is empty \n");
#endif
                
            // Receive queue is empty
        	return RECV_Q_UNDERFLOW;
        }
        
        mos_mutex_lock(&(receiveQueueMutexes[portNum]));
        // Get actual size of packet
        mos_queue_peek(&(receiveQueues[portNum]), SIZE_INDEX, &packetSize);
        
        // Get packet from receiveQueue
        for (i = 0; i < packetSize; i++) {
            retVal = mos_queue_remove(&(receiveQueues[portNum]), &(packetBuf[i]));
            if (retVal == Q_UNDERFLOW) {
#ifdef TRANSPORT_DEBUG
                printf("In receiveData(): queue is empty (underflow) \n");
#endif
                
                // Receive queue is empty
                mos_mutex_unlock(&(receiveQueueMutexes[portNum]));
                return RECV_Q_UNDERFLOW;
            }    
        }
        mos_mutex_unlock(&(receiveQueueMutexes[portNum]));
        
        // Extract data portion of packet
        memcpy(&(buffer[packetNum * PKT_DATA_SIZE]), &(packetBuf[DATA_INDEX]), 
            (packetSize - DATA_INDEX));
    }
    
    // Packet data successfully removed from receive queue
    return RECV_Q_OK;
}

// Send a packet from the base station to it's next hop.  Packet data
// is populated with contents of buffer (up to PKT_DATA_SIZE)
void sendPacket(uint8_t portNum, uint8_t *buffer, uint8_t bufferLen, 
		uint16_t destNodeId) {
    uint8_t dataLen = bufferLen;
    uint32_t seqNum = 0;
    
    uint32_t startTime = 0;
    uint32_t endTime = 0;
    
    static int32_t sampleRtt = 0;
    static int32_t estimatedRtt = ACK_TIMEOUT / 2;
    static uint32_t deviation = ACK_TIMEOUT / 4;
    static uint32_t timeOut = ACK_TIMEOUT;
    
    uint16_t destETX = 0;
    static uint16_t numHops = 0;
    uint16_t prevNumHops = 0;
    uint16_t numHopsRem = 0;
    
    uint8_t connectionInUse;
    uint8_t connectionFound;
    
    // Connection id for lowest idle connection, which will be used to send
    // the packet
    uint8_t sendConnId = 0;
     
    // uint8_t currConn = 0;
    
#ifdef TRANSPORT_DEBUG
    // Used to print contents of buffer, for debugging
    char tempBuffer[bufferLen + 1];
    memset(tempBuffer, 0, sizeof(tempBuffer));
#endif
    
    retransmit:
    connectionInUse = 0;
    connectionFound = 0;
     
    // Store port number in packet
    memcpy(&(sendPkt.data[DEST_PORT_INDEX]), &portNum, sizeof(portNum));
    memcpy(&(sendPkt.data[SRC_PORT_INDEX]), &portNum, sizeof(portNum));
    
    // Store data in packet
    if (bufferLen > PKT_DATA_SIZE) {
        dataLen = PKT_DATA_SIZE;
    }
    memcpy(&(sendPkt.data[DATA_INDEX]), buffer, dataLen);
    
#ifdef TRANSPORT_DEBUG
    strncpy(tempBuffer, buffer, dataLen);
    tempBuffer[dataLen] = '\0';
    printf("In sendPacket, dataLen = %d \n", dataLen); 
    printf("Sending packet with data = %s \n", tempBuffer);
    printf("sending packet to node %d \n", destNodeId);
    printf("sending packet to port %d \n", portNum);
#endif
    
    // Store packet size in packet
    sendPkt.size = DATA_INDEX + dataLen;
    memcpy(&(sendPkt.data[SIZE_INDEX]), &(sendPkt.size), 
        sizeof(sendPkt.size));
        
    // Find connection to use for send
    /*
    while (1) {
#ifdef TRANSPORT_DEBUG
        printf("Looking for connection to use for send... \n");
#endif
        
        mos_mutex_lock(&connectionsMutex);
        for (sendConnId = 0; sendConnId < MAX_CONNECTIONS; sendConnId++) {
            if ((connections[sendConnId].connectionBusy == 1) && 
                (connections[sendConnId].destPortNum == portNum)) {
                // Found a connection for this port number already in use
                connectionInUse = 1;
                mos_mutex_unlock(&connectionsMutex);
                break;
            }
        }
        if ((connectionInUse == 1) && 
            (connections[sendConnId].resendFlag == 1)) {
            // Resend requested
            mos_mutex_unlock(&connectionsMutex);
            break;
        }
        else if (connectionInUse == 1) {
            // Wait for connection for this port number (and node id) to be 
            // free
            mos_mutex_unlock(&connectionsMutex);
            continue;
        }
        else if (connectionInUse == 0) {
            // Find lowest idle connection to use for send
            for (sendConnId = 0; sendConnId < MAX_CONNECTIONS; sendConnId++) {
                if ((connections[sendConnId].connectionBusy == 0)) {
                    connectionFound = 1;
                    mos_mutex_unlock(&connectionsMutex);
                    break;
                }
            }
            
            mos_mutex_unlock(&connectionsMutex);
            // Idle connection found
            if (connectionFound == 1) {
                break;
            }
            
            else {
                // Idle connection not found, so continue to wait for a free
                // connection
                continue;
            }
        }  
    }
    */
    sendConnId = portNum;
    
#ifdef TRANSPORT_DEBUG
    printf("Using connection %d for send \n", sendConnId);
#endif
    
    mos_mutex_lock(&connectionsMutex);
    if (connections[sendConnId].resendFlag == 1) {
        // Resend packet using same sequence number as previous send
        seqNum = connections[sendConnId].currSeqNum;
        
        // Resend request processed, so clear resendFlag
        connections[sendConnId].resendFlag = 0;
    }
    else {
        // Seq num for next packet that will be sent
        // sendSeqNum++;
        
        // Use next sequence number for send
        seqNum = (connections[sendConnId].currSeqNum) + 1;
      
        sendSeqNum = seqNum;
    }
    
    // Store sequence number in packet
    memcpy(&(sendPkt.data[SEQ_NUM_INDEX]), &seqNum, sizeof(seqNum));
    
    // Store connection id in packet
    memcpy(&(sendPkt.data[CONN_ID_INDEX]), &sendConnId, 
        sizeof(sendConnId));
        
    // Clear ACK flag in packet
    memset(&(sendPkt.data[ACK_INDEX]), 0, sizeof(uint8_t));
    
#ifdef CROSS_LAYER
    // Get ETX value for destination
    // Use appropriate packet type depending on whether destination is
    // upstream or downstream
    if (is_base == true) {
    	destETX = getETX(AM_CTP_DL_DATA, destNodeId);
    }
    else {
    	destETX = getETX(AM_CTP_DATA, destNodeId);
    }
    
    // Caculate approx number of hops based on max ETX value per hop
    prevNumHops = numHops;
    numHops = destETX / MAX_ETX_PER_HOP;
    numHopsRem = destETX % MAX_ETX_PER_HOP;
    if (numHopsRem > 0) {
    	numHops++;
    }
    
    if (numHops >= MAX_ETX / MAX_ETX_PER_HOP) {
    	// Disregard very high hop counts, since they probably indicate loss
    	// of connectivity
    	numHops = prevNumHops;
    }
    
 
#ifdef TRANSPORT_DEBUG
    printf("destETX = %d \n", destETX);
    printf("approx num of hops = %d \n", numHops);
#endif
    
    // Don't recalculate timeout if destETX exceeds MAX_ETX, since this 
    // indicates loss of connectivity to the destination
    if (destETX <= MAX_ETX) {
	    // Recalculate timeout if there is a significant change in hop count to
    	// the destination, where signficant is defined by HOP_COUNT_DELTA.
	    // Also calculate timeout based on hop count for the initial send 
	    // (prevNumHops = 0)
	    if ((abs(numHops - prevNumHops) >= HOP_COUNT_DELTA) || 
	    		(prevNumHops == 0)) {
	    	if (prevNumHops == 0) {
	    		// Add extra timeout for initial send
	    		timeOut = (numHops * TIMEOUT_PER_HOP) + EXTRA_INITIAL_TIMEOUT;
	    	}
	    	else {
	    		timeOut = numHops * TIMEOUT_PER_HOP;
	    	}
	    	
	    	// Fold in recalculated timeOut into Jacobsen-Karels timeout 
	    	// algorithm
	    	estimatedRtt = timeOut;
	    	
#ifdef TRANSPORT_DEBUG
	    	printf("Recalculated timeout based on hopcount = %d \n", timeOut);
#endif
	    }
    }
#endif
    
    // Get current elapsed time, for use in determining RTT for ACK
    // received for sent packet
    startTime = mos_get_realtime();
    
    // Send packet
    // com_send(IFACE_RADIO, &sendPkt);
    // Send packet using CTP++
    if (is_base == true) {
    	// Send downstream packet (base to node)
#ifdef TRANSPORT_DEBUG
    	printf("Sending packet from base to node... \n");
#endif
    	net_send(&sendPkt, CTP_PROTO_ID, TRANSPORT_LISTENING_PORT, 
    			CTP_NEW_PACKET, AM_CTP_DL_DATA, destNodeId, 
    			TRANSPORT_LISTENING_PORT);
    	
#ifdef TRANSPORT_DEBUG || defined(TRANSPORT_RECV_RATIO)
    	numPktsSent++;
    	printf("Num packets sent = %l \n", numPktsSent);
#endif
    }
    else {
        // Send upstream packet (node to base)
#ifdef TRANSPORT_DEBUG
    	printf("Sending packet from node to base... \n");
#endif
    	net_send(&sendPkt, CTP_PROTO_ID, TRANSPORT_LISTENING_PORT, 
    			CTP_NEW_PACKET, AM_CTP_DATA, TRANSPORT_LISTENING_PORT);
    	
#ifdef TRANSPORT_DEBUG || defined(TRANSPORT_RECV_RATIO)
    	numPktsSent++;
    	printf("Num packets sent = %l \n", numPktsSent);
#endif
    }
    
#ifdef TRANSPORT_DEBUG    
    printf("Sent packet with seq num = %l \n", seqNum);
    mos_led_toggle(0);
#endif
    
    // Set status for this connection to busy and update parameters for this
    // connection
    connections[sendConnId].connectionBusy = 1;
    connections[sendConnId].currSeqNum = seqNum;
    connections[sendConnId].destPortNum = portNum;
    
    // Set alarm for ACK timeout
    connections[sendConnId].ackAlarm.msecs = timeOut;
    connections[sendConnId].ackAlarm.reset_to = 0;
    connections[sendConnId].ackAlarm.data = &(connectionIds[sendConnId]);
    mos_alarm(&(connections[sendConnId].ackAlarm));
    
    mos_mutex_unlock(&connectionsMutex);
    
    // TBD: Need to prevent connections[sendConnId].ackRecvd from getting cleared
    // mos_sem_wait(&ackSem);
    /*
    if (is_base == true) {
    	printf("Waiting for ACK... \n");
    	receivePacket(portNum);
    	printf("In sendPacket, received packet (ACK) \n");
    }
    */
    
    // Launch new thread to wait for ACK
    // mos_thread_new(recvAckThread, 384, PRIORITY_NORMAL);
    
    // Wait for ACK to be received
    // receivePacket(portNum);
    
    /*
    ackAlarm.msecs = timeOut;
    ackAlarm.reset_to = 0;
    mos_alarm(&ackAlarm);
    */ 
    
    // while (ackTimeoutFlag == 0) {
    // Process ACK for the sent packet, or process timeout if ACK is not 
    // received in time
    while (connections[sendConnId].ackTimeoutFlag == 0) {
        // Check that received packet is the ACK for the sent seqNum
        // printf ("In sendPacket(), locking recvPktMutex... \n");
        // mos_mutex_lock(&recvPktMutex);
        // printf("In sendPacket(), locked recvPktMutex... \n");
    	
    	/*
    	printf("In ACK processing, recvSeqNum = %d \n", recvSeqNum);
    	printf("In ACK processing, connections[sendConnId].currSeqNum = %d \n", 
    			connections[sendConnId].currSeqNum);
    	printf("In ACK processing, connections[sendConnId].ackRecvd = %d \n", 
    			connections[sendConnId].ackRecvd);
    	printf("In ACK processing, recvdConnId = %d \n", recvdConnId);
    	printf("In ACK processing, sendConnId = %d \n", recvdConnId);
    	*/
    	
        if ((connections[sendConnId].recvSeqNum == 
        	connections[sendConnId].currSeqNum) && 
            (connections[sendConnId].ackRecvd == 1)) {
        /* if ((recvSeqNum == sendSeqNum) && (ackRecvd == 1) && 
            (recvdPktNodeId == NODE_ID)) { */
            // Get current elapsed time, for use in determining RTT for
            // received ACK.  Only determine RTT for packets that
            // have not been resent
            if (connections[sendConnId].numRetransmits == 0) {    
                endTime = mos_get_realtime();
                sampleRtt = endTime - startTime; 
            
#ifdef TRANSPORT_DEBUG
	            // printf("Calc sampleRtt, startTime = %l ms \n", startTime);
	            // printf("Calc sampleRtt, endTime = %l ms \n", endTime);
	            printf("sampleRtt measured = %l ms \n", sampleRtt);
#endif
            
	            // Update timeout
	            // Use Jacobsen/Karels timeout algorithm
	            sampleRtt -= (estimatedRtt >> 3);
	            estimatedRtt += sampleRtt;
	            if (sampleRtt < 0) {
	                sampleRtt = -sampleRtt;
	            }
	            sampleRtt -= (deviation >> 3);
	            deviation += sampleRtt;
#ifdef FIXED_TIMEOUT
	            timeOut = SEND_RATE;
#else
	            timeOut = (estimatedRtt >> 3) + (deviation >> 1);
#endif	            
#ifdef TRANSPORT_DEBUG
	            printf("For new packet: \n");
	            if (sampleRtt < 0) {
	                printf("sampleRtt = -%l ms \n", (uint32_t)(-sampleRtt));
	            }
	            else {
	                printf("sampleRtt = %l ms \n", sampleRtt);
	            }
	            printf("deviation = %l ms \n", deviation);
	            // printf("timeoutAlpa = %l \n", (uint32_t)timeoutAlpha);
#endif
            }

#ifdef TRANSPORT_DEBUG
            printf("timeOut = %l ms \n", timeOut);
#endif
            
            // printf("ACK received for seq num = %l \n", recvSeqNum);
            
            // Expected ACK successfully received
            // Seq num for next packet
            // sendSeqNum++;
            
            // ACK has been processed, so clear ackRecvd and packetRecvd flags
            connections[sendConnId].ackRecvd = 0;
            // packetRecvd = 0;
            // connections[recvdConnId].packetRecvd = 0;
            
            // mos_mutex_unlock(&recvPktMutex);
            
            // Cancel (remove) ACK timeout alarm for this connection
            mos_remove_alarm(&(connections[sendConnId].ackAlarm));
            
            // Free connection
            connections[sendConnId].connectionBusy = 0;
            
            connections[sendConnId].numRetransmits = 0;
            
            break;
        }
        
        // mos_mutex_unlock(&recvPktMutex);
    }
    
#ifdef TRANSPORT_DEBUG
    printf("ackTimeoutFlag = %d \n", connections[sendConnId].ackTimeoutFlag);
#endif
            
    // Process ACK timeout (ACK timeout alarm fired)
    if (connections[sendConnId].ackTimeoutFlag == 1) {
    	// Set next timeout to be twice the previous timeout for a retransmit,
    	// as suggested by Karn/Partridge
    	// timeOut = 2 * timeOut;
    	
#ifdef TRANSPORT_DEBUG
        printf("resend timeOut = %l ms \n", timeOut);
#endif
        
        // Clear ackTimeoutFlag for connection
        connections[sendConnId].ackTimeoutFlag = 0;
        
        // Cancel (remove) ACK timeout alarm for connection
        mos_remove_alarm(&(connections[sendConnId].ackAlarm));
        
        if (connections[sendConnId].numRetransmits < MAX_RETRANSMITS) {
            (connections[sendConnId].numRetransmits)++;   

#ifdef TRANSPORT_DEBUG
            printf("Resending packet with seq num = %d \n", 
                connections[sendConnId].currSeqNum);
#endif
            
            // Resend packet
            connections[sendConnId].resendFlag = 1;
            // sendPacket(portNum, buffer, bufferLen, destNodeId);
            goto retransmit;
            
            // Free connection
            connections[sendConnId].connectionBusy = 0;
        }
        else {
            // MAX_RETRANSMITS attempted, move on to next packet
            // sendSeqNum++;
            connections[sendConnId].numRetransmits = 0;
        }    
    }
   
    // mos_mutex_unlock(&recvPktMutex);
    // printf("In sendPacket, unlocked recvPktMutex \n");
}

// Opens a connection to a remote node
int connect(uint8_t portNum, uint16_t destNodeId) {
    char messageBuff[PKT_DATA_SIZE];
    memset(messageBuff, 0, sizeof(messageBuff));
    uint8_t i = 0;
    int retVal = 0;
    
    // Establish CTP++ connection
    connectTo(destNodeId);
    
    char connectMessage[] = "CONNECT";
    
    // Send "CONNECT" message
    strcpy(messageBuff, connectMessage);
    sendPacket(portNum, messageBuff, strlen(messageBuff) + 1, destNodeId);
    
#ifdef TRANSPORT_DEBUG
    printf("Sent CONNECT message, waiting for response \n");
#endif
    
    // Get response (block waiting for response)
    receiveData(portNum, messageBuff, sizeof(messageBuff), 1);
    
#ifdef TRANSPORT_DEBUG
	printf("In connect(), processing response \n");
#endif
    
    // ACCEPTED response received
	if (strcmp(messageBuff, "ACCEPTED") == 0) {
        return CONNECT_OK;
    }
	else {
		// ACCEPTED response never received
#ifdef TRANSPORT_DEBUG
		printf("In connect(), ACCEPTED never received \n");
#endif
		return CONNECT_BAD;
	}
    /*
    for (i = 0; i < 10; i++) {
    	memset(messageBuff, 0, sizeof(messageBuff));
	    retVal = receiveData(portNum, messageBuff, sizeof(messageBuff), 0);
	    if (retVal == RECV_Q_UNDERFLOW) {
	    	// Receive queue is empty, so continue waiting
	    	mos_thread_sleep(3000);
	    }
	    else if (retVal == RECV_Q_OK) {
#ifdef TRANSPORT_DEBUG
	    	printf("In connect(), processing response \n");
#endif
	    	
	    	// ACCEPTED response received
	    	if (strcmp(messageBuff, "ACCEPTED") == 0) {
    	        return CONNECT_OK;
    	    }
	    }
    }
    
    // ACCEPTED response never received
#ifdef TRANSPORT_DEBUG
	printf("In connect(), ACCEPTED never received \n");
#endif
    return CONNECT_BAD;
    */
}

// Closes a connection to a remote node
int closeConn(uint8_t portNum, uint16_t destNodeId) {
    char messageBuff[PKT_DATA_SIZE];
    memset(messageBuff, 0, sizeof(messageBuff));
    
    char closeMessage[] = "CLOSE";
    
    // Send "CLOSE" message
    strcpy(messageBuff, closeMessage);
    sendPacket(portNum, messageBuff, strlen(messageBuff) + 1, destNodeId);
    
    // Get response (block waiting for response)
    memset(messageBuff, 0, sizeof(messageBuff));
    receiveData(portNum, messageBuff, sizeof(messageBuff), 1);
    
    // If response is CLOSED, return 0, else return -1 (error)
    if (strcmp(messageBuff, "CLOSED") == 0) {
    	// Close CTP++ connection
    	close(destNodeId);
        return 0;
    }
    else {
        return -1;
    }
}


// Sends data contained in buffer.  Buffer sizes larger than PKT_DATA_SIZE are
// segmented into multiple packets.
void sendData(uint8_t portNum, uint8_t *buffer, uint8_t bufferLen, 
		uint16_t destNodeId) {
    uint8_t bufferIndex = 0;
    
    if (bufferLen > PKT_DATA_SIZE) {
        // uint8_t remainingBufferLen = bufferLen; 
        
        // Segment buffer into multiple packets
        while (bufferLen > PKT_DATA_SIZE) {
            sendPacket(portNum, &(buffer[bufferIndex]), PKT_DATA_SIZE, 
            		destNodeId);
            bufferLen -= PKT_DATA_SIZE;
            bufferIndex += PKT_DATA_SIZE;       
        }
        
        if (bufferLen > 0) {
            // Send last segment of buffer
            sendPacket(portNum, &(buffer[bufferIndex]), bufferLen, destNodeId);
        }    
    }
    else {
        sendPacket(portNum, &(buffer[bufferIndex]), bufferLen, destNodeId);        
    }
}

// Sends an ACK packet to the specified port on the destination node
void sendAck(uint8_t portNum, uint16_t destNodeId) {
    // uint8_t nextNodeId = 0;
	
#ifdef TRANSPORT_DEBUG
	printf("Sending ACK to %d \n", destNodeId);
#endif
    
    // Store packet size in ACK packet
    sendPkt.size = ACK_PKT_SIZE;
    memcpy(&(sendPkt.data[SIZE_INDEX]), &(sendPkt.size), 
        sizeof(sendPkt.size));
    
    // Store received seq num in ACK packet to send
    memcpy(&(sendPkt.data[SEQ_NUM_INDEX]), &recvSeqNum, 
        sizeof(recvSeqNum));
     
    // Store node id for next hop in ACK packet to send
    // nextNodeId = LAST_NODE_ID - 1;
    // memcpy(&(sendPkt.data[NODE_ID_INDEX]), &destNodeId, sizeof(destNodeId));
       
    // Store port num in ACK packet
    memcpy(&(sendPkt.data[DEST_PORT_INDEX]), &(portNum), sizeof(portNum));
    
    // Store received connection id in ACK packet    
    /* memcpy(&(sendPkt.data[CONN_ID_INDEX]), &(recvdConnId), 
        sizeof(recvdConnId)); */
    memcpy(&(sendPkt.data[CONN_ID_INDEX]), &(portNum), sizeof(portNum));
    
    // Set ACK flag in packet to send
    memset(&(sendPkt.data[ACK_INDEX]), ACK_FLAG, sizeof(uint8_t));
    
    // Send ACK for received packet
    // com_send(IFACE_RADIO, &sendPkt);
    // Send packet using CTP++
    if (is_base == true) {
    	// Send downstream packet (base to node)
    	net_send(&sendPkt, CTP_PROTO_ID, TRANSPORT_LISTENING_PORT, 
    			CTP_NEW_PACKET, AM_CTP_DL_DATA, destNodeId, 
    			TRANSPORT_LISTENING_PORT);
    	
#ifdef TRANSPORT_DEBUG || defined(TRANSPORT_RECV_RATIO)
    	numPktsSent++;
    	printf("Num packets sent = %l \n", numPktsSent);
#endif
    }
    else {
        // Send upstream packet (node to base)
    	net_send(&sendPkt, CTP_PROTO_ID, TRANSPORT_LISTENING_PORT, 
    			CTP_NEW_PACKET, AM_CTP_DATA, TRANSPORT_LISTENING_PORT);
    	
#ifdef TRANSPORT_DEBUG || defined(TRANSPORT_RECV_RATIO)
    	numPktsSent++;
    	printf("Num packets sent = %l \n", numPktsSent);
#endif
    }
}

// Callback function called when an ACK receive has timed out
void ackTimeoutCallBack(void* data) {
    uint8_t connectionId = 0;
    
    // Set ackTimeoutFlag for the appropriate connection
    connectionId = *((uint8_t*)(data));
    connections[connectionId].ackTimeoutFlag = 1;
    
    mos_led_toggle(2);
}

// Adds packet data to receive queue for a port
void addToReceiveQueue(uint8_t *packetData, uint8_t packetDataLen, 
    uint8_t port) {
    int i;
    int retVal;
    
    mos_mutex_lock(&(receiveQueueMutexes[port]));
    for (i = 0; i < packetDataLen; i++) {
        retVal = mos_queue_add(&(receiveQueues[port]), packetData[i]);
     
        if (retVal == Q_FULL) {
#ifdef TRANSPORT_DEBUG
            printf("In addToReceiveQueue(): queue full \n");
            printf("receiveQueue length for port %d = %d \n", port, 
            		receiveQueues[port].length);
#endif
            
            // receive queue is full
            break;    
        }
    }
    mos_mutex_unlock(&(receiveQueueMutexes[port]));
}

// Thread to receive packets
void receiveThread() {
	while (true) {
#ifdef TRANSPORT_DEBUG
		printf("In receiveThread, calling receivePacket() ... \n");
#endif
		receivePacket(TRANSPORT_LISTENING_PORT);
		
		// TBD: Add support for receiving packets on multiple ports
	}
}

// Initialize transport protocol and network layer (CTP++)
void transportInit(boolean isBaseNode) {
	mos_mutex_init(&recvPktMutex);
    mos_mutex_init(&sendPktMutex);
    mos_mutex_init(&connectionsMutex);
    mos_mutex_init(&sendConnReplyFlagMutex);
    // mos_sem_init(&ackSem, 0);
    uint8_t i;
    
    // Initialize packet receive queues
    for (i = 0; i < MAX_PORT_NUM + 1; i++) {
    	memset(receiveBufs[i], 0, sizeof(receiveBufs[i]));
        mos_queue_init(&(receiveQueues[i]), receiveBufs[i], 
            (uint8_t)(PKT_SIZE * MAX_QUEUE_LEN));
        
        mos_mutex_init(&(receiveQueueMutexes[i]));
    }
    
    // Initialize connections[] and serverConnections[]
    for (i = 0; i < MAX_CONNECTIONS; i++) {
        connections[i].connectionBusy = 0;
        connections[i].currSeqNum = 0;
        connections[i].recvSeqNum = 0;
        connections[i].ackTimeoutFlag = 0;
        connections[i].resendFlag = 0;
        connections[i].numRetransmits = 0;
        connections[i].ackRecvd = 0;
        // connections[i].packetRecvd = 0;
        memset(connections[i].connReplyMessBuff, 0, 
        		sizeof(connections[i].connReplyMessBuff));
        connections[i].sendConnReplyFlag = 0;
        
        // Initialize ACK timeout alarm
        connections[i].ackAlarm.func = ackTimeoutCallBack;
        connections[i].ackAlarm.msecs = ACK_TIMEOUT;
        connections[i].ackAlarm.reset_to = 0;
        
        serverConnections[i].connectionBusy = 0;
        serverConnections[i].connectionOpen = 0;
        serverConnections[i].currSeqNum = 0;
    }
    
    if (isBaseNode == true) {
    	is_base = true;
    }
    else {
    	is_base = false;
    }
    
    // Initialize net layer
    net_init();
	   
    // Start the CTP backends 
    ctp_proto_init();
    
    /*
    if (isBaseNode == true) {
    	net_ioctl(CTP_PROTO_ID, CTP_SET_IS_BASE);
    }
    else {
    	net_ioctl(CTP_PROTO_ID, CTP_SET_IS_NODE);
    }
    */
	
    // Wait a while till the routing is possibly established      
    mos_mdelay(10);
    
    mos_thread_new(receiveThread, 384, PRIORITY_HIGH);
}
