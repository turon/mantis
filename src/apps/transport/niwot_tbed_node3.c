#include "mos.h"
#include "msched.h"
#include "transport.h"
#include <stdlib.h>
#include <string.h>
#include "net.h"
#include "node_id.h"
#include "ctp_plus.h"
#include "printf.h"

#define NODE_ID 3
#define BASE_PORT (NODE_ID - 1)

extern uint8_t _end;

void appSend() {
	char sendData[50];
	uint8_t packetCounter = 0;
	char packetCounterStr[3];
	int retVal = 0;
	
	mos_thread_sleep(61660);
	printf("Opening connection on port %d... \n", BASE_PORT);
	while (true) {
		retVal = connect(BASE_PORT, 0);
		if (retVal == CONNECT_OK) {
			// Successfully connected
			break;
		}
		// Else retry opening of connection
	}
	printf("Opened connection on port %d \n", BASE_PORT);
	
	while (true) {
		// Diagnostic data:
		// Data string type ("T" for tsoil)
		// RSSI (ETX) - stored in packet footer by CTP++
		// Battery voltage - TBD
		// # hops and path - stored in packet footer by CTP++
		// receive ratio for each node - printed by transport
		
		// Canned message of 45 bytes
		char testStr[] = "a_canned_message_containing_45_bytes_of_chars";
		char dataTypeStr[] = "T";
		
		// TBD - get battery voltage and append to data to send
		
		// Construct data to send to base
		memset(sendData, 0, sizeof(sendData));
		strcpy(sendData, testStr);
		strcat(sendData, dataTypeStr);
		itoa(packetCounter, packetCounterStr, 10);
		strcat(sendData, packetCounterStr);
		
		printf("sendData: %s \n", sendData);
		
		packetCounter++;
		// Reset counter
		if (packetCounter > 255) {
			packetCounter = 0;
		}
		
		// Send data to base
		printf("Calling sendPacket() for: \n");
		printf("%s \n", sendData);
		sendPacket(BASE_PORT, sendData, sizeof(sendData), 0);
		
		mos_thread_sleep(2500);
	}
	
	// Close connection to destination (last) node; port 0
	closeConn(BASE_PORT, 0);
	printf("Closed connection on port %d \n", BASE_PORT);
	
}

void start(void) {
	uint16_t power = 31;
	uint8_t retVal = 0;
	
	mos_node_id_set(NODE_ID);
	transportInit(false);
	net_ioctl(CTP_PROTO_ID, CTP_SET_TESTBED);
	// net_ioctl(CTP_PROTO_ID, CTP_SET_POWER, power);
	
	// Sleep for 30 sec to allow CTP++ routing to stabilize
	mos_thread_sleep(30000);
	
	printf("Starting appSend thread... \n");
	retVal = mos_thread_new(appSend, 384, PRIORITY_NORMAL);
	if (retVal != THREAD_OK) {
		printf("mos_thread_new retval = %d \n", retVal);
	}
	
	printf("_end = %d \n", (int)&_end);
}
