#include "mos.h"
#include "msched.h"
#include "transport.h"
#include <stdlib.h>
#include <string.h>
#include "net.h"
#include "node_id.h"
#include "ctp_plus.h"

extern uint8_t _end;

void appSend() {
	char sendData[12];
	char testStr[] = "test";
	char iStr[3];
	uint8_t i = 0;
	int retVal = 0;
	
	printf("Opening connection on port %d... \n", TRANSPORT_LISTENING_PORT);
	while (true) {
		retVal = connect(TRANSPORT_LISTENING_PORT, 0);
		if (retVal == CONNECT_OK) {
			// Successfully connected
			break;
		}
		// Else retry opening of connection
	}
	printf("Opened connection on port %d \n", TRANSPORT_LISTENING_PORT);
	
	while (true) {
		// Send data to base
		memset(sendData, 0, sizeof(sendData));
		strcpy(sendData, testStr);
		itoa(i, iStr, 10);
		strcat(sendData, iStr);
		
		printf("Calling sendPacket() for: %s \n", sendData);
		sendPacket(TRANSPORT_LISTENING_PORT, sendData, sizeof(sendData), 0);
		
		i++;
		// Reset counter
		if (i >= 60) {
			i = 0;
		}
		
		mos_thread_sleep(5000);
	}
	
	// Close connection to destination (last) node; port 0
	closeConn(0, 3);
	printf("Closed connection on port %d \n", TRANSPORT_LISTENING_PORT);
	
}

void start(void) {
	uint16_t power = 31;
	uint8_t retVal = 0;
	
	mos_node_id_set(6);
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
