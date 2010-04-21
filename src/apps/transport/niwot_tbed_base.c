#include "mos.h"
#include "msched.h"
#include "transport.h"
#include <string.h>
#include "net.h"
#include "node_id.h"
#include "ctp_plus.h"
#include "printf.h"

#define NODE_ID 0
#define NUM_NODES 3

void appRecv() {
	char recvdDataBuf[50];
	uint8_t portNum = 0;
	int retVal = 0;
	
	while (1) {
		for (portNum = 0; portNum < NUM_NODES; portNum++) {
			memset(recvdDataBuf, 0, sizeof(recvdDataBuf));
			
			// printf("Calling receiveData() for port %d \n", portNum);
			retVal = receiveData(portNum, recvdDataBuf, sizeof(recvdDataBuf), 
					0);
			if (retVal == RECV_Q_OK) {
				printf("In appRecv(), from %d port, received data: \n", 
						portNum);
				printf("%s \n", recvdDataBuf);
			}
		}
		
		mos_thread_sleep(2000);
	}
	
}

void start(void) {
	uint16_t power = 31;
	mos_node_id_set(NODE_ID);
	
	transportInit(true);
	net_ioctl(CTP_PROTO_ID, CTP_SET_TESTBED);
	// net_ioctl(CTP_PROTO_ID, CTP_SET_POWER, power);
	
	mos_thread_new(appRecv, 384, PRIORITY_NORMAL);
}
