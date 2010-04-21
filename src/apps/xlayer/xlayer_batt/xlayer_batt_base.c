#include "mos.h"
#include "msched.h"
#include "transport.h"
#include <string.h>
#include "net.h"
#include "node_id.h"
#include "ctp_plus.h"


void appRecv() {
	char recvdDataBuf[12];
	
	// avoid signal interference with default channel (26) and wi-fi 
	cc2420_set_channel(24);

	while (1) {
		memset(recvdDataBuf, 0, sizeof(recvdDataBuf));
		
		receiveData(TRANSPORT_LISTENING_PORT, recvdDataBuf, 
				sizeof(recvdDataBuf), 1);
		printf("In appRecv(), received data: %s \n", recvdDataBuf);
	}
}


void start(void) {
	mos_node_id_set(0);
	
	transportInit(true);
	net_ioctl(CTP_PROTO_ID, CTP_SET_TESTBED);
	
	mos_thread_new(appRecv, 384, PRIORITY_NORMAL);
}

