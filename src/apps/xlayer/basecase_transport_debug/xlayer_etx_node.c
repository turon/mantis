#include "mos.h"
#include "msched.h"
#include "transport.h"
#include <stdlib.h>
#include <string.h>
#include "net.h"
#include "node_id.h"
#include "ctp_plus.h"


void appSend() {
	char     sendData[12];
	char     testStr[] = "test";
	char     iStr[3];
	uint8_t  i = 0;
	uint16_t pause_time = 5000; 
    uint16_t destETX = 0;
	uint16_t estimatedETX = 0; 
	uint16_t ETXdiff = 0;
	uint16_t ETXarr[5]; //storing ETX values for exponential averaging 
	uint16_t previousETX = 0; 
	uint8_t  arrCount; 
	uint8_t  alpha = 8; //how much important of the current value to the previous values 
	uint8_t  setup_time = 0; //need around 10 seconds to get everything up 

	// Avoid signal interference with default channel (26) and wi-fi 
	cc2420_set_channel(26);

	// Set transmit power to low-power mode 
	//com_ioctl(IFACE_RADIO, CC2420_LOW_POWER_MODE);
	com_ioctl_IFACE_RADIO (CC2420_TX_POWER, 2); //set the tx power from 0 (min) to 31 (max)  
	
	printf("Opening connection on port %d... \n", TRANSPORT_LISTENING_PORT);
	connect(TRANSPORT_LISTENING_PORT, 0);
	printf("Opened connection on port %d \n", TRANSPORT_LISTENING_PORT);
	
	while (true) {
		// Send data to base
		memset(sendData, 0, sizeof(sendData));
		strcpy(sendData, testStr);
		itoa(i, iStr, 10);
		strcat(sendData, iStr);
		
		// Stop sending if pause_time more than 30 seconds 
		if (pause_time < 30000) {
			printf("Calling sendPacket() for: %s \n", sendData);
			sendPacket(TRANSPORT_LISTENING_PORT, sendData, sizeof(sendData), 0);
		} else {
			mos_thread_sleep(50000); //sleep 50 seconds (and hope the link better afterwards)  
			pause_time -= 500; //increase send rate a bit, otherwise the program is locked 
			break; //and then back to the loop again 
		}

		i++;
		// Reset counter
		if (i >= 60) {
			i = 0;
		}
    	
		setup_time++; 
		
		if (setup_time > 10)
			setup_time = 13; //protect a wrap-around 

		if (pause_time < 0) 
			pause_time = 0; //pause_time can never be less than 0 (send continuously) 
		
		mos_thread_sleep(pause_time); //adaptive send rate 
	
		if (arrCount == 5)
			arrCount = 0; 
	}
	
	// Close connection to destination (last) node; port 0
	closeConn(0, 3);
	printf("Closed connection on port %d \n", TRANSPORT_LISTENING_PORT);
}


void start(void) {
	mos_node_id_set(6);
	transportInit(false);
	net_ioctl(CTP_PROTO_ID, CTP_SET_TESTBED);
	
	mos_thread_new(appSend, 384, PRIORITY_NORMAL);
}

