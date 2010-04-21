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
	// cc2420_set_channel(26);

	// Initialize ETX array 
	for (i = 0; i < 5; i++) {
		ETXarr[i] = 0;
	}
	i = 0; 

	// Set transmit power to low-power mode 
	//com_ioctl(IFACE_RADIO, CC2420_LOW_POWER_MODE);
	// com_ioctl_IFACE_RADIO (CC2420_TX_POWER, 2); //set the tx power from 0 (min) to 31 (max)  
	
	printf("Opening connection on port %d... \n", TRANSPORT_LISTENING_PORT);
	connect(TRANSPORT_LISTENING_PORT, 0);
	printf("Opened connection on port %d \n", TRANSPORT_LISTENING_PORT);
	
	arrCount = 0; //initialize position in ETX array 

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
			mos_thread_sleep(20000); //sleep 20 seconds (and hope the link better afterwards)  
			pause_time -= 1000; //increase send rate a bit, otherwise the program is locked 
			break; //and then back to the loop again 
		}

		i++;
		// Reset counter
		if (i >= 60) {
			i = 0;
		}
    	
		// Get ETX value for destination
    	// Use appropriate packet type depending on whether destination is
    	// upstream or downstream
		if (is_base == true) {
    		destETX = getETX(AM_CTP_DL_DATA, 0);
    	}
    	else {
    		destETX = getETX(AM_CTP_DATA, 0);
    	}
    	
		ETXarr[arrCount] = destETX; //current ETX 
		previousETX = (ETXarr[0] + ETXarr[1] + ETXarr[2] + ETXarr[3] + ETXarr[4] - ETXarr[arrCount])/4; 
		estimatedETX = (alpha * ETXarr[arrCount] + (10-alpha) * previousETX)/10; //exponential averaging 
		ETXdiff = ETXarr[arrCount] - estimatedETX; 
		arrCount++; //next position in ETX array 
		
#ifdef XLAYER_DEBUG
		printf("*** XLAYER DEBUG MSG: destETX = %d \n", destETX);
    	printf("*** XLAYER DEBUG MSG: pause_time = %d \n", pause_time);
		printf("*** XLAYER DEBUG MSG: ETXarr = %d  %d  %d  %d  %d \n", ETXarr[0], ETXarr[1], ETXarr[2], ETXarr[3], ETXarr[4]); 
		printf("*** XLAYER DEBUG MSG: ETXdiff = %d \n", ETXdiff); 
    	printf("*** XLAYER DEBUG MSG: estimatedETX = %d \n", estimatedETX);
#endif

		// Compare current ETX to the estimated exponential-averaging value 
		if (setup_time > 10) { //wait around 10 seconds to get the algorithm properly initialized 
			if (ETXdiff < 0) {
				printf("Link better, send faster \n"); 
				pause_time -= (2*pause_time)/10; //decrease send rate by 20%
			} else if (ETXdiff == 0) {
				printf("Link same, do nothing \n"); 
			} else if (ETXdiff > 0) {
				printf("Link worse, send slower \n"); 
				pause_time += (2*pause_time)/10; //increase send rate by 20% 
			}
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
	uint16_t power = 30; // Transmit power
	
	mos_node_id_set(6);
	transportInit(false);
	// net_ioctl(CTP_PROTO_ID, CTP_SET_TESTBED);
	net_ioctl(CTP_PROTO_ID, CTP_SET_POWER, power);
	
	// Sleep for 20 sec to allow CTP++ routing to stabilize
	mos_thread_sleep(20000);
	
	mos_thread_new(appSend, 384, PRIORITY_NORMAL);
}

