#include "mos.h"
#include "msched.h"
#include "transport.h"
#include <string.h>
#include "net.h"
#include "node_id.h"
#include "ctp_plus.h"

// Application for a node in the middle of the network (not source or base 
// station).  No application threads are needed, since the net thread in the
// MOS net layer handles all packet processing.


void start(void) {
	mos_node_id_set(2);
	
	is_base = false;
	
	// Initialize net layer
	net_init();
		   
	// Start the CTP backends 
	ctp_proto_init();
	
	// Wait a while till the routing is possibly established      
	mos_mdelay(10);
	
	net_ioctl(CTP_PROTO_ID, CTP_SET_TESTBED);
	com_ioctl_IFACE_RADIO (CC2420_TX_POWER, 2); //set the tx power from 0 (min) to 31 (max)  
	cc2420_set_channel(26);
	printf("Running net layer and CTP backend... \n");
	
	// transportInit(true);
}
