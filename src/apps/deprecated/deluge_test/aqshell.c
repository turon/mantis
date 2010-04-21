/* This program is a simple shell for monitoring nodes running Aqueduct.
 * When possible it makes a connection to a Java server that controls 
 * multiple shells.  When it is not connected to the Java server it listens
 * for a connection.  See also src/apps/deluge_test/ExperServer.java.
 * 
 * Note: ExperServer is a Java program so all data are sent to it in
 * big-endian (network) byte order.  This code assumes that host byte
 * order and the sensor node's endianness are the same, so it probably
 * needs to be modified before you run it on your PowerPC-based Mac.
 */

#include "mos.h"

#include <stdio.h>
#include "msched.h"
#include "com.h"
#include "aqueduct_shell.h"
#include <sys/time.h>
#include "deluge_impl.h"
#include "arg.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <signal.h>
#include <errno.h>

#define TEST_PORT 58799		// ExperServer listens on this port
#define SERIAL_PORT 58800	// Unused
#define USB_PORT 58900		// Node on ttyUSB0 listens on port 58900, ttyUSB1 -> 58901, etc.

// Synchronize the beginning of a packet, probably not necessary for TCP
#define SYNC_BYTE_1 '#'
#define SYNC_BYTE_2 '3'
#define SYNC_BYTE_3 'C'

static char syncbytes[3] = { SYNC_BYTE_1, SYNC_BYTE_2, SYNC_BYTE_3 };

// Terminal color codes
static const char WHITE[] = "\e[0m";
static const char YELLOW[] = "\e[40;33m";
static const char CYAN[] = "\e[40;36m";
static const char GREEN[] = "\e[40;32m";
static const char RED[] = "\e[40;31m";
//static const char HI_WHITE[] = "";
static const char HI_YELLOW[] = "\e[30;43m";
static const char HI_CYAN[] = "\e[30;46m";
static const char HI_GREEN[] = "\e[30;42m";
static const char HI_RED[] = "\e[30;41m";

static char* serial_file;		// Example: /dev/ttyUSB0
static FILE* log_file;			// Where we're logging to
static int printing;			// Are we printing?
static int printPkts;			// Are we printing packet dumps?
//static char buffer[1024];
static int ssock = -1;			// Socket when we are server
static int sport = 0;			// Port we're listening on
static int csock = -1;			// Socket the shell actually uses
static char* hostname;			// Host that ExperServer is running on
static uint16_t node_id;		// Remember the last id we heard from the node

static uint16_t packet_counts[DELUGE_STATS_COUNT][4];	// Store packet counts here
static uint8_t recordstats = 0;	// Are we recording stats?
static int allquiet = 1;		// No reprogramming traffic from this node recently?

static comBuf spkt;
static struct {
	struct aqshell_header head;
	uint8_t data[256];
} aqpkt;

static int shutdownMode = 0;	// Are we waiting to shut down?

// TODO Not entirely sure if this is a proper signal handler
// TODO This works find with Fedora Core 3, with Gentoo I see multiple signals
// for one control-C keystroke.
static void sigHandler(int sig)
{
	int e = errno;
	switch (sig)
	{
		case SIGINT:	// ctrl-C
		case SIGTERM:	// kill
		{
			write(1, "Caught signal.\n", 15);
			if (shutdownMode) {
				// We already caught this signal once, and nothing is happening
				exit(1);
			}
			shutdownMode = 1;
			
			if (ssock != -1) close(ssock);
			if (csock != -1) {
				// Tell the client to close its connection.  It's a cleaner
				// shutdown that way.  Otherwise the port may appear to be
				// in use for several seconds after the shell closes.
				// TODO looks like this tells the other end to shut down even if it
				// is the server, problem?
				write(csock, syncbytes, 3);
				int zero = 0;
				write(csock, &zero, 4);
				write(csock, &zero, 4);
				
				aqpkt.head.command = AQSHELL_CLOSE;
				aqpkt.head.flags = AQSHELL_F_SHELLCTL;
				aqpkt.head.length = 0;
				write(csock, &aqpkt, sizeof(aqpkt.head));
				write(1, "Waiting for client to close the socket.\n", 49);
			} else {
				// No clients to wait on
				exit(0);
			}
			break;
		}
		case SIGALRM:
			// If this alarm goes off, then the connected node has not
			// sent a profile, request, or data packet for the last 
			// 30 seconds
			if (csock != -1) {
				allquiet = 1;
				
				write(csock, syncbytes, 3);
				int zero = 0;
				write(csock, &zero, 4);
				write(csock, &zero, 4);
				
				aqpkt.head.command = AQSHELL_ALLQUIET;
				aqpkt.head.flags = AQSHELL_F_SHELLCTL;
				//aqpkt.head.id = htons(node_id);
				aqpkt.head.length = 0;
				write(csock, &aqpkt, sizeof(aqpkt.head));
				write(1, "All quiet.\n", 11);
			}
			break;
	}
	errno = e;
}

#if 0		// For debugging the shell
static void dumpPkt(char* msg, comBuf* pkt)
{
	printf("%s:", msg);
	int i;
	for (i=0; i<pkt->size; i++)
		printf(" %02x", pkt->data[i]);
	printf("\n");
}

static ssize_t dbg_recv(int s, void *buf, size_t len, int flags)
{
	ssize_t ret = recv(s, buf, len, flags);
	if (ret == -1)
		perror("dbg_recv: recv");
	else  {
		printf("%d bytes from net:", ret);
		int i;
		uint8_t* buf2 = (uint8_t*)buf;
		for (i=0; i<ret; i++)
			printf(" %02x", buf2[i]);
		printf("\n");
	}
	return ret;
}
#else
static inline void dumpPkt(char* msg, comBuf* pkt) {} 
#define dbg_recv recv
#endif

// Print a 16-bit value in binary
static char* bin16(uint16_t n, char* buffer)
{
	char* p = buffer+16;
	*p = 0;
	do {
		*--p = '0' + (n & 1);
		n >>= 1;
	} while (p != buffer);
	return buffer;
}

// Forward a packet from the node to ExperServer
static void forward(struct timeval *timestamp, struct aqshell_pkt* pkt)
{
	if (csock == -1) return;
	
	// Packets sent to ExperServer include a timestamp
	int ts = htonl((int)timestamp->tv_sec);
	int tu = htonl((int)timestamp->tv_usec);
	pkt->head.id = htons(pkt->head.id);
	
	int sendlen = sizeof(pkt->head) + pkt->head.length;
	if (send(csock, syncbytes, 3, 0) != 3 ||
		send(csock, &ts, 4, 0) != 4 ||
		send(csock, &tu, 4, 0) != 4 ||
		send(csock, pkt, sendlen, 0) != sendlen)
	{
		perror("forward: send");
	}
}

// Listen to the serial port for command packets from the node
void recvThread()
{
	struct timeval timestamp;
	char binbuf2[17];
	char binbuf1[17];
	char binbuf0[17];
	
	com_mode(IFACE_SERIAL, IF_LISTEN);
	while (!shutdownMode)
	{
		comBuf* cpkt = com_recv(IFACE_SERIAL);
		if (shutdownMode) return;
		
		gettimeofday(&timestamp, NULL);
		struct aqshell_pkt* pkt = (struct aqshell_pkt*)cpkt->data;
		/*if (log_file) {
			fwrite(&timestamp, sizeof(timestamp), 1, log_file);
			fwrite(pkt->data, pkt->size, 1, log_file);
		}*/
		dumpPkt("from node", cpkt);
		if (printPkts) printf("%10d.%06d ", (int)timestamp.tv_sec, (int)timestamp.tv_usec);
		node_id = pkt->head.id;
		
		// Forward replies to ExperServer
		if ((pkt->head.flags & AQSHELL_F_FORWARDREPLY) && pkt->head.command != AQSHELL_MESSAGE)
			forward(&timestamp, pkt);
		int notquiet = 0;
		
		switch (pkt->head.command)
		{
			case 200: break;
			case AQSHELL_CLEARSTATS:
			case AQSHELL_STARTSTATS:
			case AQSHELL_STOPSTATS:
			case AQSHELL_SAVESTATS:
			case AQSHELL_GETSTATS:
			case AQSHELL_GETID:
			case AQSHELL_NOOP:
			case AQSHELL_GETID_REPLY:
			case AQSHELL_GETVERSION:
			case AQSHELL_SETIMAGESIZE:
			case AQSHELL_SETCACHESIZE:
			case AQSHELL_STOPUPDATE:
				if (printing) printf("%c\n", pkt->head.command);
				break;
			case AQSHELL_CACHEHIT:
				if (recordstats) packet_counts[DELUGE_STATS_COUNT-2][0]++;		// cache hit
				if (printing) printf("%c\n", pkt->head.command);
				break;
			case AQSHELL_CACHEMISS:
				if (recordstats) packet_counts[DELUGE_STATS_COUNT-2][1]++;		// cache miss
				if (printing) printf("%c\n", pkt->head.command);
				break;
			case AQSHELL_CACHEHITFORWARD:
				if (recordstats) packet_counts[DELUGE_STATS_COUNT-2][2]++;		// cache hit forward
				if (printing) printf("%c\n", pkt->head.command);
				break;
			case AQSHELL_CACHEHITOLDFORWARD:
				if (recordstats) packet_counts[DELUGE_STATS_COUNT-2][3]++;		// cache hit past forward
				if (printing) printf("%c\n", pkt->head.command);
				break;
			case AQSHELL_MESSAGE:
				cpkt->data[cpkt->size] = 0;
				if (printing) printf("%s", &cpkt->data[1]);
				break;
			case AQSHELL_START:
				if (printing) printf("start\n");
				forward(&timestamp, pkt);
				break;
			case AQSHELL_NEWVERSION:
				if (printing) printf("new version\n");
				forward(&timestamp, pkt);
				break;
			case AQSHELL_COMPLETEPAGE:
				if (printing) printf("completed page\n");
				forward(&timestamp, pkt);
				break;
			case AQSHELL_COMPLETEPROG:
				if (printing) printf("completed program\n");
				forward(&timestamp, pkt);
				break;
			case AQSHELL_GETSTATS_REPLY:
				if (printing) printf("stats\n");
				forward(&timestamp, pkt);
				break;
			case AQSHELL_SETVERSION:
			case AQSHELL_SETVERSION_REPLY:
				if (printing) printf("version ack\n");
				//forward(&timestamp, pkt);
				break;
			case AQSHELL_SUMMARY: {
				deluge_foot_summary* summary = (deluge_foot_summary*)&pkt->data[0];
				if (printPkts) printf("%d>>%d %d ", pkt->data[pkt->head.length-1], summary->id, summary->type);
				if (printPkts) printf("%s[%3d %3d %3d %3d %3d] ", YELLOW,
					summary->neighbors[0], summary->neighbors[1], 
					summary->neighbors[2], summary->neighbors[3], 
					summary->neighbors[4]);
				if (printPkts) printf("%3d %3d %3d%s\n", summary->version,
					summary->highPage, summary->dtb, WHITE);
				if (recordstats && summary->id < DELUGE_STATS_COUNT) packet_counts[summary->id][0]++;
				break;
			}
			case AQSHELL_PROFILE: {
				deluge_foot_profile* profile = (deluge_foot_profile*)&pkt->data[0];
				if (printPkts) printf("%d>>%d %d ", pkt->data[pkt->head.length-1], profile->id, profile->type);
				if (printPkts) printf("%s%10d %5d %3d %3d%s\n", CYAN,
					profile->codeSize, profile->crc, 
					profile->version, profile->goalPage, WHITE);
				if (recordstats && profile->id < DELUGE_STATS_COUNT) packet_counts[profile->id][1]++;
				break;
			}
			case AQSHELL_REQUEST: {
				deluge_foot_request* request = (deluge_foot_request*)&pkt->data[0];
				if (printPkts) printf("%d>>%d %d ", pkt->data[pkt->head.length-1], request->id, request->type);
				if (printPkts) printf("%s%s %s %s %5d %3d %3d %3d%s\n", GREEN,
					bin16(request->packets[2],binbuf2), 
					bin16(request->packets[1],binbuf1), 
					bin16(request->packets[0],binbuf0), 
					request->to, 
					request->version, request->page, request->rateChange, WHITE);
				if (recordstats && request->id < DELUGE_STATS_COUNT) packet_counts[request->id][2]++;
				break;
			}
			case AQSHELL_DATA: {
				deluge_foot_data* data = (deluge_foot_data*)&pkt->data[0];
				if (printPkts) printf("%d>>%d %d ", pkt->data[pkt->head.length-1], data->id, data->type);
				if (printPkts) printf("%s%3d %3d %3d%s\n", RED,
					data->version, data->page, data->packet, WHITE);
				if (recordstats && data->id < DELUGE_STATS_COUNT) packet_counts[data->id][3]++;
				break;
			}
			case AQSHELL_SUMMARY_SEND: {
				deluge_foot_summary* summary = (deluge_foot_summary*)&pkt->data[0];
				if (printPkts) printf("%d<<%d %d ", pkt->data[pkt->head.length-1], summary->id, summary->type);
				if (printPkts) printf("%s[%3d %3d %3d %3d %3d] ", HI_YELLOW,
					summary->neighbors[0], summary->neighbors[1], 
					summary->neighbors[2], summary->neighbors[3], 
					summary->neighbors[4]);
				if (printPkts) printf("%3d %3d %3d%s\n", summary->version,
					summary->highPage, summary->dtb, WHITE);
				if (recordstats && summary->id < DELUGE_STATS_COUNT) packet_counts[summary->id][0]++;
				break;
			}
			case AQSHELL_PROFILE_SEND: {
				deluge_foot_profile* profile = (deluge_foot_profile*)&pkt->data[0];
				if (printPkts) printf("%d<<%d %d ", pkt->data[pkt->head.length-1], profile->id, profile->type);
				if (printPkts) printf("%s%10d %5d %3d %3d%s\n", HI_CYAN,
					profile->codeSize, profile->crc, 
					profile->version, profile->goalPage, WHITE);
				if (recordstats && profile->id < DELUGE_STATS_COUNT) packet_counts[profile->id][1]++;
				notquiet = 1;
				break;
			}
			case AQSHELL_REQUEST_SEND: {
				deluge_foot_request* request = (deluge_foot_request*)&pkt->data[0];
				if (printPkts) printf("%d<<%d %d ", pkt->data[pkt->head.length-1], request->id, request->type);
				if (printPkts) printf("%s%s %s %s %5d %3d %3d %3d%s\n", HI_GREEN,
					bin16(request->packets[2],binbuf2), 
					bin16(request->packets[1],binbuf1), 
					bin16(request->packets[0],binbuf0), 
					request->to, 
					request->version, request->page, request->rateChange, WHITE);
				if (recordstats && request->id < DELUGE_STATS_COUNT) packet_counts[request->id][2]++;
				notquiet = 1;
				break;
			}
			case AQSHELL_DATA_SEND: {
				deluge_foot_data* data = (deluge_foot_data*)&pkt->data[0];
				if (printPkts) printf("%d<<%d %d ", pkt->data[pkt->head.length-1], data->id, data->type);
				if (printPkts) printf("%s%3d %3d %3d%s\n", HI_RED,
					data->version, data->page, data->packet, WHITE);
				if (recordstats && data->id < DELUGE_STATS_COUNT) packet_counts[data->id][3]++;
				notquiet = 1;
				break;
			}
			default:
				if (printing) printf("Unknown packet type: %#02X\n", pkt->head.command);
				break;
		}	

		if (allquiet && notquiet) {
			if (csock != -1) {
				int ts = htonl((int)timestamp.tv_sec);
				int tu = htonl((int)timestamp.tv_usec);
				
				pkt->head.command = AQSHELL_NOTQUIET;
				pkt->head.flags = AQSHELL_F_SHELLCTL;
				//pkt->head.id = htons(node_id);
				pkt->head.length = 0;
		
				if (write(csock, syncbytes, 3) != 3 ||
					write(csock, &ts, 4) != 4 ||
					write(csock, &tu, 4) != 4 ||
					write(csock, pkt, sizeof(pkt->head)) != sizeof(pkt->head))
				{
					perror("sendNotQuiet: write");
				}
			}
			allquiet = 0;
			printf("Not quiet.\n");
		}
		if (notquiet) {
			alarm(30);
		}
		
		com_free_buf(cpkt);
	}
}

static const char HANDSHAKE[] = "aqshell Handshake";

// Exchange a handshake string so we know we have the right protocol
static int doHandshake(int csock, int isServer)
{
	char buffer[sizeof(HANDSHAKE)];
	short hlen = (short)sizeof(HANDSHAKE)-1;
	short nlen = htons(hlen);
	
	if (isServer) {
		// server goes first
		// TODO actually the order doesn't matter, both ends should send, then both check
		if (send(csock, &nlen, 2, 0) != 2 || send(csock, HANDSHAKE, hlen, 0) != hlen) {
			perror("doHandshake: send");
			return -1;
		}
	}
		
	if (recv(csock, &nlen, 2, 0) != 2) {
		perror("doHandshake: recv");
		return -1;
	}
	hlen = ntohs(nlen);
	if (hlen != sizeof(HANDSHAKE)-1) {
		fprintf(stderr, "Bad handshake length %d\n", hlen);
		return -1;
	}
	if (recv(csock, buffer, hlen, 0) != hlen) {
		perror("doHandshake: recv");
		return -1;
	}
	buffer[hlen] = 0;
	if (strcmp(HANDSHAKE, buffer) != 0) {
		fprintf(stderr, "Bad handshake %.*s\n", hlen, buffer);
		return -1;
	}
	
	if (!isServer) {
		// client goes second
		if (send(csock, &nlen, 2, 0) != 2 || send(csock, HANDSHAKE, hlen, 0) != hlen) {
			perror("doHandshake: send");
			return -1;
		}
		// Send our port so ExperServer knows which shells it is listening to
		int port = htonl(sport);
		if (send(csock, &port, 4, 0) != 4) {
			perror("doHandshake: send");
			return -1;
		}
	}
	printf("Good handshake.\n");
	return 0;
}

// Handle a command sent from ExperServer
static void doNetCommand(struct aqshell_pkt* pkt)
{
	if (pkt->head.flags & AQSHELL_F_NODECTL) {
		// The command is meant for the node, so forward it
		spkt.size = sizeof(pkt->head)+pkt->head.length;
		memcpy(spkt.data, pkt, spkt.size);
		dumpPkt("to node", &spkt);
		com_send(IFACE_SERIAL, &spkt);
	} else if (pkt->head.flags & AQSHELL_F_SHELLCTL) {
		// This command is meant for this shell
		switch (pkt->head.command) {
			case AQSHELL_CLEARSTATS:
				memset(packet_counts, 0, sizeof(packet_counts));
				if (printing) printf("Cleared stats.\n");
				break;
			case AQSHELL_STARTSTATS:
				recordstats = 1;
				if (printing) printf("Start recording stats.\n");
				break;
			case AQSHELL_STOPSTATS:
				recordstats = 0;
				if (printing) printf("Stop recording stats.\n");
				break;
			case AQSHELL_GETSTATS: {
				if (printing) printf("Sending stats.\n");
				pkt->head.id = htons(node_id);
				pkt->head.length = 2 + 4 * 2 * 4;
				
				uint16_t r;
				for (r = 0; r<DELUGE_STATS_COUNT; ) {
					uint16_t* data = (uint16_t*)pkt->data;
					*data++ = htons(r);
					int i;
					for (i=0; i<4; i++) {
						int j;
						for (j=0; j<4; j++) {
							*data++ = htons(packet_counts[r][j]);
						}
						r++;
					}
					
					int zero = 0;
					int sendlen = sizeof(pkt->head) + pkt->head.length;
					if (send(csock, syncbytes, 3, 0) != 3 ||
						send(csock, &zero, 4, 0) != 4 ||
						send(csock, &zero, 4, 0) != 4 ||
						send(csock, pkt, sendlen, 0) != sendlen)
					{
						perror("doNetCommand: send");
						break;
					}
				}
				break;
			}
			case AQSHELL_CLOSE:
				if (close(csock) == -1) perror("doNetCommand: close");
				csock = -1;
				break;
			default:
				if (printing) printf("Unknown packet type: %#02X\n", pkt->head.command);
				// Don't ACK a bad command
				pkt->head.flags &= ~AQSHELL_F_PLEASEACK;
				break;
		}
		// Send an ACK if we're supposed to
		if (pkt->head.flags & AQSHELL_F_PLEASEACK) {
			pkt->head.flags |= AQSHELL_F_ACK;
			pkt->head.id = htons(node_id);
			pkt->head.length = 0;
			int zero = 0;
			int sendlen = sizeof(struct aqshell_header);
			if (send(csock, syncbytes, 3, 0) != 3 ||
				send(csock, &zero, 4, 0) != 4 ||
				send(csock, &zero, 4, 0) != 4 ||
				send(csock, pkt, sendlen, 0) != sendlen)
			{
				perror("doNetCommand: send");
			}
		}
	}
}

static const char serial_start[] = "/dev/ttyS";
static const char usb_start[] = "/dev/ttyUSB";

static void sockThread()
{
	// set up socket for receiving connections from central test server
	if((ssock = socket(PF_INET, SOCK_STREAM, 0)) == -1) {
		perror("sockThread: socket");
		return;
	}
	
	// Choose the port to listen on based on the name of the serial device
	// we're monitoring.
	if (0==strncmp(serial_start, serial_file, sizeof(serial_start)-1)) {
		sport = SERIAL_PORT + atoi(&serial_file[sizeof(serial_start)-1]);
	} else if (0==strncmp(usb_start, serial_file, sizeof(usb_start)-1)) {
		//printf("atoi(%s)\n", &serial_file[sizeof(usb_start)-1]);
		sport = USB_PORT + atoi(&serial_file[sizeof(usb_start)-1]);
	} else {
		return;
	}

	struct sockaddr_in addr;
	addr.sin_family = AF_INET;				// host byte order
	addr.sin_port = htons(sport);			// short, network byte order
	addr.sin_addr.s_addr = INADDR_ANY;
	memset(&(addr.sin_zero), '\0', 8); 		// zero the rest of the struct
	
	if (bind(ssock, (struct sockaddr *)&addr, sizeof(struct sockaddr_in)) == -1) {
		perror("sockThread: bind");
		if (close(ssock)) perror("close");
		return;
	}
	
	if (listen(ssock, 5) == -1) {
		perror("sockThread: listen");
		if (close(ssock)) perror("sockThread: close");
		return;
	}
	
	// attempt to connect to central test server
	do {
		if((csock = socket(PF_INET, SOCK_STREAM, 0)) == -1) {
			perror("sockThread: socket");
			break;
		}

		addr.sin_family = AF_INET;					// host byte order
		addr.sin_port = htons(TEST_PORT);			// short, network byte order
		struct hostent* entry = gethostbyname(hostname);
		if (!entry) {
			herror("sockThread: gethostbyname");
			if (close(csock)) perror("sockThread: close");
			csock = -1;
			break;
		}
		memcpy(&(addr.sin_addr), entry->h_addr_list[0], entry->h_length);
		memset(&(addr.sin_zero), '\0', 8); 		// zero the rest of the struct

		if (connect(csock, (struct sockaddr *)&addr, sizeof(struct sockaddr_in))) {
			perror("sockThread: connect");
			if (close(csock)) perror("sockThread: close");
			csock = -1;
			break;
		}
		
		if (doHandshake(csock, 0) == -1)  {
			if (close(csock)) perror("sockThread: close");
			csock = -1;
			break;
		}
		printf("Connected to test server at %s on port %d.\n", entry->h_name, TEST_PORT);
	} while(0);
	
	char buffer[1024];
	struct aqshell_pkt* pkt = (struct aqshell_pkt*)buffer;
	while (!shutdownMode)
	{
		// listen for commands
		while (csock != -1)
		{
			int ret;
			// make sure we are at the beginning of a command packet
			ret = dbg_recv(csock, buffer, 1, 0);
			if (ret <= 0) goto csock_error;
			if (ret == 1 && buffer[0] != SYNC_BYTE_1) continue;
			ret = dbg_recv(csock, buffer, 1, 0);
			if (ret <= 0) goto csock_error;
			if (ret == 1 && buffer[0] != SYNC_BYTE_2) continue;
			ret = dbg_recv(csock, buffer, 1, 0);
			if (ret <= 0) goto csock_error;
			if (ret == 1 && buffer[0] != SYNC_BYTE_3) continue;
			
			// Receive the command
			ret = dbg_recv(csock, pkt, sizeof(pkt->head), 0);
			if (ret < sizeof(pkt->head)) goto csock_error;
			if (pkt->head.length > 0) {
				ret = dbg_recv(csock, pkt->data, pkt->head.length, 0);
				if (ret < pkt->head.length) goto csock_error;
			}
			
			// And handle it
			doNetCommand(pkt);
			continue;
			
			csock_error:
			if (ret == -1)
				perror("sockThread: recv");
			if (close(csock))
				perror("sockThread: close");
			csock = -1;
		}
		printf("Connection closed.\n");
		if (shutdownMode)
			exit(0);
		
		// wait for test server to connect to us
		printf("Listening for connection on port %d.\n", sport);
		while (csock == -1)
		{
			int namelen = sizeof(struct sockaddr_in);
			csock = accept(ssock, (struct sockaddr *)&addr, &namelen);
			if (csock == -1) {
				perror("sockThread: accept");
				continue;
			}
			
			char* name = inet_ntoa(addr.sin_addr);
			struct hostent* entry = gethostbyaddr(name, strlen(name), AF_INET);
			if (!entry) {
				herror("gethostbyaddr");
			} else {
				name = entry->h_name;
			}
		
			if (doHandshake(csock, 1) == -1)  {
				if (close(csock)) perror("sockThread: close");
				csock = -1;
				continue;
			}
			printf("Connected to test server at %s on port %d.\n", name, sport);
		}
	}
}

void start()
{
	char* val = NULL;
	
	if (mos_arg_check("--log", (uint8_t**)&val) && val) {
		log_file = fopen(val, "w");
		if (!log_file)
			perror("start: fopen");
	}
	if (!mos_arg_check("--host", (uint8_t**)&hostname) || !hostname) {
		hostname = "localhost";
	}
	if (!mos_arg_check("--sdev", (uint8_t**)&serial_file) || !serial_file) {
		serial_file = "/dev/ttyS0";
	}
	if (mos_arg_check("--nopkts", NULL)) {
		printPkts = 0;
	} else {
		printPkts = 1;
	}
	if (mos_arg_check("--noprint", NULL)) {
		printing = 0;
		printPkts = 0;
	} else {
		printing = 1;
	}
	
	// Set up signal handlers
	// TODO make sure I did this right
	struct sigaction sa_old;
	struct sigaction sa_new;
	
	sa_new.sa_handler = sigHandler;
	sigemptyset(&sa_new.sa_mask);
	sa_new.sa_flags = 0;
	sigaction(SIGINT, &sa_new, &sa_old);
	sigaction(SIGTERM, &sa_new, &sa_old);
	sigaction(SIGALRM, &sa_new, 0);
	
	// Start threads for listening on serial port and TCP socket
	mos_thread_new(recvThread, 0, PRIORITY_NORMAL);
	mos_thread_new(sockThread, 0, PRIORITY_NORMAL);
}
