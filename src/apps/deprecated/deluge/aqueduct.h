#ifndef __AQUEDUCT_H__
#define __AQUEDUCT_H__

#define DEBUG 4
#if !defined(NODE_ID)
	#define NODE_ID 0
#endif
#include <stdarg.h>

#define min(X, Y)  ((X) < (Y) ? (X) : (Y))


#if DEBUG == 0
#define DEBUG_P1(...)
#define DEBUG_P2(...)
#define DEBUG_P3(...)
#define DEBUG_P4(...)
#else
#if DEBUG == 1
#define DEBUG_P1 printf
#define DEBUG_P2 printf
#define DEBUG_P3 printf
#define DEBUG_P4 printf
#endif
#if DEBUG == 2
#define DEBUG_P1(...)
#define DEBUG_P2 printf
#define DEBUG_P3 printf
#define DEBUG_P4 printf
#endif
#if DEBUG == 3
#define DEBUG_P1(...)
#define DEBUG_P2(...)
#define DEBUG_P3 printf
#define DEBUG_P4 printf
#endif
#if DEBUG == 4
#define DEBUG_P1(...)
#define DEBUG_P2(...)
#define DEBUG_P3(...)
#define DEBUG_P4 printf
#endif
#endif

void no_op(const char *format, ...) { }

/*
static uint16_t DEBUG_P(int lvl, const char *format, ...)
{
#if DEBUG != 0
	if (DEBUG <= lvl && DEBUG != 0) {
		va_list arg;
		const char *p = format;
		
		if(format == NULL)
			return -1;		
		va_start(arg, format);
		printf(p, arg);
		va_end(arg);
		
	}
#endif
	return 0;
}
*/
#if DEBUG >= 1
#define TERM_RESET() DEBUG_P4("\033[0m");
#define FG_BOLD() DEBUG_P4("\033[1m");
#define FG_BLACK() DEBUG_P4("\033[30m");
#define FG_RED() DEBUG_P4("\033[31m");
#define FG_GREEN() DEBUG_P4("\033[32m");
#define FG_YELLOW() DEBUG_P4("\033[33m");
#define FG_BLUE() DEBUG_P4("\033[34m");
#define FG_MAGENTA() DEBUG_P4("\033[35m");
#define FG_CYAN() DEBUG_P4("\033[36m");
#define FG_WHITE() DEBUG_P4("\033[37m");
#define BG_BLACK() DEBUG_P4("\033[40m");
#define BG_RED() DEBUG_P4("\033[41m");
#define BG_GREEN() DEBUG_P4("\033[42m");
#define BG_YELLOW() DEBUG_P4("\033[43m");
#define BG_BLUE() DEBUG_P4("\033[44m");
#define BG_MAGENTA() DEBUG_P4("\033[45m");
#define BG_CYAN() DEBUG_P4("\033[46m");
#define BG_WHITE() DEBUG_P4("\033[47m");
#define MARK() DEBUG_P1("\033[1m@LINE: %d\033[0m\n", __LINE__);
#endif
#define WARN(x) printf("\033[43m\033[1mWARNING: %s\033[0m\n", x);

/**
 * Assertion mechanism - all 3 blinking LEDs is a bad sign
 */
static inline void assert(bool condition, const char *message)
{
#if DEBUG >= 1
	int i = 0;
	if (!condition) {
BG_RED(); FG_BOLD();
		printf("Assertion Broken: %s\n", message);
		while (1) {
			mos_led_toggle(1);
			mos_led_toggle(2);
			mos_led_toggle(0);
			mos_thread_sleep(300);
			i++;
			if (i % 10 == 0) {
				printf("(repeat) Assertion Broken: %s\n", message);
				i = 0;
			}
		}
TERM_RESET();
	}
#endif
}


#define UNLOCK_SLEEP_LOCK(l, t) mos_mutex_unlock(l); mos_thread_sleep(t); mos_mutex_lock(l);
#define UNLOCK_SLEEP_LOCK2(l, l2, t) mos_mutex_unlock(l); mos_mutex_unlock(l2); mos_thread_sleep(t); mos_mutex_lock(l); mos_mutex_lock(l2);
#define AQUEDUCT_LISTEN_PORT 40 + NODE_ID
#define BASE_STATION_ID 0

#define STATE_OFF 0
#define STATE_ON  1

 //#define kbus2sec(x)     (x * 1024 * 10 ^ -6)
 //#define sec2kbus(x)     (x / 0.001024)

/** Aqueduct constants */

/* Total number of images that we are storing */
#define AQUEDUCT_MAX_IMAGES_STORED 1
/* Total number of unique IDs available for images */
#define AQUEDUCT_MAX_UNIQUE_VERSION_IDS 4
/* Total number of unique IDs available for images */
#define AQUEDUCT_MAX_UNIQUE_IMAGE_IDS 2

#define AQUEDUCT_MAX_SUMMARY_IMAGES_LOGGED AQUEDUCT_MAX_UNIQUE_IMAGE_IDS * AQUEDUCT_MAX_UNIQUE_VERSION_IDS

/* Defined in Deluge paper */
#define AQUEDUCT_K              2
#define AQUEDUCT_MIN_DATA_RATE  ((uint32_t)9)
#define AQUEDUCT_OMEGA          ((uint32_t)8)
#define AQUEDUCT_T_R_MSECONDS   ((uint32_t)500) // two seconds
// Estimate the time it takes to transmit 1 data packet at 250 kbps plus a 10 ms backoff
//#define AQUEDUCT_T_TX_MSECONDS	(((((uint32_t)sizeof(aqueduct_foot_data) * (uint32_t)8)) / (uint32_t)250) + 100)
#define AQUEDUCT_T_TX_MSECONDS	(((((uint32_t)sizeof(aqueduct_foot_data) * (uint32_t)8)) / (uint32_t)250) + 100)
#define AQUEDUCT_TX_DELAY_INC	1
#define AQUEDUCT_TX_DELAY_MAX	40
#define AQUEDUCT_TX_DELAY_MIN	0

#define AQUEDUCT_LAMBDA ((uint8_t)2)

#ifdef ARCH_MICRO
// 2 seconds
#define AQUEDUCT_T_L_SECONDS    ((uint32_t)2)
#define AQUEDUCT_T_L    (AQUEDUCT_T_L_SECONDS * (uint32_t)TICKS_PER_SEC)
// 256 seconds
#define AQUEDUCT_T_H_SECONDS    ((uint32_t)60)
#define AQUEDUCT_T_H    (AQUEDUCT_T_H_SECONDS * (uint32_t)TICKS_PER_SEC)
#endif

#define AQUEDUCT_MAX_NEIGHBORS	8
#define AQUEDUCT_MAX_PACKETS_PER_PAGE 48
#define AQUEDUCT_MAX_PAGES     48

/* measured in terms of bytes per packet */
#define AQUEDUCT_DATA_PAYLOAD_SIZE   23

/** Aqueduct state-machine states */
#define AQUEDUCT_MAINTAIN  1
#define AQUEDUCT_RESTART_MAINTAIN 2
#define AQUEDUCT_RX        3
#define AQUEDUCT_TX        4
#define AQUEDUCT_FWD       5
// Don't change state if we see this
#define AQUEDUCT_IGNORE    6

/** Messages */
#define AQUEDUCT_PACKET_SUMMARY 1
#define AQUEDUCT_PACKET_PROFILE 2
#define AQUEDUCT_PACKET_REQUEST 3
#define AQUEDUCT_PACKET_DATA    4
#define AQUEDUCT_PACKET_COMMAND 5

#define AQUEDUCT_VALUE_NOT_INITIALIZED AQUEDUCT_MAX_PAGES + 1
#define AQUEDUCT_NULL_PAGE ((int8_t)-1)
typedef struct {
	uint8_t image_ID;
	uint8_t version;
	int8_t high_page;
	int8_t absolute_high;
	uint8_t age[AQUEDUCT_MAX_PAGES];
	mos_mutex_t rw_lock;
	uint8_t dtb;

	uint8_t aqueduct_state;
	uint32_t maintain_round_duration_sec;  // measured in seconds!
	uint16_t node_causing_rx;   //ID of the node that caused RX mode

	uint8_t equivolant_summaries_heard;
	uint8_t outdated_summaries_heard;
	uint8_t newer_summaries_heard;

	uint8_t equivolant_profiles_heard;
	uint8_t outdated_profiles_heard;
	uint8_t newer_profiles_heard;

	uint8_t request_heard;
	uint8_t data_heard;

	int8_t requested_page;
	int8_t goal_page;             // Highest page available from neighbor
	uint8_t pages_needed[AQUEDUCT_MAX_PAGES];
	uint16_t packets_needed[3];    // Filled in left-to-right (i.e. leftmost slot of element 0 is packet 0)

	int8_t page_to_be_serviced;
	uint16_t packets_to_be_serviced[3];  // Filled in right-to-left (i.e. rightmost slot of element 0 is packet 0)

	uint32_t last_request_sent_timestamp;
	uint32_t request_between_0_and_high_page_timestamp;
//	uint32_t last_request_to_me_timestamp;      // Unused?
	uint32_t last_request_overheard_timestamp;
	uint32_t data_between_0_and_high_page_plus_one_timestamp;  // See rule M.5
	uint32_t last_data_overheard_timestamp;
//	uint32_t last_data_to_me_timestamp;         // Unused?
	uint16_t data_to_me_count;
} aqueduct_app;


typedef struct {
	uint8_t image_ID;   // The image's unique ID
	uint8_t version;    // The current image version number
	int8_t high_page;   // MEMBER: Highest page available, FORWARD: Highest page available AT THE ROOT OF THE SPT
	uint8_t dtb;        // MEMBER: 0, FORWARD: Distance to root of the SPT
} aqueduct_summary;

typedef struct {
	uint8_t image_ID;
	uint8_t version;
	int8_t goal_page;    // Highest page available from sender
	uint8_t absolute_high;  // The high page for this image (may not all be available)
	uint8_t age[AQUEDUCT_MAX_PAGES];
} aqueduct_profile;

typedef struct {
	uint8_t image_ID;           // Requested image
	uint8_t version;            // The outdated version
	int8_t page;                // Requested page of the new version
	uint16_t packets_needed[3]; // Bit vector indicating which packets of the page are needed
} aqueduct_request;

typedef struct {
	uint8_t image_ID;   // Image ID
	uint8_t version;    // Version number of the updated version
	int8_t page;        // Page number
	uint8_t packet;     // Packet number
	uint8_t payload[AQUEDUCT_DATA_PAYLOAD_SIZE]; // The actual data
} aqueduct_data;

typedef struct {
//#ifdef AQUEDUCT_SYMMETRIC_LINKS
//	uint16_t neighbors[AQUEDUCT_NEIGHBOR_COUNT];
//#endif
	aqueduct_summary summary;
	uint16_t sender_ID;      // ID of sender
	uint8_t type;     // AQUEDUCT_PACKET_SUMMARY
} __attribute__ ((packed)) aqueduct_foot_summary;

typedef struct {
//	uint32_t codeSize;  // Size in bytes of the current image
//	uint16_t crc;       // CRC of the whole image
//	uint8_t version;    // Version number of the image
//	int8_t goalPage;    // Highest page in the image 
	aqueduct_profile profile;
	uint16_t sender_ID;        // ID of sender
	uint8_t type;       // AQUEDUCT_PACKET_PROFILE
} __attribute__ ((packed)) aqueduct_foot_profile;

typedef struct {
	aqueduct_request request;
	uint16_t sender_ID;         // ID of sender
	uint16_t destination_ID;    // ID of the node this request is sent to
	uint8_t type;        // AQUEDUCT_PACKET_REQUEST
} __attribute__ ((packed)) aqueduct_foot_request;

// This footer follows AQUEDUCT_PACKET_SIZE bytes of data
typedef struct {
	aqueduct_data data;
	uint16_t crc;       // Data's CRC Value
	uint16_t sender_ID; // ID of sender
	uint8_t type;       // AQUEDUCT_PACKET_DATA
} __attribute__ ((packed)) aqueduct_foot_data;

#endif

