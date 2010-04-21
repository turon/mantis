#include <inttypes.h>
#include <stdbool.h>
#include "msched.h"
#include "led.h"
#include "clock.h"
#include "printf.h"
#include "com.h"
#include "node_id.h"
#include "mutex.h"
#include <stdlib.h>
#include "string.h"
#include "crc.h"
#include "sem.h"

#include "aqueduct.h"

/* Network protocol headers */
//#include "mst.h"
//#include "flood.h"
//#include "simple_proto.h"

#if defined(PLATFORM_MICA2)
	#include "cc1000.h"
#elif defined(PLATFORM_MICAZ) || defined(PLATFORM_TELOSB)
	#include "cc2420.h"
#endif

/****** Begin Helper Functions *******/
#define GET_AQUEDUCT_PACKET_TYPE(buffer) buffer->data[buffer->size - 1]
/****** End Helper Functions *******/

static uint16_t my_ID;
static comBuf my_buf;

mos_mutex_t my_rw_lock; /* Keeps the listening thread in check */
mos_sem_t my_timeout_sem;

static uint8_t my_app_count;
static aqueduct_app my_app_memory[AQUEDUCT_MAX_IMAGES_STORED];

static void aqueduct_tx(aqueduct_app *app);

static void change_state(aqueduct_app *app, uint8_t new_state);
static uint32_t random_time(uint32_t low, uint32_t high);
static void reset_logs(aqueduct_app *app);
static void send_request(aqueduct_app *app, uint16_t id_S);
static void aqueduct_rx(aqueduct_app *app);

static bool previously_received_request(aqueduct_app *app);
static bool previously_received_data_packet(aqueduct_app *app);
static void handle_summary(comBuf *buffer);
static void handle_profile(comBuf *buffer);
void handle_request(comBuf *buffer);
static void parse_com_buf(aqueduct_app *app, comBuf *buffer);
// static comBuf *aqueduct_recv(uint8_t port);
//static void timeout_alarm(void);
// static void listen(aqueduct_app *app, uint32_t timeout_ms);
static void aqueduct_send(comBuf *buffer, uint8_t port);
static void send_summary(aqueduct_app *app);
static uint32_t adjust_round_duration(bool inconsistency, uint32_t current_round_duration);
static void send_profile(aqueduct_app *app);
static void aqueduct_maintain(aqueduct_app *app);

void dump_uint8_t_array(int array_length, uint8_t *array)
{
	#if DEBUG >= 1
	int i = 0;
	DEBUG_P1("[");
	while (i < (array_length - sizeof(uint8_t))) {
		if (array_length - i > 32) {
			DEBUG_P1("%C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, ",
				array[i], array[i + 1], array[i + 2], array[i + 3],
				array[i + 4], array[i + 5], array[i + 6], array[i + 7],
				array[i + 8], array[i + 9], array[i + 10], array[i + 11],
				array[i + 12], array[i + 13], array[i + 14], array[i + 15],
				array[i + 16], array[i + 17], array[i + 18], array[i + 19],
				array[i + 20], array[i + 21], array[i + 22], array[i + 23],
				array[i + 24], array[i + 25], array[i + 26], array[i + 27],
				array[i + 28], array[i + 29], array[i + 30], array[i + 31]);
			i += 32;
		} else if (array_length - i > 16) {
			DEBUG_P1("%C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, %C, ", array[i], array[i + 1], array[i + 2], array[i + 3],
				array[i + 4], array[i + 5], array[i + 6], array[i + 7],
				array[i + 8], array[i + 9], array[i + 10], array[i + 11],
				array[i + 12], array[i + 13], array[i + 14], array[i + 15]);
			i += 16;
		} else if (array_length - i > 8) {
			DEBUG_P1("%C, %C, %C, %C, %C, %C, %C, %C, ", array[i], array[i + 1], array[i + 2], array[i + 3],
				array[i + 4], array[i + 5], array[i + 6], array[i + 7]);
			i += 8;
		} else if (array_length - i > 4) {
			DEBUG_P1("%C, %C, %C, %C, ", array[i], array[i + 1], array[i + 2], array[i + 3]);
			i += 4;
		} else if (array_length - i > 2) {
			DEBUG_P1("%C, %C, ", array[i], array[i + 1]);
			i += 2;
		} else {
			DEBUG_P1("%C, ", array[i]);
			i++;
		}
	}

	if (array_length > 0) {
		DEBUG_P1("%C", array[i]);
	}
	DEBUG_P1("]");
	#endif
}

/**
 * @brief Prints out the contents of the aqueduct_app struct
 */
/*
void dump_app(aqueduct_app *app)
{
	DEBUG_P1(
"***** APP DUMP *****\n\
* my_ID = %d\n\
* image_ID = %C\n\
* version = %C\n\
* high_page = %C\n\
* aqueduct_state = %C\n\
* maintain_round_duration_sec = %l\n\
* node_causing_rx = %d\n\
* equivolant_summaries_heard = %C\n\
* outdated_summaries_heard = %C\n\
* newer_summaries_heard = %C\n\
* equivolant_profiles_heard = %C\n\
* outdated_profiles_heard = %C\n\
* newer_profiles_heard = %C\n\
* requested_page = %C\n\
* goal_page = %C\n\
* page_to_be_serviced = %C\n\
* packets_to_be_serviced = %d\n\
* last_request_sent_timestamp = %l\n\
* last_request_to_me_timestamp = %l\n\
* last_request_overheard_timestamp = %l\n\
* last_data_overheard_timestamp = %l\n\
* last_data_to_me_timestamp = %l\n\
* data_to_me_count = %C\n",
	          my_ID, app->image_ID, app->version, app->high_page, 
	          app->aqueduct_state, app->maintain_round_duration_sec, app->node_causing_rx,
	          app->equivolant_summaries_heard, app->outdated_summaries_heard,
	          app->newer_summaries_heard, app->equivolant_profiles_heard,
	          app->outdated_profiles_heard, app->newer_profiles_heard, app->requested_page,
	          app->goal_page, app->page_to_be_serviced,
	          app->packets_to_be_serviced, app->last_request_sent_timestamp,
	          app->last_request_to_me_timestamp, app->last_request_overheard_timestamp,
	          app->last_data_overheard_timestamp, app->last_data_to_me_timestamp,
	          app->data_to_me_count);
	DEBUG_P1("* age = "); dump_uint8_t_array(sizeof(app->age), app->age);
	DEBUG_P1("\n");
	DEBUG_P1("* pages_needed = "); dump_uint8_t_array(sizeof(app->pages_needed), (uint8_t*)&app->pages_needed);
	DEBUG_P1("\n");
	DEBUG_P1("* packets_needed = "); dump_uint8_t_array(sizeof(app->packets_needed), (uint8_t*)&app->packets_needed);
	DEBUG_P1("\n");
	DEBUG_P1("******************\n");
}
*/

/**
 * @brief Prints out the contents of the aqueduct_request struct
 */
void dump_req(aqueduct_request *req)
{
	DEBUG_P1(
"***** REQUEST DUMP *****\n\
* image_ID = %C\n\
* version = %C\n\
* page = %C\n",
	          req->image_ID, req->version, req->page);
	DEBUG_P1("* packets_needed = ");
	dump_uint8_t_array(sizeof(req->packets_needed), (uint8_t*)&req->packets_needed);
	DEBUG_P1("\n");
	DEBUG_P1("******************\n");
}

/**
 * @brief Prints out the contents of the aqueduct_request struct
 */
void dump_data(aqueduct_data *data)
{
	DEBUG_P1(
"***** DATA DUMP *****\n\
* image_ID = %C\n\
* version = %C\n\
* page = %C\n\
* packet = %C\n",
	          data->image_ID, data->version, data->page, data->packet);
	DEBUG_P1("* payload = ");
	dump_uint8_t_array(sizeof(data->payload), (uint8_t*)&data->payload);
	DEBUG_P1("\n");
	DEBUG_P1("* CRC = %d\n", crc_compute((uint8_t *)data, sizeof(aqueduct_data)));
	DEBUG_P1("******************\n");
}


/**
 * @brief Changes the state (not sure if this is really that useful)
 */
static void change_state(aqueduct_app *app, uint8_t new_state)
{
FG_BOLD();
	DEBUG_P4("====================> PROPOSING NEW STATE = %C <===================\n", new_state);
TERM_RESET();
	assert(((app->aqueduct_state == AQUEDUCT_RX && new_state == AQUEDUCT_MAINTAIN) ||
			(app->aqueduct_state == AQUEDUCT_TX && new_state == AQUEDUCT_MAINTAIN) ||
			(app->aqueduct_state == AQUEDUCT_FWD && new_state == AQUEDUCT_MAINTAIN) ||
			(app->aqueduct_state == AQUEDUCT_RESTART_MAINTAIN && new_state == AQUEDUCT_MAINTAIN) ||
			(app->aqueduct_state == AQUEDUCT_MAINTAIN && new_state == AQUEDUCT_RESTART_MAINTAIN) ||
			(app->aqueduct_state == AQUEDUCT_MAINTAIN && (new_state == AQUEDUCT_RX || new_state == AQUEDUCT_TX || new_state == AQUEDUCT_MAINTAIN))),
	"Illegal state change");
	app->aqueduct_state = new_state;
}

/**
 * @brief Returns a random uint32_t between low and high
 */
static uint32_t random_time(uint32_t low_ms, uint32_t high_ms)
{
	return low_ms + rand() % (high_ms - low_ms);
}

/**
 * @brief Initalizes the summary and profile logs
 */
static void reset_logs(aqueduct_app *app)
{
	DEBUG_P1("Resetting logs\n");
	app->equivolant_summaries_heard = 0;
	app->outdated_summaries_heard = 0;
	app->newer_summaries_heard = 0;

	app->equivolant_profiles_heard = 0;
	app->outdated_profiles_heard = 0;
	app->newer_profiles_heard = 0;

	app->data_heard = 0;
	app->request_heard = 0;
	app->data_to_me_count = 0;

////	app->last_request_sent_timestamp = 0;
////	app->request_between_0_and_high_page_timestamp = 0;
//	app->last_request_to_me_timestamp = 0;
////	app->last_request_overheard_timestamp = 0;
////	app->data_between_0_and_high_page_plus_one_timestamp = 0;
////	app->last_data_overheard_timestamp = 0;
//	app->last_data_to_me_timestamp = 0;
	
}

/**
 * @brief: Returns app->goal_page if no more pages needed.  Assumes this has been set by handle_profile()
 */
static int8_t get_next_needed_page(aqueduct_app *app)
{
	DEBUG_P2("get_next_needed_page(): begin\n");
	DEBUG_P1("get_next_needed_page(): app->goal_page = %C\n", app->goal_page);
	DEBUG_P1("get_next_needed_page(): app->requested_page = %C\n", app->requested_page);
	DEBUG_P1("get_next_needed_page(): app->high_page = %C\n", app->high_page);
	assert(app->goal_page != AQUEDUCT_NULL_PAGE, "get_next_needed_page(): app->goal_page is not initialized\n");
	assert(app->absolute_high != AQUEDUCT_NULL_PAGE, "get_next_needed_page(): app->absolute_high is not initialized\n");
	uint8_t i, result;
	
	for (i = 0; i <= app->absolute_high; i++) {
		if (app->pages_needed[i]) {
			DEBUG_P1("get_next_needed_page(): page %C needed\n", i);
			if (i != app->requested_page) {
				/* set all packets to "needed" (i.e. 1) */
				app->packets_needed[0] = (uint16_t)0xFFFFFFFF;
				app->packets_needed[1] = (uint16_t)0xFFFFFFFF;
				app->packets_needed[2] = (uint16_t)0xFFFFFFFF;
			}
			result = i;
			break;
		}
	}
	result = i;
	DEBUG_P1("get_next_needed_page(): return %C\n", result);
	DEBUG_P2("get_next_needed_page(): end\n");
	return result;
}

/**
 * @brief Send Request
 */
static void send_request(aqueduct_app *app, uint16_t destination_ID)
{
	DEBUG_P2("send_request(destination: %d): begin\n", destination_ID);

	assert(app->goal_page != AQUEDUCT_NULL_PAGE, "send_request(): app->goal_page is not set!\n");
//	assert(app->requested_page != AQUEDUCT_NULL_PAGE, "send_request(): app->requested_page is not set!\n");

	my_buf.size = 14;//sizeof(aqueduct_foot_request);
	my_buf.data[0] = app->image_ID;                                          //1
	my_buf.data[1] = app->version;                                           //1
	DEBUG_P1("send_request(): app->image_ID=%C, app->version=%C, app->high_page = %C\n", app->image_ID, app->version, app->high_page);
	app->requested_page = get_next_needed_page(app);
	my_buf.data[2] = app->requested_page;                                    //1

	memcpy(&my_buf.data[3], &app->packets_needed, sizeof(uint16_t) * 3);     //6
	memcpy(&my_buf.data[9], &my_ID, sizeof(uint16_t));                       //2
	memcpy(&my_buf.data[11], &destination_ID, sizeof(uint16_t));             //2

	/* TODO: I think this should really be handled by aqueduct_send and taken out of these structs */
	my_buf.data[13] = AQUEDUCT_PACKET_REQUEST;                               //1
	app->last_request_sent_timestamp = mos_get_realtime();

	mos_led_toggle(0);
	aqueduct_send(&my_buf, AQUEDUCT_LISTEN_PORT);
	DEBUG_P2("send_request(destination: %d): end\n", destination_ID);
}
/**
 * @brief Returns TRUE if requested_page is incomplete
 */
static bool requested_page_incomplete(aqueduct_app *app)
{
	return !((app->packets_needed[0] | 0) == 0 && (app->packets_needed[1] | 0) == 0 && (app->packets_needed[2] | 0) == 0);
}

/**
 * @brief A node in RX state is responsible for actively requesting the remaining packets required to complete the requested page
 */
static void aqueduct_rx(aqueduct_app *app)
{
	DEBUG_P2("aqueduct_rx(): begin\n");
	assert(app->aqueduct_state == AQUEDUCT_RX, "aqueduct_rx(): not in state AQUEDUCT_RX");
	uint16_t responsible_node = app->node_causing_rx;
	uint8_t retry_counter = 0; /* The total number of retries before we quit this loop */

	/*
	 * After a node makes a request, it waits for a response and makes subsequent requests if some of
	 * the data was lost in the progress.  Nodes delay subsequent requests until they detect a period
	 * of silence equal to omega packet trasmit times to help ensure the completion of data transmissions
	 * before requests are made.  The value of omega is chosen such that simply dropping a couple of
	 * packets is not detected as a period of silence.
	 */
	DEBUG_P4("aqueduct_rx(): initial request send\n");
	send_request(app, responsible_node);
	
	while (1) {
		uint32_t timeout;
		/* NOTE: this is multiplied by 100000 so we don't have to mess with floating point */
		uint32_t data_receive_rate = 0;
		uint32_t prev_neighbor_sent_request_timestamp = app->request_between_0_and_high_page_timestamp;

		DEBUG_P1("aqueduct_rx(): AQUEDUCT_T_TX_MSECONDS = %l\n", AQUEDUCT_T_TX_MSECONDS);
		timeout = (uint32_t)(AQUEDUCT_OMEGA * AQUEDUCT_T_TX_MSECONDS + random_time(0, AQUEDUCT_T_R_MSECONDS));
		DEBUG_P1("aqueduct_rx(): timeout = %l\n", timeout);
		
		reset_logs(app);
		//listen(app, timeout);
		
		UNLOCK_SLEEP_LOCK(&app->rw_lock, timeout);
		//UNLOCK_SLEEP_LOCK2(&app->rw_lock, &app->rw_lock2, timeout);
		//mos_mutex_unlock(&app->rw_lock);
		//mos_thread_sleep(timeout);
		//mos_mutex_lock(&app->rw_lock);

		data_receive_rate = ((uint32_t)app->data_to_me_count * (uint32_t)100000) / timeout;
		DEBUG_P1("aqueduct_rx(): data_receive_rate = %l\n", data_receive_rate);
		if (data_receive_rate < AQUEDUCT_MIN_DATA_RATE) {
			retry_counter++;
		}

		/**
		 * R.1
		 *
		 * After not receiving a request or a data packet for time t = w * T_{tx} + r,
		 * where r is a random value in the range T_r, transmit a request to node S.
		 */
//		if (app->last_request_to_me_timestamp == 0 && app->last_data_to_me_timestamp == 0) {
		if (prev_neighbor_sent_request_timestamp == app->request_between_0_and_high_page_timestamp && app->data_to_me_count == 0) {
			DEBUG_P4("aqueduct_rx(): another request send\n");
			MARK();
			send_request(app, responsible_node);
		}

		/**
		 * R.2
		 *
		 * After LAMBDA requests with a packet reception rate of ALPHA' <. ALPHA,
		 * transition to MAINTAIN even if page GAMMA + 1 is incomplete.
		 */
		if (retry_counter >= AQUEDUCT_LAMBDA) {
			change_state(app, AQUEDUCT_MAINTAIN);
			break;
		/**
		 * R.3
		 *
		 * If all packets for page p are received and data for page GAMMA + 1 passes the CRC, mark
		 * the page as complete and transition to MAINTAIN.
		 */
		} else if (!requested_page_incomplete(app)) {
			MARK();
			/* We've received all packets for this page! PAGE COMPLETE */
			app->pages_needed[app->requested_page] = 0;
//			app->age[app->requested_page]++;
			app->high_page = app->requested_page;
			MARK();

			// memset(&app->packets_needed, 0, sizeof(uint16_t) * 3); /* this should always be true */
			BG_BLUE(); FG_WHITE(); FG_BOLD();
			DEBUG_P4("aqueduct_rx(): *******************************************************************\n");
			DEBUG_P4("aqueduct_rx(): ****************** PAGE %C COMPLETED!!!  **************************\n", app->high_page);
			DEBUG_P4("aqueduct_rx(): *******************************************************************\n");
			TERM_RESET();
			DEBUG_P1("aqueduct_rx(): app->absolute_high = %C\n", app->absolute_high);
			/* Check if update is finished */
			if (app->high_page == app->absolute_high) {
				BG_RED(); FG_WHITE(); FG_BOLD();
				DEBUG_P4("aqueduct_rx(): *******************************************************************\n");
				DEBUG_P4("aqueduct_rx(): ************ UPDATE COMPLETED Now version: %C *********************\n", app->version);
				DEBUG_P4("aqueduct_rx(): *******************************************************************\n");
				TERM_RESET();
				int8_t z;
				DEBUG_P1("aqueduct_rx(): checking pages:\n");
				for (z = 0; z < AQUEDUCT_MAX_PAGES; z++) {
					DEBUG_P1("%d...", z);
					assert(app->pages_needed[z] == 0, "aqueduct_rx(): page still needed!");
				}
//				app->goal_page = AQUEDUCT_NULL_PAGE;
			}

			app->requested_page = get_next_needed_page(app);
			change_state(app, AQUEDUCT_MAINTAIN);
//			send_summary(app);
			break;
		} /** end R.3 */

		DEBUG_P1("aqueduct_rx(): app->data_to_me_count = %d\n", app->data_to_me_count);
//		DEBUG_P1("aqueduct_rx(): app->last_request_to_me_timestamp = %l\n", app->last_request_to_me_timestamp);
//		DEBUG_P1("aqueduct_rx(): app->last_data_to_me_timestamp = %l\n", app->last_data_to_me_timestamp);

		DEBUG_P1("aqueduct_rx(): retry_counter = %C\n", retry_counter);
	}

	DEBUG_P2("aqueduct_rx(): end\n");
}

/**
 * @brief Rule M.5(i).
 * 
 * TRUE if a request for a page p <= GAMMA was previously received within time
 * t = 2 * T_{M, I}.  Helps synchronize the process; nodes that fall behind can catch up.
 * 
 * @param app the app we're testing
 */
static bool previously_received_request(aqueduct_app *app)
{
	FG_MAGENTA(); FG_BOLD();
	DEBUG_P1("previously_received_request(): %l + %l * 2 >= %l --> %C\n", app->request_between_0_and_high_page_timestamp,
		                                                                  app->maintain_round_duration_sec * 1000,
		                                                                  mos_get_realtime(),
		                                                                  app->request_between_0_and_high_page_timestamp + (app->maintain_round_duration_sec * 1000) * 2 >= mos_get_realtime());
	TERM_RESET();

	if (app->request_between_0_and_high_page_timestamp == 0) {
		return FALSE;
	} else {
		return (app->request_between_0_and_high_page_timestamp + (app->maintain_round_duration_sec * 1000) * 2 >= mos_get_realtime());
	}
}

/**
 * @brief Rule M.5(ii).
 *
 * TRUE if a data packet for page p <= GAMMA + 1 was previously received within time t = T_{M, I}
 * In other words, nodes may not issue a request if a transfer of a page is in progress.
 *
 * @param app the app we're testing
 */
static bool previously_received_data_packet(aqueduct_app *app)
{
	FG_MAGENTA(); FG_BOLD();
	DEBUG_P1("previously_received_data_packet(): %l + %l >= %l --> %C\n", app->data_between_0_and_high_page_plus_one_timestamp,
		                                                                  app->maintain_round_duration_sec * 1000,
		                                                                  mos_get_realtime(),
		                                                                  app->data_between_0_and_high_page_plus_one_timestamp + (app->maintain_round_duration_sec * 1000) >= mos_get_realtime());
	TERM_RESET();

	if (app->data_between_0_and_high_page_plus_one_timestamp == 0) {
		return FALSE;
	} else {
		return (app->data_between_0_and_high_page_plus_one_timestamp + (app->maintain_round_duration_sec * 1000) >= mos_get_realtime());
	}
}

/**
 * @brief Count the summaries we've seen -- part of rule M.1.
 * @NOTE  The list used in this gets reset after the round times out.
 */
static void handle_summary(comBuf *buffer)
{
	assert(GET_AQUEDUCT_PACKET_TYPE(buffer) == AQUEDUCT_PACKET_SUMMARY, "handle_summary(): Not a summary packet.");
	int i;

	/* TODO: this casting can become unreliable if the data struct changes (due to word alignment) */
	aqueduct_foot_summary fs;
	fs.summary.image_ID = buffer->data[0];
	fs.summary.version = buffer->data[1];
	fs.summary.high_page = buffer->data[2];
	fs.summary.dtb = buffer->data[3];
	fs.sender_ID = ((uint16_t)(buffer->data[4])) | (((uint16_t)buffer->data[5]) >> 8);
	fs.type = buffer->data[6];
	aqueduct_foot_summary *foot_summary = &fs;
	aqueduct_summary *summary = &(foot_summary->summary);
	aqueduct_app *app;

	for (i = 0; i < my_app_count; i++) {
		app = &my_app_memory[i];
		if (app->aqueduct_state != AQUEDUCT_MAINTAIN) {
			DEBUG_P1("handle_summary(): Ignoring summary because state = %C\n", app->aqueduct_state);
		} else {
			MARK();
			if (app->image_ID == summary->image_ID && app->version == summary->version && app->high_page == summary->high_page) {
				DEBUG_P1("handle_summary(): Summary same\n");
				app->equivolant_summaries_heard++;
			} else if (app->image_ID == summary->image_ID && app->version == summary->version && app->high_page > summary->high_page) {
				DEBUG_P1("handle_summary(): Summary outdated (app->high_page > sum->high_page)\n");
				app->outdated_summaries_heard++;
			} else if (app->image_ID == summary->image_ID && app->version > summary->version) {
				DEBUG_P1("handle_summary(): Summary outdated (app->vers > sum->vers)\n");
				app->outdated_summaries_heard++;
			} else if (app->image_ID == summary->image_ID && app->version < summary->version) {
				DEBUG_P1("handle_summary(): Summary newer (app->vers < sum->vers)\n");
				app->newer_summaries_heard++;
			/**
			 * Rule M.5
			 *
			 * On receiving an advertisement with v' = v and H' > H, transition to RX
			 */
			} else if (app->image_ID == summary->image_ID && app->version == summary->version && app->high_page < summary->high_page) {
				DEBUG_P1("handle_summary(): I haven't received all my pages\n");
				app->newer_summaries_heard++;
				/* we need to know the goal_page to confirm end of data */
				if (app->goal_page != AQUEDUCT_NULL_PAGE) {
	//				assert(app->goal_page != AQUEDUCT_NULL_PAGE, "handle_summary(): app->goal_page is not initialized.\n");
					if (!previously_received_request(app) && !previously_received_data_packet(app)) {
						DEBUG_P1("handle_summary(): node causing rx = %d\n", foot_summary->sender_ID);
						app->node_causing_rx = foot_summary->sender_ID;
						/* We're still within the context of AQUEDUCT_MAINTAIN -- this can be undone when listen() returns */
						change_state(app, AQUEDUCT_RX);
					}
				} else {
					WARN("handle_summary(): app->goal_page == AQUEDUCT_NULL_PAGE\n");
					DEBUG_P1("handle_summary(): app->summary = %C\n", summary->high_page);
					DEBUG_P1("handle_summary(): app->high_page = %C\n", app->high_page);
					DEBUG_P1("handle_summary(): app->version = %C\n", app->version);
				}
			} else {
				WARN("handle_summary(): should not reach here -- probably corrupted packet\n");
			}
		}
	}
}

/**
 * @brief Compares the age of the pages from the new profile to the old one
 */
static void compare_profiles(aqueduct_app *app, uint8_t new_age[], int8_t goal_page)
{
	assert(goal_page != AQUEDUCT_NULL_PAGE, "compare_profiles(): goal_page is not set\n");
	int i;
	for (i = 0; i <= goal_page; i++) {
		if (new_age[i] != app->age[i]) {
			app->pages_needed[i] = TRUE;
		}
	}
}

/**
 * @brief Logs the profile received -- M.4 requires this
 */
static void handle_profile(comBuf *buffer)
{
	DEBUG_P2("handle_profile(): begin\n");
	assert(GET_AQUEDUCT_PACKET_TYPE(buffer) == AQUEDUCT_PACKET_PROFILE, "handle_profile(): Not a profile packet.");
	aqueduct_foot_profile fp;
	aqueduct_foot_profile *foot_profile = &fp;
	fp.profile.image_ID = buffer->data[0];
	fp.profile.version = buffer->data[1];
	fp.profile.goal_page = buffer->data[2];
	fp.profile.absolute_high = buffer->data[3];
	memcpy(&fp.profile.age, &buffer->data[4], sizeof(uint8_t) * AQUEDUCT_MAX_PAGES);
	fp.sender_ID = buffer->data[4 + sizeof(uint8_t) * AQUEDUCT_MAX_PAGES];
	fp.sender_ID = fp.sender_ID | (buffer->data[4 + sizeof(uint8_t) * AQUEDUCT_MAX_PAGES] >> 8);
	fp.type = buffer->data[5 + sizeof(uint8_t) * AQUEDUCT_MAX_PAGES];
	aqueduct_profile *profile = &(foot_profile->profile);

//	assert(profile->goal_page != AQUEDUCT_NULL_PAGE, "handle_profile(): profile->goal_page is not set!\n");

	int i;

	for (i = 0; i < my_app_count; i++) {
		aqueduct_app *app = &my_app_memory[i];
		if (app->aqueduct_state != AQUEDUCT_MAINTAIN) {
			DEBUG_P1("handle_profile(): Ignoring profile because we're not in AQUEDUCT_MAINTAIN\n");
			continue;
		} else {
			if (app->image_ID == profile->image_ID && app->version == profile->version && app->high_page == profile->goal_page) {
				DEBUG_P1("handle_profile(): profiles equal\n");
				app->equivolant_profiles_heard++;
			} else if (app->image_ID == profile->image_ID && app->version == profile->version && app->high_page > profile->goal_page) {
				DEBUG_P1("handle_profile(): outdated profile heard (my high_page > goal_page)\n");
				app->outdated_profiles_heard++;
//			} else if (app->image_ID == profile->image_ID && app->version == profile->version && app->high_page < profile->goal_page) {
//				DEBUG_P1("handle_profile(): newer profile heard (my high_page < goal_page)\n");
//				app->newer_profiles_heard++;
			} else if (app->image_ID == profile->image_ID && app->version > profile->version) {
				DEBUG_P1("handle_profile(): outdated profile heard (my version > profile version)\n");
				app->outdated_profiles_heard++;
			} else if ((app->image_ID == profile->image_ID && app->version < profile->version) ||
				       (app->image_ID == profile->image_ID && app->version == profile->version && app->high_page < profile->goal_page)) {

				if (app->image_ID == profile->image_ID && app->version < profile->version) {
					DEBUG_P1("handle_profile(): newer profile heard (my version < profile version)\n");
				} else {
					DEBUG_P1("handle_profile(): newer profile heard (my high page < goal page)\n");
				}

				app->newer_profiles_heard++;
				/* prepare reception of new version */
				compare_profiles(app, profile->age, profile->absolute_high);
				app->version = profile->version;
				app->goal_page = profile->goal_page;
				app->absolute_high = profile->absolute_high;
				app->requested_page = get_next_needed_page(app);
				app->high_page = app->requested_page - 1;  //highest available page -- could be -1 if we're requesting page 0
				memcpy(&app->age, &profile->age, sizeof(uint8_t) * AQUEDUCT_MAX_PAGES);
				DEBUG_P1("handle_profile(): app->version = %C\n", app->version);
				DEBUG_P1("handle_profile(): app->requested_page = %C\n", app->requested_page);
				DEBUG_P1("handle_profile(): app->goal_page = %C\n", app->goal_page);
				app->page_to_be_serviced = AQUEDUCT_NULL_PAGE;
				app->packets_to_be_serviced[0] = 0;
				app->packets_to_be_serviced[1] = 0;
				app->packets_to_be_serviced[2] = 0;

				/* TODO: Why can't I change to RX mode now?? */
				app->node_causing_rx = foot_profile->sender_ID;
				DEBUG_P1("handle_profile(): app->node_causing_rx = %d\n", app->node_causing_rx);
				change_state(app, AQUEDUCT_RX);
			}
		}
	}
	DEBUG_P2("handle_profile(): end\n");
}

/**
 * @brief Services the packet
 */
static void send_data(aqueduct_app *app, uint8_t packet)
{
	DEBUG_P2("send_data(packet: %C): begin\n", packet);
	aqueduct_data data;

	my_buf.size = sizeof(aqueduct_foot_data);
	my_buf.data[0] = app->image_ID;            // 1
	my_buf.data[1] = app->version;             // 1
	my_buf.data[2] = app->page_to_be_serviced; // 1
	my_buf.data[3] = packet;                   // 1
	
	/* HACK: insert fake data */
	int i;
	data.image_ID = app->image_ID;
	data.version = app->version;
	data.page = app->page_to_be_serviced;
	data.packet = packet;
	for (i = 0; i < AQUEDUCT_DATA_PAYLOAD_SIZE; i++) {
		data.payload[i] = packet;
		my_buf.data[4 + i] = packet;
	}
//	memcpy(&my_buf.data[4], &data.payload, sizeof(uint8_t) * AQUEDUCT_DATA_PAYLOAD_SIZE); // AQUEDUCT_DATA_PAYLOAD_SIZE
	uint16_t crc = crc_compute((uint8_t *)&data, sizeof(aqueduct_data));
	memcpy(&my_buf.data[4 + AQUEDUCT_DATA_PAYLOAD_SIZE], &crc, sizeof(uint16_t));
	memcpy(&my_buf.data[6 + AQUEDUCT_DATA_PAYLOAD_SIZE], &my_ID, sizeof(uint16_t));
	my_buf.data[8 + AQUEDUCT_DATA_PAYLOAD_SIZE] = AQUEDUCT_PACKET_DATA;

#if DEBUG > 0
	dump_data(&data);
#endif
	mos_led_toggle(0);
	aqueduct_send(&my_buf, AQUEDUCT_LISTEN_PORT);
	DEBUG_P2("send_data(packet: %C): end\n", packet);
}

/**
 * @brief A node in TX is responsible for broacasting all requested packets for a given page.
 */
static void aqueduct_tx(aqueduct_app *app) 
{
	DEBUG_P2("aqueduct_tx(): begin\n");
	/* Deluge services requests by taking the union of any new requests with previous requests
	 * not yet serviced.  Packets are sent in round-robin order to provide fairness among requesters.
	 */
	int i, k;
	uint16_t j;
	bool sent_data = FALSE;
	while (app->packets_to_be_serviced[0] != 0 || app->packets_to_be_serviced[1] != 0 || app->packets_to_be_serviced[2] != 0) {
		for (k = 0; k < 3; k++) {
			/* NOTE: packets_to_be_serviced are read in reverse order (compared to packets_needed) for each element.
			 * For example, the left-most bit of packets_needed[0] == the right-most bit of packets_to_be_serviced[0].
			 * The right-most bit of packets_needed[0] == the left-most bit of packets_to_be_serviced[0].
			 */
			for (i = 0; i < 16; i++) {
				j = (1 << i);
	//			DEBUG_P1("(%d & %d) = %C\n", app->packets_to_be_serviced[k], j, (app->packets_to_be_serviced[k] & j));
				if ((app->packets_to_be_serviced[k] & j) != 0) {
					//listen(app, AQUEDUCT_TX_DELAY_MAX);
//					UNLOCK_SLEEP_LOCK(&app->rw_lock, AQUEDUCT_TX_DELAY_MAX);
					//UNLOCK_SLEEP_LOCK2(&app->rw_lock, &app->rw_lock2, AQUEDUCT_TX_DELAY_MAX);
					//mos_mutex_unlock(&app->rw_lock);
					//mos_thread_sleep(AQUEDUCT_TX_DELAY_MAX);
					//mos_mutex_lock(&app->rw_lock);
					uint8_t packet_index = i + 16 * k;
					DEBUG_P1("aqueduct_tx(): Next packet to send: %C\n", packet_index);
					send_data(app, packet_index);
					app->packets_to_be_serviced[k] &= (0xFFFF ^ j); // Turn off bit (j * k)
					sent_data = TRUE;
				}
				if (app->packets_to_be_serviced[k] == 0) {
					break;
				}
			}
		}
	}

	app->page_to_be_serviced = AQUEDUCT_NULL_PAGE;
	assert(app->packets_to_be_serviced[0] == 0 && app->packets_to_be_serviced[1] == 0 && app->packets_to_be_serviced[2] == 0, "aqueduct_tx(): packets_to_be_serviced not clear!\n");

//	if (sent_data) {
//		mos_thread_sleep(2000);
//	}
	change_state(app, AQUEDUCT_MAINTAIN);
	DEBUG_P2("aqueduct_tx(): end\n");
}

/**
 * @brief Implements Rule M.6.
 */
void handle_request(comBuf *buffer)
{
	assert(GET_AQUEDUCT_PACKET_TYPE(buffer) == AQUEDUCT_PACKET_REQUEST, "handle_request(): incorrect packet type");
	DEBUG_P2("handle_request(): begin\n");
	/* unpackage buffer */
	aqueduct_foot_request fr;
	aqueduct_foot_request *foot_request = &fr;
	aqueduct_request *req = &fr.request;
	req->image_ID = buffer->data[0];
	req->version = buffer->data[1];
	req->page = buffer->data[2];
	memcpy(&req->packets_needed, &buffer->data[3], sizeof(uint16_t) * 3);
	memcpy(&foot_request->sender_ID, &buffer->data[9], sizeof(uint16_t));
	memcpy(&foot_request->destination_ID, &buffer->data[11], sizeof(uint16_t));
	foot_request->type = buffer->data[13];

	aqueduct_app *app;

	int i;

	for (i = 0; i < my_app_count; i++) {
		app = &my_app_memory[i];
		/* We process the data if the node requesting has an outdated version
		 * and the page is less than or equal to our high_page.
		 */
		/*****************************************
		 * HACK: Very strange behavior:
		 *       (app->image_ID == req->image_ID && app->version == req->version) evaluates to FALSE
		 *       even when app->image_ID == req->image_ID AND app->version == req->version are TRUE.
		 *       This is a cheap workaround.
		 */
		int app_image__eq__req_image = ((app->image_ID == req->image_ID));
		int app_image__eq__req_image__AND__app_version__eq__req_version = (app_image__eq__req_image && (app->version == req->version));

		/** This timestamp used in rule M.2 and M.3 */
		app->last_request_overheard_timestamp = mos_get_realtime();
		app->request_heard++;
		DEBUG_P1("handle_request(): overheard request at: %l\n", app->last_request_overheard_timestamp);
		DEBUG_P1("handle_request(): app->request_heard = %C\n", app->request_heard);

		//if (app->image_ID == req->image_ID && app->version == req->version && app->high_page >= req->page) {
		if (app_image__eq__req_image__AND__app_version__eq__req_version && app->high_page >= req->page) {
		/***end of hack**************************/

			if (app->page_to_be_serviced != AQUEDUCT_NULL_PAGE && app->page_to_be_serviced != req->page) {
				DEBUG_P1("handle_request(): Ignoring request for %C when a request existed for page %C.\n", req->page, app->page_to_be_serviced);
				continue;
			}

			if (my_ID == foot_request->destination_ID) {
				MARK();
//				/** This timestamp used in rule R.1 */
//				app->last_request_to_me_timestamp = mos_get_realtime();
				app->request_between_0_and_high_page_timestamp = mos_get_realtime();

				if (app->page_to_be_serviced == AQUEDUCT_NULL_PAGE) {
					MARK();
					app->page_to_be_serviced = req->page;
				}

				DEBUG_P1("handle_request(): req->page = %C\n", req->page);
				DEBUG_P1("handle_request(): req->packets_needed = ");
				dump_uint8_t_array(6, (uint8_t*)&req->packets_needed);
				DEBUG_P1("\n");
				DEBUG_P1("handle_request(): before - app->packets_to_be_serviced = ");
				dump_uint8_t_array(6, (uint8_t*)&app->packets_to_be_serviced);
				DEBUG_P1("\n");
				app->packets_to_be_serviced[0] |= req->packets_needed[0];
				app->packets_to_be_serviced[1] |= req->packets_needed[1];
				app->packets_to_be_serviced[2] |= req->packets_needed[2];

				DEBUG_P1("handle_request(): after - app->packets_to_be_serviced = ");
				dump_uint8_t_array(6, (uint8_t*)&app->packets_to_be_serviced);
				DEBUG_P1("\n");
				/**
				 * Rule M.6
				 *
				 * On receiving a request for data from a page p <= GAMMA from version v,
				 * transition to TX.
				 */
				if (app->aqueduct_state == AQUEDUCT_MAINTAIN) {
					change_state(app, AQUEDUCT_TX);
				}
			}
		}
	}
	DEBUG_P2("handle_request(): end\n");
}

/**
 * @brief Handles data packets.  Related to R.2 - tracks the reception rate
 */
static void handle_data(comBuf *buffer)
{
	aqueduct_foot_data *foot_data = (aqueduct_foot_data *)buffer->data;
	assert(GET_AQUEDUCT_PACKET_TYPE(buffer) == AQUEDUCT_PACKET_DATA, "handle_data(): incorrect packet type");

	aqueduct_data *data = (aqueduct_data *)foot_data;
	aqueduct_app *app;
#if DEBUG > 0
	dump_data(data);
#endif
	int i;
	for (i = 0; i < my_app_count; i++) {
		app = &my_app_memory[i];
		/**
		 * This counter is useful for M.2, and M.3
		 */
		app->last_data_overheard_timestamp = mos_get_realtime();
		app->data_heard++;
		/* We process the data if the version is earlier than the one we have
		 * and the page is the one we requested.
		 */
		if (app->image_ID == data->image_ID && app->version == data->version && app->requested_page == data->page) {
			MARK();
			assert(app->goal_page != AQUEDUCT_NULL_PAGE, "handle_data(): app->goal_page is not set!\n");
			assert(app->requested_page != AQUEDUCT_NULL_PAGE, "handle_data(): app->requested_page is not set!\n");
			assert(data->page != AQUEDUCT_NULL_PAGE, "handle_data(): data->requested_page is not set!\n");

			/**
			 * Process the payload
			 *
			 * 1. Validate CRC
			 * 2. Store data or reject packet
			 */
			if (foot_data->crc == crc_compute((uint8_t *)data, sizeof(aqueduct_data))) {
				MARK();
				uint16_t j = (0xFFFF) ^ (1 << (data->packet % 16));

				DEBUG_P1("handle_data(): data->packet = %C\n", data->packet);
				DEBUG_P1("handle_data(): j = %d\n", j);
				DEBUG_P1("handle_data(): before - packets_needed = ");
				dump_uint8_t_array(6, (uint8_t*)&app->packets_needed);
				DEBUG_P1("\n");

				uint16_t old_need = app->packets_needed[data->packet / 16];
				app->packets_needed[data->packet / 16] &= j;
				/* TODO: this next line is just for debugging */
				if (app->requested_page == data->page && app->packets_needed[data->packet / 16] == old_need) {
FG_YELLOW();
					DEBUG_P1("handle_data(): I got a duplicate packet for page %C\n", data->page);
TERM_RESET();
				}
				app->data_to_me_count++;
//				app->last_data_to_me_timestamp = mos_get_realtime();  /** Useful for R.1 */

				DEBUG_P1("handle_data(): after  - packets_needed = ");
				dump_uint8_t_array(6, (uint8_t*)&app->packets_needed);
				DEBUG_P1("\n");


			} else {
				DEBUG_P1("handle_data(): CRC does not match!  Received: %d, Expected: %d\n", foot_data->crc, crc_compute((uint8_t *)data, sizeof(aqueduct_data)));
			}
		} else if (app->image_ID == data->image_ID && app->version == data->version && app->high_page + 1 >= data->page){
			DEBUG_P1("handle_data(): data between [0, high_page + 1] heard\n");
			app->data_between_0_and_high_page_plus_one_timestamp++;
		}
	}
}

/**
 * @brief Dispatches incoming packets
 */
static void parse_com_buf(aqueduct_app *app, comBuf *buffer)
{
	comBuf b;
	b.size=buffer->size;
	memcpy(&b.data, &buffer->data, b.size);
	comBuf *bp;
	bp = &b;
	uint8_t type = GET_AQUEDUCT_PACKET_TYPE(bp);
	
	switch(type) {
		case AQUEDUCT_PACKET_SUMMARY:
			DEBUG_P1("Parsing AQUEDUCT_PACKET_SUMMARY\n");
			handle_summary(&b);
			break;
		case AQUEDUCT_PACKET_PROFILE:
			DEBUG_P1("Parsing AQUEDUCT_PACKET_PROFILE\n");
			handle_profile(&b);
			break;
		case AQUEDUCT_PACKET_REQUEST:
			DEBUG_P1("Parsing AQUEDUCT_PACKET_REQUEST\n");
			handle_request(&b);
			break;
		case AQUEDUCT_PACKET_DATA:
			DEBUG_P1("Parsing AQUEDUCT_PACKET_DATA\n");
			handle_data(&b);
			break;
		case AQUEDUCT_PACKET_COMMAND:
			DEBUG_P1("Parsing AQUEDUCT_PACKET_COMMAND\n");
			break;
		default:
			DEBUG_P1("type = %C\n", type);
			WARN("parse_com_buf: Illegal packet type -- Probably a corrupted packet.\n");
			break;
	}

}

/**
 * @brief Receives packets on the specified port.
 * HACK: For some reason I can't get the net layer to cooperate.
 * I gave up on it and decided to use only the com layer.
 * I put the PORT on as the footer.
 */
// static comBuf *aqueduct_recv(uint8_t port)
// {
// 	comBuf *buffer;
// 	uint8_t iface;
// 	IF_SET set;
// 
// 	com_mode(IFACE_RADIO, IF_LISTEN); //put radio in listen mode
// 	
// 	IF_ZERO(&set);
// 	IF_SET(IFACE_RADIO, &set);
// 
// 	iface = com_select(&set, COM_NOBLOCK);
// 	if (IF_ISSET(IFACE_RADIO, &set)) {
// 		buffer = com_recv(IFACE_RADIO);
// 
// /* HACK */
// 		if (buffer && ((buffer->data[buffer->size - 1] + 1 == AQUEDUCT_LISTEN_PORT) || (buffer->data[buffer->size - 1] - 1 == AQUEDUCT_LISTEN_PORT))) {
// 			#if DEBUG >= 1
// 				int i;
// 				FG_GREEN();
// 				DEBUG_P3(">> buffer |%C| = [", buffer->size);
// 				for (i = 0; i < buffer->size - 1; i++) {
// 					DEBUG_P3("%C, ", buffer->data[i]);
// 				}
// 				DEBUG_P3("%C]\n", buffer->data[i]);
// 				TERM_RESET();
// 			#endif
// 			buffer->size--;
// 			return buffer;
// 		} else {
// 			com_free_buf(buffer);
// 			return NULL;
// 		}
// 	} else {
// 		return NULL;
// 	}
// }

static bool inconsistency_overheard(aqueduct_app *app)
{
	return (app->outdated_summaries_heard || app->newer_summaries_heard || app->outdated_profiles_heard || app->newer_profiles_heard || app->request_heard || app->data_heard);
}

// static bool my_timeout_alarm_flag; /* FALSE if the alarm has not been signaled.  TRUE otherwise */
// static uint32_t my_timeout_alarm_ms; /* Time that the timeout alarm waits for before setting the my_timeout_alarm_flag */
// static mos_thread_t *timeout_alarm_thread;
// /**
//  * @brief Sets the flag that times out the listen thread
//  */
// static void timeout_alarm(void)
// {
// 
// 	my_timeout_alarm_flag = FALSE;
// 	mos_sem_post(&my_timeout_sem);
// 
// 	timeout_alarm_thread = _current_thread;
// 	mos_thread_sleep(my_timeout_alarm_ms);
// 
// 	DEBUG_P1("Listen timed out\n");
// 	my_timeout_alarm_flag = TRUE;
// }
// 
// 
// 
// /**
//  * @brief Listen for a given period of time.
//  */
// static void listen(aqueduct_app *app, uint32_t timeout_ms)
// {
// 	DEBUG_P2("\nListening for timespan: %l\n", timeout_ms);
// 	/* TODO: clean this up - originally here as a hack to compensate timeouts being off */
// //	timeout_ms = timeout_ms;
// 	comBuf *buffer;
// 
// 	my_timeout_alarm_ms = timeout_ms;
// 
// 	/* Alarms are more appropriate to use... but they're not implemented in mantis-0.9.5
// 	 * and they don't work in mantis-unstable.
// 	 */
// 	DEBUG_P2("Just before spawn\n");
// 	if (mos_thread_new(timeout_alarm, 128, PRIORITY_NORMAL) != THREAD_OK) {
// 		assert(FALSE, "Creation of timeout_alarm thread failed!\n");
// 	}
// 
// 	mos_sem_wait(&my_timeout_sem);
// 	while (!my_timeout_alarm_flag) {
// 		//uint32_t start_time = mos_get_realtime();
// 		/* HACK: If I don't do this the timing is thrown way off (i.e. fires too fast) and nothing gets received */
// 		mos_thread_sleep(40);
// 
// 		DEBUG_P3(".");
// 
// 		buffer = aqueduct_recv(AQUEDUCT_LISTEN_PORT);
// 
// 		if (buffer) {
// 			DEBUG_P1("Received comBuf\n");
// 			mos_led_toggle(1);
// 			uint8_t old_state = app->aqueduct_state;
// 			parse_com_buf(app, buffer);
// 			/* HACK: This is not exactly the most thread safe... it would be best if we didn't have to spawn the new timeout thread */
// 			if (app->aqueduct_state != old_state) {
// 				DEBUG_P1("State changed -- immediately exiting listen\n");
// 				my_timeout_alarm_flag = TRUE;
// 				timeout_alarm_thread->st = 0;
// 			} else if (inconsistency_overheard(app) && app->aqueduct_state == AQUEDUCT_MAINTAIN) {
// 				DEBUG_P4("listen(): aqueduct_maintain(): Inconsistency heard -- immediately exiting listen\n");
// 				my_timeout_alarm_flag = TRUE;
// 				timeout_alarm_thread->st = 0;
// 				change_state(app, AQUEDUCT_RESTART_MAINTAIN);
// 			}
// 			com_free_buf(buffer);
// 		}
// 	}
// }

/**
 * @brief Receives packets on the specified port.
 * HACK: For some reason I can't get the net layer to cooperate.
 * I gave up on it and decided to use only the com layer.
 * I put on the PORT as a footer.
 */
static void aqueduct_send(comBuf *buffer, uint8_t port)
{
	buffer->size++;
	buffer->data[buffer->size - 1] = port;
	
// 	#if DEBUG >= 3
//		int i;
		FG_RED();
		DEBUG_P1("<< buffer |%C| = [", buffer->size);
		dump_uint8_t_array(buffer->size, (uint8_t*)buffer->data);
//		for (i = 0; i < buffer->size - 1; i++) {
//			DEBUG_P3("%C, ", buffer->data[i]);
//		}
//		DEBUG_P3("%C]\n", buffer->data[i]);
		TERM_RESET();
// 	#endif
	com_send(IFACE_RADIO, buffer);
}

/**
 * @brief Broadcasts the summary of image_ID and version.
 */
static void send_summary(aqueduct_app *app)
{
	DEBUG_P2("send_summary(version: %C, high_page: %C): begin\n", app->version, app->high_page);
	assert(app != NULL, "send_summary(): parameter \"app\" is null");
	assert(app->image_ID < AQUEDUCT_MAX_UNIQUE_IMAGE_IDS && app->version < AQUEDUCT_MAX_UNIQUE_VERSION_IDS, 
		   "send_summary(): invalid ID/version ID");
	assert(app->high_page != AQUEDUCT_NULL_PAGE, "send_summary(): high_page is null\n");
	assert(app->high_page <= app->absolute_high, "send_summary(): high_page > absolute_high\n");
	/* NOTE: if we're being uber-careful we should assert that image_ID and version are stored in our "image memory" */

	my_buf.size = 7;
	my_buf.data[0] = app->image_ID;                      //1
	my_buf.data[1] = app->version;                       //1
	my_buf.data[2] = app->high_page;                     //1   Pages [0, app->high_page] are complete
	my_buf.data[3] = app->dtb;                           //1
	my_buf.data[4] = (uint8_t)my_ID;                     //1
	my_buf.data[5] = my_ID << 8;                         //1
	my_buf.data[6] = AQUEDUCT_PACKET_SUMMARY;            //1
	
	mos_led_toggle(0);
	aqueduct_send(&my_buf, AQUEDUCT_LISTEN_PORT);
	DEBUG_P2("send_summary(version: %C, high_page: %C): end\n", app->version, app->high_page);
}

/**
 * @brief part of MAINTAIN state.  Handles rule M.2, M.3
 * @TODO: have not taken int account request inconsistencies or data packet inconsistencies!
 * @param current_round_duration is measured in seconds
 */
static uint32_t adjust_round_duration(bool inconsistency, uint32_t current_round_duration) 
{
	DEBUG_P1("adjust_round_duration(): inconsitency = %C\n", inconsistency);
	if (inconsistency) {
		/* Rule M.2: 
		 *
		 * If any overheard packet indicates an inconsistency among neighboring nodes 
		 * (i.e. advertisements with OMEGA_PRIME != OMEGA, any requests, or any data packet) were 
		 * overheard during round i, set T_{M, I} to T_L and begin a new round.
		 */
		DEBUG_P1("adjust_round_duration(): return AQUEDUCT_T_L_SECONDS\n");
		return AQUEDUCT_T_L_SECONDS;
	} else {
		/* Rule M.3:
		 *
		 * If no overheard packet indicates an inconsistency among neighbors during the previous
		 * round, set T_{M,I} to min(2 * T_{M, I-1}, T_H)
		 */
		DEBUG_P1("adjust_round_duration(): return min(2 * current_round_duration, AQUEDUCT_T_H_SECONDS) = %d\n", min(2 * current_round_duration, AQUEDUCT_T_H_SECONDS));
		return min(2 * current_round_duration, AQUEDUCT_T_H_SECONDS);
	}
}

/**
 * @brief Send out an object profile for the appropriate image_id
 */
static void send_profile(aqueduct_app *app)
{
	DEBUG_P2("send_profile(): begin\n");
	assert(app != NULL, "send_profile(): parameter \"app\" is null");
	assert(app->image_ID < AQUEDUCT_MAX_UNIQUE_IMAGE_IDS && app->version < AQUEDUCT_MAX_UNIQUE_VERSION_IDS, 
		   "send_profile - invalid ID/version ID");
	assert(app->high_page != AQUEDUCT_NULL_PAGE, "send_profile(): high_page is null\n");

	/* NOTE: if we're being uber-careful we should assert that image_ID and version are stored in our "image memory" */

	my_buf.size = 3 + 1 + AQUEDUCT_MAX_PAGES + 2 + 1;
	my_buf.data[0] = app->image_ID;                                             //1
	my_buf.data[1] = app->version;                                              //1
	my_buf.data[2] = app->high_page;   //This is the receiver's goal_page       //1
	my_buf.data[3] = app->absolute_high;                                        //1
	memcpy(&my_buf.data[4], &app->age, sizeof(uint8_t) * AQUEDUCT_MAX_PAGES);   //AQUEDUCT_MAX_PAGES
	my_buf.data[4 + sizeof(uint8_t) * AQUEDUCT_MAX_PAGES] = (uint8_t)my_ID;
	my_buf.data[5 + sizeof(uint8_t) * AQUEDUCT_MAX_PAGES] = my_ID << 8;
	my_buf.data[6 + sizeof(uint8_t) * AQUEDUCT_MAX_PAGES] = AQUEDUCT_PACKET_PROFILE;          //1

	mos_led_toggle(0);
	aqueduct_send(&my_buf, AQUEDUCT_LISTEN_PORT);
	DEBUG_P2("send_profile(): end\n");
}

/**
 * @brief MAINTAIN state.  Handles rule M.1.  Calls for handlers of M.2, M.3, and M.4
 */
static void aqueduct_maintain(aqueduct_app *app)
{
	DEBUG_P2("aqueduct_maintain(): begin\n");
	assert(app->aqueduct_state == AQUEDUCT_MAINTAIN, "aqueduct_main -- not in AQUEDUCT_MAINTAIN");

	uint32_t current_round_duration_sec = app->maintain_round_duration_sec;  //a.k.a. T_{M, I}
	uint32_t r_i = random_time((current_round_duration_sec * 1000 / 2), (current_round_duration_sec * 1000));
	uint32_t timeout_ms = (r_i);
	uint32_t prev_neighbor_sent_request_timestamp = app->request_between_0_and_high_page_timestamp;

	DEBUG_P2("aqueduct_maintain(): timeout_ms = %l, current_round_duration = %l, r_i = %l\n", timeout_ms, current_round_duration_sec, r_i);

	reset_logs(app);
//	uint8_t old_state = app->aqueduct_state;
	//listen(app, timeout_ms);
	
	UNLOCK_SLEEP_LOCK(&app->rw_lock, timeout_ms);
	//UNLOCK_SLEEP_LOCK2(&app->rw_lock, &app->rw_lock2, timeout_ms);
	//mos_mutex_unlock(&app->rw_lock);
	//mos_thread_sleep(timeout_ms);
	//mos_mutex_lock(&app->rw_lock);

	if (app->aqueduct_state == AQUEDUCT_RESTART_MAINTAIN) {
		assert(inconsistency_overheard(app), "Inconsistency was not overheard yet somehow we're in RESTART_MAINTAIN\n");
		DEBUG_P1("aqueduct_maintain(): AQUEDUCT_RESTART_MAINTAIN\n");
		app->maintain_round_duration_sec = AQUEDUCT_T_L_SECONDS;
		change_state(app, AQUEDUCT_MAINTAIN);
	}

	DEBUG_P2("aqueduct_maintain(): Returned from listen at time t_i + r_i\n\n");

	/**** At this point we should be at time t_i + r_i. ****/

	/**
	 * Rule M.1:
	 *
	 * "Broadcast an advertisement with summary OMEGA at time t_i + r_i only if
	 * less than k advertisements with summary OMEGA_PRIME = OMEGA have been
	 * received since time t_i."
	 */
	uint32_t time_before = mos_get_realtime();
	if (app->equivolant_summaries_heard < AQUEDUCT_K && app->high_page != AQUEDUCT_NULL_PAGE) {
		send_summary(app);
	}

	/**
	 * Rule M.4:
	 *
	 * Transmit the object profile for version v at time t_i + r_i only if an advertisement with version v' <. v was
	 * received at or after time t_i and less than k attempts to update the object profile to version v have been
	 * overheard
	 */
	if (app->outdated_summaries_heard && app->equivolant_profiles_heard < AQUEDUCT_K && app->high_page != AQUEDUCT_NULL_PAGE) {
		send_profile(app);
	}

	DEBUG_P1("aqueduct_maintain(): app->equivolant_summaries_heard = %C\n", app->equivolant_summaries_heard);
	DEBUG_P1("aqueduct_maintain(): app->app->high_page = %C\n", app->high_page);
	DEBUG_P1("aqueduct_maintain(): app->outdated_summaries_heard = %C\n", app->outdated_summaries_heard);
	DEBUG_P1("aqueduct_maintain(): app->newer_summaries_heard = %C\n", app->newer_summaries_heard);
	DEBUG_P1("aqueduct_maintain(): app->request_heard = %C\n", app->request_heard);
	DEBUG_P1("aqueduct_maintain(): app->data_heard = %C\n", app->data_heard);

	uint32_t remaining_time = timeout_ms + (mos_get_realtime() - time_before);
	if (app->aqueduct_state == AQUEDUCT_MAINTAIN && current_round_duration_sec * 1000 > remaining_time) {
 		DEBUG_P2("aqueduct_maintain(): \"Sleeping\" off the rest of the round (sleep for: %l)\n", (((current_round_duration_sec * 1000) - timeout_ms)));
		//listen(app, remaining_time);
		
		UNLOCK_SLEEP_LOCK(&app->rw_lock, remaining_time);
		//UNLOCK_SLEEP_LOCK2(&app->rw_lock, &app->rw_lock2, remaining_time);
		//mos_mutex_unlock(&app->rw_lock);
		//mos_thread_sleep(remaining_time);  // sleep off the rest of the round
		//mos_mutex_lock(&app->rw_lock);
 		if (app->aqueduct_state == AQUEDUCT_RESTART_MAINTAIN) {
 			assert(inconsistency_overheard(app), "Inconsistency was not overheard yet somehow we're in RESTART_MAINTAIN\n");
 			DEBUG_P1("aqueduct_maintain(): AQUEDUCT_RESTART_MAINTAIN\n");
 			app->maintain_round_duration_sec = AQUEDUCT_T_L_SECONDS;
 			change_state(app, AQUEDUCT_MAINTAIN);
 		}
 
 		/** Rule M.5 */
 		if ((previously_received_request(app) || previously_received_data_packet(app)) && app->aqueduct_state == AQUEDUCT_RX) {
 			change_state(app, AQUEDUCT_MAINTAIN);
 		}
 	
 		/** Rule M.6 */
 		if (prev_neighbor_sent_request_timestamp != app->request_between_0_and_high_page_timestamp && app->aqueduct_state == AQUEDUCT_MAINTAIN) {
 			change_state(app, AQUEDUCT_TX);
 		}
	}

	/**** At this point we should be at time t_i + t_{m, i}.  This round is "over" ****/
	/**
	 * Part of rule M.2, M.3
	 */
	app->maintain_round_duration_sec = adjust_round_duration(inconsistency_overheard(app), current_round_duration_sec);
	DEBUG_P2("aqueduct_maintain(): end\n");
}

/**
 * @brief This is the main driver of aqueduct.
 */
void aqueduct_state_machine(void)
{
	int i = 0;
	uint8_t old_state = my_app_memory[i].aqueduct_state;
	/* TODO: clean this up */
	while (1) {
//		for (i = 0; i < my_app_count; i++) {
			aqueduct_app *app = &my_app_memory[i];
			DEBUG_P1("aqueduct_state_machine(): lock\n");
			mos_mutex_lock(&app->rw_lock);
			DEBUG_P1("aqueduct_state_machine(): resume\n");
			switch(app->aqueduct_state) {
				case AQUEDUCT_MAINTAIN: {
					aqueduct_maintain(app);
					break;
				}
				case AQUEDUCT_RX: {
					aqueduct_rx(app);
					break;
				}
				case AQUEDUCT_TX: {
					aqueduct_tx(app);
					break;
				}
				default: {
					DEBUG_P1("app->aqueduct_state = %C\n", app->aqueduct_state);
					assert(FALSE, "aqueduct_state_machine(): should never get here\n");
				}
			}
			if (old_state != app->aqueduct_state) {
				BG_GREEN(); FG_BOLD();
					DEBUG_P4("====================> NEW STATE = %C <===================\n", app->aqueduct_state);
				TERM_RESET();
				old_state = app->aqueduct_state;
			}
			DEBUG_P1("aqueduct_state_machine(): unlock\n");
			mos_mutex_unlock(&app->rw_lock);
			
			
		}
//	}
}

void sniffer(void)
{
	comBuf *buffer;
	int i;
	while (1) {
		com_mode(IFACE_RADIO, IF_LISTEN); //put radio in listen mode
		buffer = com_recv(IFACE_RADIO);
		FG_GREEN();
		DEBUG_P4(">> buffer |%C| = [", buffer->size);
		for (i = 0; i < buffer->size - 1; i++) {
			DEBUG_P4("%C, ", buffer->data[i]);
		}
		DEBUG_P4("%C]\n", buffer->data[i]);
		TERM_RESET();
		DEBUG_P4("free buff");
		com_free_buf(buffer);
	}
}

// aqueduct_app *current_app = NULL;
// comBuf *current_buffer = NULL;
// int handler_counter = 0;
// #define MAX_HANDLERS 2
// 
// void message_handler(void)
// {
// 	DEBUG_P1("message_handler(): start\n");
// 	handler_counter++;
// 	DEBUG_P1("message_handler(): before handler_counter = %C\n", handler_counter);
// 	assert(current_app && current_buffer, "message_handler(): current_app is NULL or current_buffer is NULL\n");
// 	parse_com_buf(current_app, current_buffer);
// 	com_free_buf(current_buffer);
// 	current_app = NULL;
// 	current_buffer = NULL;
// 	handler_counter--;
// 	DEBUG_P1("message_handler(): after handler_counter = %C\n", handler_counter);
// 	DEBUG_P1("message_handler(): end\n");
// }

void aqueduct_listen(void)
{
	comBuf *buffer;
	com_mode(IFACE_RADIO, IF_LISTEN); //put radio in listen mode
	aqueduct_app *app = &my_app_memory[0];
	DEBUG_P1("aqueduct_listen(): listening...\n");	
	while (1) {
		buffer = com_recv(IFACE_RADIO);
//		if (buffer && ((buffer->data[buffer->size - 1] == AQUEDUCT_LISTEN_PORT))) {
		if (buffer && ((buffer->data[buffer->size - 1] + 1 == AQUEDUCT_LISTEN_PORT) || (buffer->data[buffer->size - 1] - 1 == AQUEDUCT_LISTEN_PORT))) {
			DEBUG_P1("aqueduct_listen(): lock\n");
			mos_mutex_lock(&app->rw_lock);
			DEBUG_P1("aqueduct_listen(): resumed\n");
//			#if DEBUG >= 3
//				int i;
				FG_GREEN();
				DEBUG_P1(">> buffer |%C| = [", buffer->size);
				dump_uint8_t_array(buffer->size, buffer->data);
//				for (i = 0; i < buffer->size - 1; i++) {
//					DEBUG_P1("%C, ", buffer->data[i]);
//				}
//				DEBUG_P1("%C]\n", buffer->data[i]);
				TERM_RESET();
// 			#endif
			buffer->size--;

// 			current_app = app;
// 			current_buffer = buffer;
// 			if (handler_counter < MAX_HANDLERS) {
// 				int i;
// 				mos_mutex_lock(&app->rw_lock2);
// 				if ((i = mos_thread_new(message_handler, 512, PRIORITY_NORMAL)) != 0) {
// 					printf("mos_thread_new returned error code: %d\n", i);
// 				};
// 				mos_mutex_unlock(&app->rw_lock2);
// 			} else {
// 				DEBUG_P1("aqueduct_listen(): dropping buffer\n");
// 				com_free_buf(buffer);
// 			}

			parse_com_buf(app, buffer);
			com_free_buf(buffer);
			DEBUG_P1("aqueduct_listen(): unlocking\n");
			mos_mutex_unlock(&app->rw_lock);
		} else {
			com_free_buf(buffer);
		}

	}
}

void start(void)
{
	mos_node_id_set(NODE_ID);
	my_ID = mos_node_id_get();

 TERM_RESET();

	DEBUG_P4("************ Welcome to Aqueduct!  I'm node: %d\n", my_ID);
	mos_led_toggle(0);
	mos_led_toggle(1);

	mos_sem_init(&my_timeout_sem, 1);

	srandom(mos_get_realtime());

	my_app_count = 1;
	memset(&my_app_memory[0], 0, sizeof(aqueduct_app));

	if (NODE_ID != 3) {
		mos_mutex_init(&my_app_memory[0].rw_lock);
		my_app_memory[0].image_ID = 0;
		my_app_memory[0].version = 2;
		my_app_memory[0].high_page = 3;
		my_app_memory[0].dtb = 0;
		my_app_memory[0].absolute_high = my_app_memory[0].high_page;
		memset(&(my_app_memory[0].age), 0, AQUEDUCT_MAX_PAGES);
		my_app_memory[0].aqueduct_state = AQUEDUCT_MAINTAIN;
		my_app_memory[0].maintain_round_duration_sec = AQUEDUCT_T_L_SECONDS;
		my_app_memory[0].requested_page = AQUEDUCT_NULL_PAGE;
		my_app_memory[0].goal_page = AQUEDUCT_NULL_PAGE;
		my_app_memory[0].page_to_be_serviced = AQUEDUCT_NULL_PAGE;
		DEBUG_P1("Initially: %C, %C, %C\n", my_app_memory[0].image_ID, my_app_memory[0].version, my_app_memory[0].high_page);
	} else {
		mos_mutex_init(&my_app_memory[0].rw_lock);
		my_app_memory[0].image_ID = 0;
		my_app_memory[0].version = 3;
		my_app_memory[0].high_page = 3;
		my_app_memory[0].dtb = 0;
		my_app_memory[0].absolute_high = my_app_memory[0].high_page;
		memset(&(my_app_memory[0].age), 1, AQUEDUCT_MAX_PAGES);
		my_app_memory[0].aqueduct_state = AQUEDUCT_MAINTAIN;
		my_app_memory[0].maintain_round_duration_sec = AQUEDUCT_T_L_SECONDS;
		my_app_memory[0].requested_page = AQUEDUCT_NULL_PAGE;
		my_app_memory[0].goal_page = AQUEDUCT_NULL_PAGE;
		my_app_memory[0].page_to_be_serviced = AQUEDUCT_NULL_PAGE;
		DEBUG_P1("Initially: %C, %C, %C\n", my_app_memory[0].image_ID, my_app_memory[0].version, my_app_memory[0].high_page);
	}
	int i = 0;

	mos_multi_threaded();
	printf("address of i = %x\n", &i);
	if ((i = mos_thread_new(aqueduct_state_machine, 2048, PRIORITY_NORMAL)) != 0) {
		printf("mos_thread_new returned error code: %d\n", i);
	};
	if ((i = mos_thread_new(aqueduct_listen, 1024, PRIORITY_NORMAL)) != 0) {
		printf("mos_thread_new returned error code: %d\n", i);
	};
}
