#ifndef CTP_PLUS_DEBUG_H_
#define CTP_PLUS_DEBUG_H_
//#include "ctp_plus.h"

/*----------------------------- debugging code below -----------------------------*/

#include <stdarg.h>
#include <string.h>

#define PAD_RIGHT 1
#define PAD_ZERO 2

static void inline printchar(char **str, char c)
{	
	if (str) {
		**str = c;
		++(*str);
	}
}

static uint8_t prints(char **out, const char *string, int width, int pad)
{
	uint8_t pc = 0;
	char padchar = ' ';

	if (width > 0) {
		pc = strlen(string);		
		if (pc >= width) width = 0;
		else width -= pc;
		if (pad & PAD_ZERO) padchar = '0';
		pc = 0;
	}
	if (!(pad & PAD_RIGHT)) {
		for ( ; width > 0; --width) {
			printchar (out, padchar);
			++pc;
		}
	}
	for ( ; *string ; ++string) {
		printchar (out, *string);
		++pc;
	}
	for ( ; width > 0; --width) {
		printchar (out, padchar);
		++pc;
	}

	return pc;
}

/* the following should be enough for 16 bit int */
#define PRINT_BUF_LEN 6

static uint8_t printi(char **out, uint16_t u, uint8_t b, uint8_t width, uint8_t pad, char letbase)
{
	char print_buf[PRINT_BUF_LEN], *s;
	uint8_t  pc = 0;
	
	if (u == 0) {
		print_buf[0] = '0';
		print_buf[1] = '\0';
		return prints (out, print_buf, width, pad);
	}

	s = print_buf + PRINT_BUF_LEN-1;
	*s = '\0';

	while (u) {		
		--s;
		if( (u % b) >= 10 )
			*s = (u % b) + letbase - 10;
    else
		  *s = (u % b) + '0';
		u /= b;		
	}
	
	return pc + prints (out, s, width, pad);
}

static uint8_t print(char **out, const char *format, va_list args )
{
	uint8_t width, pad, pc = 0;
	char scr[2];

	for (; *format != 0; ++format) {
		if (*format == '%') {
			++format;
			width = pad = 0;
			if (*format == '\0') break;			
			if (*format == '-') {
				++format;
				pad = PAD_RIGHT;
			}
			while (*format == '0') {
				++format;
				pad |= PAD_ZERO;
			}
			for ( ; *format >= '0' && *format <= '9'; ++format) {
				width *= 10;
				width += *format - '0';
			}
			if( *format == 's' ) {
				char *s = va_arg( args, char * );
				if(s != NULL)
				   pc += prints (out, s, width, pad);
				continue;
			}
			if( *format == 'd' ) {
				pc += printi (out, va_arg( args, uint16_t), 10, width, pad, 'a');
				continue;
			}/*
			if( *format == 'l' ) {
				pc += printi_32 (out, va_arg( args, uint32_t), 10, width, pad, 'a');
				continue;
			}	*/	
			if( *format == 'C' ) {
				pc += printi (out, va_arg( args, uint16_t), 10, width, pad, 'a');
				continue;
			}
			if( *format == 'x' ) {
				pc += prints (out, "0x", 0, pad);
				pc += printi (out, va_arg( args, int ), 16, (width>2 ? width-2 : 0), pad, 'a');
				continue;
			}
			if( *format == 'X' ) {
				pc += prints (out, "0x", 0, pad);
				pc += printi (out, va_arg( args, int ), 16, (width>2 ? width-2 : 0), pad, 'A');
				continue;
			}			
			if( *format == 'c' ) {
				/* char are converted to int then pushed on the stack */
				scr[0] = (char)va_arg( args, int);
				scr[1] = '\0';
				pc += prints (out, scr, width, pad);
				continue;
			}
		}
		else {
			printchar (out, *format);
			++pc;
		}
	}
	if (out) **out = '\0';
	va_end( args );
	return pc;
}

uint8_t mos_sprintf(char *out, const char *format, ...)
{
    va_list args;        
    va_start( args, format );
    return print(&out, format, args );
}


/* Print Route
 * Format:
 * E2E ROUTE ETX <etx>.  PATH: hop1 hop2  ... hopN
 */
#define PRINTBUF_LEN  80
static char printbuf[PRINTBUF_LEN];
void printRouteFooter(const comBuf * buf) 
{
   uint8_t   i, ret;
   uint16_t  etx;
   ctp_data_header_t * dataHeader = (ctp_data_header_t*)buf->data;
	 
   ret = buf->data[buf->size-1];  // footer length
   if(ret + getHeaderSize(dataHeader->type) < buf->size)
   {    
     ret = buf->size - ret + 2;
     etx = b2w(buf->data[ret-2], buf->data[ret-1]);  
          
     for(i = 0; (ret < buf->size-2) && (i < PRINTBUF_LEN-6); ret += 2) {
       i += mos_sprintf(printbuf+i, " %d", b2w(buf->data[ret], buf->data[ret+1]));
     }
     printbuf[i] = '\0';
     printf("\nE2E ROUTE ETX %d. PATH: %d%s %s%d\n", etx, b2w(dataHeader->originHigh, dataHeader->originLow), 
                 printbuf, (ret < buf->size - 2) ? "... " : "", myid);      
   }
   else {
    	  printf("\nE2E ROUTE info unknown. corrupted packet.\n");   
   }  
}

/* Print Beacon Control Table
 * Format:
 * Upstream Beacon Control: 
 * ------------------------------------------------------------------------------------------------------------------
 * UL timer: <timer> <timer_range>
 * ------------------------------------------------------------------------------------------------------------------
 *
 * Dowstream Beacon Control
 * ------------------------------------------------------- 
 * DL beacons: <index> (<dest>, <timer>, <timer_range>)
 * -------------------------------------------------------
 */
#define BBUFLEN     80
static char bbuf[BBUFLEN];  
void printBeaconTimer() 
{
  uint8_t i, j = 0;	  
  if(num_dest > 0) {    
    for(i=0, j = 0; i< MAX_NUM_DEST && (j < BBUFLEN - 20); i++) 
      if(dest_nodes[i].id != BROADCAST_ADDR) 
        j += mos_sprintf(bbuf+j, " %C(%d, %C, %C)", i, dest_nodes[i].id, dest_nodes[i].beaconTimer, dest_nodes[i].beaconTimerCount);
    bbuf[j] = '\0';
  }
  printf("\nUL beacons: timer %d counter %d\n",  beaconTimer, beaconTimerCount);    
  printf("DL beacons:%s\n", bbuf);   
}


#define QBUFLEN   (CTP_DATA_QUEUE_SIZE * 6)
static char qbuf[QBUFLEN];
static uint16_t qdestbuf[CTP_DATA_QUEUE_SIZE];
void printDataQueueStatus()
{
   uint8_t i, j;
   for(i = dataQueueHead, j = 0; (i != (dataQueueHead + dataQueueSize) % CTP_DATA_QUEUE_SIZE) && (j < QBUFLEN - 8); )
   {
      if(qdestbuf[i] == 0xffff)
        j += mos_sprintf(qbuf+j, " BS");
      else
        j += mos_sprintf(qbuf+j, " %d", qdestbuf[i]);
        
      i =(i + 1) % CTP_DATA_QUEUE_SIZE;  
   }
   bbuf[j] = '\0';    
   printf("\nQueued Dests: %s\n", qbuf);   
}


/* Print Neighbor(Routing) Table 
 * Format:
 * main table 
 * ------------------------------------------------------------------------------------------------------------------
 * Index  freshness  address  ETXtoBase  inbound_lqi  outbound_lqi  PRR  last_recv_seqNo : ETXtoDest(1) ETXtoDest(2) ....
 * ------------------------------------------------------------------------------------------------------------------
 *
 * sub table (for each downstream destionation)
 * ------------------------------------------------------- 
 * dest_addr  bestETXtoDest nextHoptoDest Dest_freshness
 * -------------------------------------------------------
 */
void printTable(const routing_entry_t * t, const char * status, uint16_t addr)
{
  uint8_t i, j;

#ifdef TESTBED_LOGGING
printf("NT: parent indx %C (%s %d)", parent, status, addr);
#endif
  for(i=0, j=0; i<MAX_NEIGHBORS; i++) {
    if (t[i].addr==BROADCAST_ADDR) continue; 
#ifndef  TESTBED_LOGGING
    if (j==0) 
      printf("\n\nNT: parent indx %C (%s %d)\n%C  %2C  %2C  %d\t%d\t%C\t%C\t%d\t%C\t%3d\n", parent, status, addr,
                i, t[i].freshness, t[i].flag, t[i].addr,t[i].etx,t[i].ib_lqi, t[i].ob_lqi,t[i].prr, t[i].lastrecvdseqno, t[i].d_prr);
    else
      printf("%C  %2C  %2C  %d\t%d\t%C\t%C\t%d\t%C\t%3d\n",
               i, t[i].freshness, t[i].flag, t[i].addr,t[i].etx,t[i].ib_lqi, t[i].ob_lqi,t[i].prr, t[i].lastrecvdseqno, t[i].d_prr);
    j++;
#else
     printf("%C  %2C  %2C  %d\t%d\t%C\t%C\t%d\t%C\t%3d",
               i, t[i].freshness, t[i].flag, t[i].addr,t[i].etx,t[i].ib_lqi, t[i].ob_lqi,t[i].prr, t[i].lastrecvdseqno, t[i].d_prr);
#endif
  }
  printf("\n");

  if(num_dest <=0) return;
      
  printf("\nNT reverse ETX\n");
  for(i = 0; i < MAX_NEIGHBORS; i++) {
    if (t[i].addr == BROADCAST_ADDR) continue;

    printf("%C\t%d\t%C:%6d%6d\n", i, t[i].addr, t[i].ob_lqi, t[i].r_etx[0], t[i].r_etx[1]);
  }

  printf("\ndest num %C\n", num_dest);
  for(i = 0; i < MAX_NUM_DEST; ++i) {
     if(dest_nodes[i].id != BROADCAST_ADDR)
       printf("%C\t%d\t%d\t%d\t%C\t%C\n", i, dest_nodes[i].id, dest_nodes[i].r_nextHop, 
                      dest_nodes[i].r_bestETX, dest_nodes[i].freshness, dest_nodes[i].inqueue);
  } 
  printf("\n");
}

enum statusOfPacket {SENT, RECEIVED, DROPPED, DELAYED};
char *precoded_status[] = {"sent", "received", "dropped", "delayed"};

/* Print CTP_ROUTING frame (beacon for upstream cast) 
 * Format
 * main header
 * -----------------------------------------------------------------------
 * frame_type  Num_LEEPEntries  BeaconSeqNo  Options  Parent  EXTtoBase  
 * -----------------------------------------------------------------------
 * appended LEEP entries
 * --------------------------
 * neighbor_addr  inbound_lqi
 * --------------------------
 */
void printRTheader(const ctp_routing_header_t * rh, enum statusOfPacket status, uint16_t source)
{ 
  uint8_t i, j;  
  leep_entry_t * et;
  
  if(status==RECEIVED) {
    printf("\n\n%s RT (%d)\n%C\t%C\t%C\t%C\t%d\t%d\n", precoded_status[status], source, rh->type,rh->leepEntries, rh->seqno, rh->options, b2w(rh->parentHigh, rh->parentLow), b2w(rh->etxHigh, rh->etxLow) );
  }
  else if(status==SENT){
    printf("\n\n%s RT\n%C\t%C\t%C\t%C\t%d\t%d\n", precoded_status[status], rh->type,rh->leepEntries, rh->seqno, rh->options, b2w(rh->parentHigh, rh->parentLow), b2w(rh->etxHigh, rh->etxLow) ); 
  }   
  else {  
    return;
  }
      
  if( (j = rh->leepEntries>>4) > MAX_LIENTRIES )
    j = MAX_LIENTRIES; 
    
  for (i = 0; i < j; i++) {
    et = (leep_entry_t *)(&(rh->data[i*3])); 
    printf("%d\t%C%c\n", b2w(et->node_idHigh, et->node_idLow), et->lq, (i == j-1) ? '\n':' ');
  }
}

/* Print CTP_DL_ROUTING frame (beacon for downstream cast) 
 * Format
 * main header
 * --------------------------------------------------------------------------------------------------
 * frame_type  Num_LEEPEntries  BeaconSeqNo  Options  Parent(nextHop)  EXTtoBase  Num_DL_destination 
 * --------------------------------------------------------------------------------------------------
 * appended LEEP entries
 * --------------------------
 * neighbor_addr  inbound_lqi
 * --------------------------
 * appended destination entries
 * --------------------------
 * dest_addr  Parent(nextHop)  ETXtoDest
 * -------------------------- 
 */
void printRTheader_dl(const ctp_dl_routing_header_t * rh, enum statusOfPacket status, uint16_t source) 
{
  uint8_t i, j, k=0;  
  leep_entry_t * et;
  dest_entry_t * dt;  

  if(status==RECEIVED) {
    printf("\n\n%s DL RT (%d)\n%C\t%C\t%C\t%C\t%d\t%d\t%C\n", precoded_status[status], source, rh->type,rh->leepEntries, rh->seqno, rh->options, b2w(rh->parentHigh, rh->parentLow), b2w(rh->etxHigh, rh->etxLow), rh->destEntries);
  }
  else if(status==SENT){
    printf("\n\n%s DL RT\n%C\t%C\t%C\t%C\t%d\t%d\t%C\n", precoded_status[status], rh->type,rh->leepEntries, rh->seqno, rh->options, b2w(rh->parentHigh, rh->parentLow), b2w(rh->etxHigh, rh->etxLow), rh->destEntries); 
  } 
  else {  
    return;
  } 
  if( (j = rh->leepEntries>>4) > MAX_LIENTRIES )
    j = MAX_LIENTRIES;         
  for (i = 0; i < j; i++) {
    et = (leep_entry_t *)(&(rh->data[i * 3])); 
    printf("%d\t%C\n", b2w(et->node_idHigh, et->node_idLow), et->lq);
  }    
 
  if( (k = rh->destEntries) > MAX_DESTENTRIES )
    k = MAX_DESTENTRIES;    
  for (i=0, j=j*3; i < k; i++) {
    dt = (dest_entry_t *)(&(rh->data[j+i*6])); 
    printf("%d\t%d\t%d%c\n", b2w(dt->node_idHigh, dt->node_idLow), 
                             b2w(dt->r_parentHigh, dt->r_parentLow), b2w(dt->r_etxHigh, dt->r_etxLow), (i == k-1) ? '\n':' ');
  }  
} 

 
/* Print CTP_DATA frame (downstream cast data) 
 * Format 
 * --------------------------------------------------------------
 * frame_type  Options  EXTtoBase  THL  SeqNo  Oringinator_addr 
 * --------------------------------------------------------------
 */
void printDTheader(const ctp_data_header_t * rh, uint8_t status, uint16_t dest)
{
  if(status==SENT || status == RECEIVED)
    printf("\n\n%s DT (%d)\n%C\t%C\t%d\t%C\t%C\t%d\n\n", precoded_status[status], dest,
         rh->type, rh->options, b2w(rh->etxHigh, rh->etxLow), rh->thl, rh->originSeqNo, b2w(rh->originHigh, rh->originLow));    
  else if(status==DELAYED) 
    printf("\n\n%s DT\n%C\t%C\t%d\t%C\t%C\t%d\n\n", precoded_status[status],
         rh->type, rh->options, b2w(rh->etxHigh, rh->etxLow), rh->thl, rh->originSeqNo, b2w(rh->originHigh, rh->originLow));    
   
}

/* Print CTP_DATA frame (downstream cast data) 
 * Format 
 * ---------------------------------------------------------------------------
 * frame_type  Options  EXTtoBase  THL  SeqNo  Oringinator_addr  Destination
 * ---------------------------------------------------------------------------
 */
void printDTheader_dl(const ctp_dl_data_header_t * rh, uint8_t status, uint16_t dest)
{
  if(status==SENT || status == RECEIVED)
      printf("\n\n%s DL DT (%d)\n%C\t%C\t%d\t%C\t%C\t%d\t%d\n\n", precoded_status[status], dest, rh->type, rh->options, 
            b2w(rh->etxHigh, rh->etxLow), rh->thl, rh->originSeqNo, b2w(rh->originHigh, rh->originLow), b2w(rh->destHigh, rh->destLow));    
  else if(status==DELAYED)
      printf("\n\n%s DL DT\n%C\t%C\t%d\t%C\t%C\t%d\t%d\n\n", precoded_status[status], rh->type, rh->options, 
            b2w(rh->etxHigh, rh->etxLow), rh->thl, rh->originSeqNo, b2w(rh->originHigh, rh->originLow), b2w(rh->destHigh, rh->destLow)); 
}


/**  register the names of each running threads **/
void beacon_thread();
void send_thread();
void app_send();
void app_recv();
void debug_thread();

static char * threadNames[] = {"kernal", "beacon", "app_send", "app_recv", "debug", "send", "other"};  
uint8_t inline getThreadName(void (*func)(void)) {
  if(func == 0) return 0; 
  if(func == beacon_thread) return 1;
//  if(func == app_send) return 2;
//  if(func == app_recv) return 3;
  if(func == debug_thread) return 4;
  if(func == send_thread)  return 5;
  return 6;
} 

uint8_t inline snapshotLinkList(comBuf * from_bfl_head, comBuf ** to_bfl) { 
  uint8_t nbf;   
  for(nbf = 0, to_bfl[0] = from_bfl_head; nbf < NUM_BUFS-1 && to_bfl[nbf] != NULL; nbf++) {   
    to_bfl[nbf+1] = to_bfl[nbf]->next;    
  }
  return nbf;
}

/* Print debug info of thread stacks and comBufs 
 * Format
 * thread table
 * -----------------------------------------------------------------------
 * index  stack_addr  unused_bytes  state  is_condition_good  thread_name  
 * -----------------------------------------------------------------------
 * system sleepQueue:  linkedlist of ready threads (identified by their stack addrs) 
 * system readyQueue:  linkedlist of ready threads (identified by their stack addrs)
 * system comBufs
 * --------------------------------------------------------------------------
 * free comBuf pool :  linkedlist of available free comBufs
 * RADIO comBuf list ( thread waiting on IFACE_RADIO: thread's stack_addr):  
 *                         linkedlist of received comBufs hung on IFACE_RADIO
 * --------------------------------------------------------------------------
 */

void debug_thread(void)
{ 
 /* get to the first thread, which should be the kernal idle-loop thread */
  static mos_thread_t * mythreads = NULL;
  mythreads = _current_thread;
  while(mythreads->func != 0 || mythreads->state==EMPTY) {
    --mythreads;
  }      
      
  static uint16_t unused;          
  static uint8_t ok = 1, i, debug_print_timer = 200;         
  static char * myThreadNames[MAX_THREADS];  
             
  for(i = 0; i < MAX_THREADS; i++) {    
    if(mythreads[i].state != EMPTY) {
      unused = mos_check_stack(&mythreads[i]); 
      myThreadNames[i] = threadNames[getThreadName( mythreads[i].func )];      
      printf( "%C   %x    %x     %d       %3d     %C   %x   %s\n", 
                i, mythreads[i].sp, mythreads[i].stack, 
                mythreads[i].stackSize, unused, mythreads[i].state, myThreadNames[i]);
    }
  } 
  
  while(1) 
  {
    //printf("\ndebug thread\n");
      
    for(i = 0; i < MAX_THREADS; i++) {
      if(mythreads[i].state != EMPTY) {
        unused = mos_check_stack(&mythreads[i]);

        if(mythreads[i].sp < mythreads[i].stack) {
          printf("stack %C has overflowed\n", i);   ok = 0;
        }
        if(unused == 0) {
          printf("stack %C looks full\n", i);   ok = 0;
        }
        if(debug_print_timer == 0 || ok == 0) { 
          myThreadNames[i] = threadNames[getThreadName( mythreads[i].func )];
          printf("%C   %x     %3d    %C    %C  %s\n", 
                  i, mythreads[i].stack, unused, mythreads[i].state, ok, myThreadNames[i]);        
        }   
      }
    }
/*
    static comBuf * bf0[NUM_BUFS], * bf1[NUM_BUFS]; 
    static uint8_t nbf0, nbf1; 
    static mos_thread_t * tp = NULL;
       
    if(debug_print_timer == 0 || ok == 0) { 
      print_readyQ();  // need to hack msched.c
      print_sleepQ();  // need to hack msched.c      
      
      // need to hack com.c
      uint8_t int_handle = mos_disable_ints();
      nbf0 = snapshotLinkList(check_free_combuf(), bf0);
      nbf1 = snapshotLinkList(check_iface_combuf(IFACE_RADIO), bf1);
      tp = check_if_thread(IFACE_RADIO);
      mos_enable_ints(int_handle);
           
      printf("\nfree comBuf pool (%C) :\t", nbf0);  for(i=0; i<nbf0; i++) {printf("%x\t", bf0[i]); }             
      printf("\nRADIO comBuf list (%C) ( thread waiting on IFACE_RADIO: %x) :\t", nbf1, tp==NULL ? 0:tp->stack);
      for(i=0; i<nbf1; i++) { printf("%x\t", bf1[i]); }  printf("\n");      
    }
*/
/*    if(debug_print_timer == 0) {
      mos_thread_t * mth;
      mth = sendMutex.q.head; printf("sendMutex own %d: ", sendMutex.owner==NULL? 0: sendMutex.owner-mythreads);
      while(mth != NULL) { printf("%x ", mth->stack); mth = mth->next; }
      printf ("\n");

      mth = beaconMutex.q.head; printf("beaconMutex own %d: ", beaconMutex.owner==NULL? 0:beaconMutex.owner-mythreads);
      while(mth != NULL) { printf("%x ", mth->stack); mth = mth->next; }
      printf ("\n");

      mth = neighborTableLock.q.head; printf("neighborTableLock own %d: ", neighborTableLock.owner==NULL?0:neighborTableLock.owner-mythreads);
      while(mth != NULL) { printf("%x ", mth->stack); mth = mth->next; }
      printf ("\n");

      mth = optionlock.q.head; printf("optionLock own %d: ", optionlock.owner==NULL?0:optionlock.owner-mythreads);
      while(mth != NULL) { printf("%x ", mth->stack); mth = mth->next; }
      printf ("\n");

      mth = dataQueueMutex.q.head; printf("dataQueueMutex own %d: ", dataQueueMutex.owner==NULL?0:dataQueueMutex.owner-mythreads);
      while(mth != NULL) { printf("%x ", mth->stack); mth = mth->next; }
      printf ("\n");
    }    
*/ 
    debug_print_timer = (debug_print_timer==0) ? 200 :(debug_print_timer-1);   
    mos_thread_sleep(250);
  }  
}
/*----------------------------- debugging code above -----------------------------*/

#endif /*CTP_PLUS_DEBUG_H_*/




