//#include "ctp_plus.h"

/*----------------------------- debugging code below -----------------------------*/


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
void printTable(routing_entry_t * t, char * status)
{
  uint8_t i;
  for(i=0; i<MAX_NEIGHBORS; i++) {
    if (i==0) 
      printf("\n\n%s %d NT: cur parent indx is %C (%s)\n%C  %C\t%d\t%d\t%C\t%C\t%d\t%C:\t%d\t%d\n", is_base? "BS":"ND",
               myid, parent, status, i, t[i].freshness, t[i].addr,t[i].etx,t[i].ib_lqi, t[i].ob_lqi,t[i].prr, t[i].lastrecvdseqno, t[i].r_etx[0], t[i].r_etx[1]);
    else
      printf("%C  %C\t%d\t%d\t%C\t%C\t%d\t%C:\t%d\t%d%c\n",
               i, t[i].freshness, t[i].addr,t[i].etx,t[i].ib_lqi, t[i].ob_lqi,t[i].prr, t[i].lastrecvdseqno, t[i].r_etx[0], t[i].r_etx[1], (i==MAX_NEIGHBORS-1)?'\n':' ');
  }
 
  if(num_dest <=0) return;
  printf("\ndest num %C (%d, %d, %d)\n", num_dest, dest_nodes[0].id, dest_nodes[1].id, dest_nodes[2].id);

  for(i=0; i<MAX_NUM_DEST; i++) 
  {
    if(dest_nodes[i].id!=BROADCAST_ADDR) 
       printf("%d\t%d\t%d\t%C\n", dest_nodes[i].id, dest_nodes[i].r_bestETX, dest_nodes[i].r_nextHop, dest_nodes[i].freshness);
  } 
  printf("\n");
}

enum statusOfPacket {SENT, RECEIVED, DROPPED};
char *precoded_status[] = {"sent", "received", "dropped"};

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
void printRTheader(ctp_routing_header_t * rh, enum statusOfPacket status, uint16_t source)
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
void printRTheader_dl(ctp_dl_routing_header_t * rh, enum statusOfPacket status, uint16_t source) 
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
void printDTheader(ctp_data_header_t * rh, uint8_t status, uint16_t dest)
{
  printf("\n\n%s DT (%d)\n%C\t%C\t%d\t%C\t%C\t%d\n\n", precoded_status[status], dest,
         rh->type, rh->options, b2w(rh->etxHigh, rh->etxLow), rh->thl, rh->originSeqNo, b2w(rh->originHigh, rh->originLow));    
}

/* Print CTP_DATA frame (downstream cast data) 
 * Format 
 * ---------------------------------------------------------------------------
 * frame_type  Options  EXTtoBase  THL  SeqNo  Oringinator_addr  Destination
 * ---------------------------------------------------------------------------
 */
void printDTheader_dl(ctp_dl_data_header_t * rh, uint8_t status, uint16_t dest)
{
  printf("\n\n%s DL DT (%d)\n%C\t%C\t%d\t%C\t%C\t%d\t%d\n\n", precoded_status[status], dest,
         rh->type, rh->options, b2w(rh->etxHigh, rh->etxLow), rh->thl, rh->originSeqNo, b2w(rh->originHigh, rh->originLow), b2w(rh->destHigh, rh->destLow));    
}


/**  register the names of each running threads **/
void beacon_thread();
//void net_thread();
void app_send();
void app_recv();
void debug_thread();

static char * threadNames[] = {"kernal", "beacon", "app_send", "app_recv", "debug", "other"};  
uint8_t inline getThreadName(void (*func)(void)) {
  if(func == 0) return 0; 
  if(func == beacon_thread) return 1;
  if(func == app_send) return 2;
  if(func == app_recv) return 3;
  if(func == debug_thread) return 4;
  return 5;
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
    printf("\ndebug thread\n");
      
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
    debug_print_timer = (debug_print_timer==0) ? 200 :(debug_print_timer-1);   
    mos_thread_sleep(250);
  }  
}
/*----------------------------- debugging code above -----------------------------*/





