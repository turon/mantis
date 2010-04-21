#include "ctp.h"
#include "net.h"
#include "node_id.h"

#ifdef CTP
// NOTE: Keep consistency of address format.
// 1. com_send, com_recv on link layer won't take care of the address format. 
// 2. apps on network layer need to do htons conversion, if neccesary. 
//    e.g.  in ctp:
//          com_sendto(IFACE_RADIO, &send_buf, htons(dest))
//          sender's node id is: htons(recv_buf->source)


boolean is_base = 1;
uint16_t myid; 

mos_mutex_t sendQueueMutex;
mos_mutex_t beaconMutex;
mos_mutex_t parentLock;
mos_mutex_t optionlock;
mos_mutex_t packetPoolMutex;
mos_cond_t sendQueueCond;

/*-- protected by optionLock --*/
uint8_t options = CTP_OPT_PULL;
/*-- protected by optionLock --*/

/*-- protected by beaconMutex --*/
comBuf beaconPacket;
int16_t beaconTimer = BEACON_TIMER_MAX;
uint16_t beaconTimerCount = BEACON_TIMER_MAX;
uint8_t beaconSeqNo = 0;
uint8_t lastParent = 0;
/*-- protected by beaconMutex --*/

/*-- protected by sendQueueMutex --*/
uint8_t queueHead = 0;
uint8_t queueSize = 0;
comBuf* sendQueue[CTP_QUEUE_SIZE];
/*-- protected by sendQueueMutex --*/

/*-- protected by packetPoolMutex --*/
uint8_t poolHead = 0;
uint8_t poolSize = CTP_FORWARDING_SIZE;
comBuf  forwardingPackets[CTP_FORWARDING_SIZE];
comBuf* packetPool[CTP_FORWARDING_SIZE];
/*-- protected by packetPoolMutex --*/

/*-- protected by parentLock --*/
routing_entry_t neighbors[MAX_NEIGHBORS];
uint16_t bestEtx;
uint16_t nextHop;
uint8_t  parent;
uint8_t originSeqNo = 0;
/*-- protected by parentLock --*/

uint32_t seed;

#define htons(v) (((v) >> 8) | (((v) & 0xff) << 8))


/* variales for dubugging */
uint8_t  debug = 1;
char     myid_toString[6];
uint8_t  myid_len;

static stackval_t debug_stack[128];

/* Print debugging message */
void printTable(routing_entry_t * t, char * status)
{
  uint8_t i;
  for(i=0; i<MAX_NEIGHBORS; i++) {
    if (i==0) 
      printf("\n\n%s %d NT: cur parent indx is %C (%s)\n%C  %C\t%d\t%d\t%C\t%C\t%d\t%C\n", is_base? "BS":"ND",
               myid, parent, status, i, t[i].freshness, t[i].addr,t[i].etx,t[i].ib_lqi, t[i].ob_lqi,t[i].prr, t[i].lastrecvdseqno);
    else if(i==MAX_NEIGHBORS-1)
      printf("%C  %C\t%d\t%d\t%C\t%C\t%d\t%C\n\n",i, t[i].freshness, t[i].addr,t[i].etx,t[i].ib_lqi, t[i].ob_lqi,t[i].prr, t[i].lastrecvdseqno);
    else
      printf("%C  %C\t%d\t%d\t%C\t%C\t%d\t%C\n",i, t[i].freshness, t[i].addr,t[i].etx,t[i].ib_lqi, t[i].ob_lqi,t[i].prr, t[i].lastrecvdseqno);
  }  
}
/* Print Debugging message */
void printRTheader_send(ctp_routing_header_t * rh)
{ 
  uint8_t i, j;
  leep_entry_t * et;

  printf("\n\nsent RT\n%C\t%C\t%C\t%C\t%d\t%C\t%C\n", rh->type,rh->leepEntries, rh->seqno, rh->options, rh->parent, rh->etxHigh, rh->etxLow);  
  j = rh->leepEntries>>4;
  for (i=0; i<j && i < MAX_LIENTRIES; i++) {
    et = (leep_entry_t *)(&(rh->data[i*3]));
    if(i==j-1){
      printf("%d\t%C\n\n", (et->node_idHigh << 8) | (et->node_idLow & 0xff), et->lq);
    }
    else{
      printf("%d\t%C\n", (et->node_idHigh << 8) | (et->node_idLow & 0xff), et->lq);
    }
  }
}
/* Print debugging message */
void printRTheader_recv(ctp_routing_header_t * rh, uint16_t source)
{ 
  uint8_t i, j;  
  leep_entry_t * et;

  printf("\n\nreceived RT (%d)\n%C\t%C\t%C\t%C\t%d\t%C\t%C\n", source, rh->type,rh->leepEntries, rh->seqno, rh->options, rh->parent, rh->etxHigh, rh->etxLow);  
  j = rh->leepEntries>>4;
  for (i=0; i<j && MAX_LIENTRIES; i++) {
    et = (leep_entry_t *)(&(rh->data[i*3]));
    if(i==j-1)
      printf("%d\t%C\n\n", (et->node_idHigh << 8) | (et->node_idLow & 0xff), et->lq);
    else
      printf("%d\t%C\n", (et->node_idHigh << 8) | (et->node_idLow & 0xff), et->lq);
  }
}

/* Print debugging message */
void printDTheader(ctp_data_header_t * rh, char * status, uint16_t dest)
{
  printf("\n\n%s DT (%d)\n%C\t%C\t%C\t%C\t%C\t%C\t%C\t%C\n\n", status,dest,
         rh->type, rh->options, rh->etxHigh, rh->etxLow, rh->thl, rh->originSeqNo, rh->originHigh, rh->originLow);    
}


void send_thread(void) {
  uint16_t rVal;
  comBuf* packet;
  uint8_t ret;
  while(1) {
    if(debug) printf("\nsend thread\n");  
    mos_mutex_lock(&sendQueueMutex);
    
    while (queueSize == 0) {
      mos_cond_wait(&sendQueueCond, &sendQueueMutex);
    }

    packet = dequeuePacket();  //if sendQueue is empty it will return NULL
    if (packet == NULL) {  
      if(debug) printf("CTP ERROR MSG: fatal error -- why dequeuePacket returns nothing.\n");
      ret = 1;
    }
    else
      ret=send_packet(packet);
    mos_mutex_unlock(&sendQueueMutex);

    /* If this is a forwarding data packet, we need to return the buf to packetPool. */
    if (!ret && packet >= &forwardingPackets[0] &&
      packet <= &forwardingPackets[CTP_FORWARDING_SIZE - 1]) 
    {  
      mos_mutex_lock(&packetPoolMutex);
      freePacket(packet);
      mos_mutex_unlock(&packetPoolMutex);      

      mos_mutex_lock(&optionlock);
      options &= (~CTP_OPT_ECN) ;  // unset congestion bit
      mos_mutex_unlock(&optionlock);
    }

    // Wait the random time before sending another to avoid collision
    rVal = 200 + ((rand_mlcg()) %256);
    mos_thread_sleep(rVal);
  }
}

static uint8_t beacon_print_count = 20;
void beacon_thread(void) {
  uint8_t i, parentChanged, hasNoLIEntry;
  uint16_t dest;

  while(1) {
    if(debug) printf("\nbeacon thread: timer %C\n", beaconTimer);
    
    mos_mutex_lock(&beaconMutex);
    beaconTimer--;
    mos_mutex_unlock(&beaconMutex);
    
   /* minimal beacon interval is 1000 ms */
    mos_thread_sleep(1000);

   /* update routingTable with freshness and parent */
    mos_mutex_lock(&parentLock);
    if(!is_base)
      bestEtx = 0xffff; 
    for (i = 0, parentChanged = MAX_NEIGHBORS, hasNoLIEntry = 1; i < MAX_NEIGHBORS; i++) {
      if (neighbors[i].addr == BROADCAST_ADDR)
        continue;
      if (neighbors[i].freshness == 0) {
        neighbors[i].addr = BROADCAST_ADDR;
        neighbors[i].etx = 0xffff;
        neighbors[i].ib_lqi = 0xff;  
        neighbors[i].ob_lqi = 0xff;  
        neighbors[i].lastrecvdseqno = 0;
        neighbors[i].prr = 0; 
      }
      else {
        neighbors[i].freshness--;
        if(is_base) {
          hasNoLIEntry = 0;
        } 
        else {
          if(neighbors[i].ob_lqi <= MAX_LQI) hasNoLIEntry = 0;
         /* only applied to node */
          if (neighbors[i].etx < 0xffff - neighbors[i].ob_lqi)
            if(neighbors[i].etx + neighbors[i].ob_lqi < bestEtx) {
              bestEtx = neighbors[i].etx + neighbors[i].ob_lqi;
              parentChanged = i;
            }
        }
      }
    }
    /* chooseNewParent */
    /* only applied to node */
    if (!is_base && parentChanged != parent) {
      if (parentChanged < MAX_NEIGHBORS) {
        parent = parentChanged;
        nextHop = bestEtx > MAX_ETX ? BROADCAST_ADDR : neighbors[parent].addr;        
      }     
      else {
        parent = 0;
        nextHop = BROADCAST_ADDR;
      } 
      if(debug) printf("parent updated. current bestEtx %d, current nextHop %d\n", bestEtx, nextHop);
    } 
    if(debug && (beacon_print_count--)==0) {printTable(neighbors,"Updated freshness"); beacon_print_count=10;}   

    dest = nextHop;
    mos_mutex_unlock(&parentLock);    

    mos_mutex_lock(&optionlock);
    if(hasNoLIEntry) { // I have no routing or valid leap info yet
      options |= CTP_OPT_PULL;    // set PULL bit
    }
    else {
      options &= (~CTP_OPT_PULL); // unset PULL bit
    }
    mos_mutex_unlock(&optionlock);    
    
   /* produce a beacon packet */
    mos_mutex_lock(&beaconMutex);
    if (beaconTimer <= 0) {
      mos_mutex_lock(&sendQueueMutex);
      enqueueBeacon();
      mos_mutex_unlock(&sendQueueMutex);

      if (dest == BROADCAST_ADDR) {
        resetBeaconInterval();
      }
      else {
        beaconTimerCount *= 2;
        if (beaconTimerCount > BEACON_TIMER_MAX) {
          beaconTimerCount = BEACON_TIMER_MAX;
        }
        beaconTimer = beaconTimerCount;
      }
    }
    else if (dest == BROADCAST_ADDR) {
      resetBeaconInterval();
    }
    mos_mutex_unlock(&beaconMutex);
  }
}

void updateRoutingTable(uint16_t source, ctp_routing_header_t* rh) {
  uint8_t i, j, lqi = 0;
  uint16_t worstEtx;
  leep_entry_t * et;

  mos_mutex_lock(&parentLock);

 /* investigate the attached leep frame at the footer */
 /* only applied to node */
  if(!is_base) {
   /* Be careful not to exceed MAX_LIENTRIES */
    for(i=0, lqi = 0; i < ((rh->leepEntries) >> 4) && i < MAX_LIENTRIES; i++) {
      et = (leep_entry_t *)(&(rh->data[i*3]));
      if( ((et->node_idHigh << 8) | (et->node_idLow & 0xff)) == myid ) {
        lqi = et->lq;
        break;
      }
    }
  }
 /* If the neighbor is already in the table, update it */
  for (i = 0; i < MAX_NEIGHBORS && source != neighbors[i].addr; i++); 

 /* If it's not in the table, find an empty entry or 
    replace an entry with a worst ETX if this one is better. */  
  if (i == MAX_NEIGHBORS) {
    worstEtx = (rh->etxHigh << 8 | rh->etxLow);  
    for (j = 0; j < MAX_NEIGHBORS; j++) {
      if (neighbors[j].addr == BROADCAST_ADDR) {
        i = j;
        break;
      }
     /* only applied to node */ 
      if (!is_base && worstEtx < neighbors[j].etx) {
        worstEtx = neighbors[j].etx;
        i = j;
      }
    }
    if (i < MAX_NEIGHBORS) {
      neighbors[i].addr = source; 
      neighbors[i].etx = 0xffff;
      neighbors[i].ib_lqi = 0xff;  
      neighbors[i].ob_lqi = 0xff;       
      neighbors[i].prr = 0; 
      neighbors[i].lastrecvdseqno = 0; 
    }
  }

  if (i < MAX_NEIGHBORS) {     
    neighbors[i].freshness = NEW_FRESH;
    
    if(!is_base) {
   /* Update neighbor's ETX */
      neighbors[i].etx = (rh->etxHigh << 8) | (rh->etxLow) ;     
   /* Update my out-bound lqi to neighbor[i].addr */
      if (lqi > 0 ) neighbors[i].ob_lqi = lqi; 
    }

   /* Update PRR */
    neighbors[i].prr  = (neighbors[i].prr) << //((rh->seqno - neighbors[i].lastrecvdseqno) % 256); //seqno is a 8-bit unsigned integer
              ((rh->seqno > neighbors[i].lastrecvdseqno) ? (rh->seqno-neighbors[i].lastrecvdseqno) : (0xff-neighbors[i].lastrecvdseqno + rh->seqno+1)); 
    neighbors[i].prr |= 1;  // set the lsb to be 1
    neighbors[i].lastrecvdseqno = rh->seqno;
   
   /* Update my in-bound lqi from neighbor[i].addr */
    for(j = 0, lqi = MAX_LQI+1; j < PRR_WINDOWSIZE; ++j) 
      lqi -= ((neighbors[i].prr >> j) & 0x01);  
    neighbors[i].ib_lqi = (lqi > MAX_LQI) ? 0xff : lqi;
  }

 /* Update my parent, bestEtx and nextHop */
 /* only applied to node */
  if(!is_base) {
    bestEtx = 0xffff;
    for (i = 0, j= parent; i < MAX_NEIGHBORS; i++) {    
      if (neighbors[i].etx < 0xffff - neighbors[i].ob_lqi)
        if (neighbors[i].etx + neighbors[i].ob_lqi < bestEtx) {
          bestEtx = neighbors[i].etx + neighbors[i].ob_lqi;
          j = i;
        }
    }
    parent = j;
    nextHop = bestEtx > MAX_ETX ? BROADCAST_ADDR : neighbors[parent].addr;  
    //if(debug) printf("routing table updated. bestEtx %d, nextHop %d\n", bestEtx, nextHop);
  }
  if(debug) printTable(neighbors, "recvd a RT frame");    
  mos_mutex_unlock(&parentLock);
}


void ctp_proto_init(void) {

  uint8_t i;
	
 /* initialize my node_id */
  myid = mos_node_id_get();

 /* status msg: if I am the base or node */
  printf("CTP STATS MSG: Initializing %s sensor %d\n", is_base? "BASE":"NODE", myid);  

  mos_mutex_init(&sendQueueMutex);
  mos_mutex_init(&beaconMutex);
  mos_mutex_init(&parentLock);
  mos_mutex_init(&optionlock);
  mos_mutex_init(&packetPoolMutex);
  mos_cond_init(&sendQueueCond);

  seed_random();
  mos_led_off(0);
  mos_led_off(1);
  mos_led_off(2);
  
 /* initialize the send queue */
  queueHead = 0;
  queueSize = 0; 

 /* initialize the option */
  options = CTP_OPT_PULL;

 /* initialize beacon system */
  resetBeaconInterval();
  beaconSeqNo = 250;
  lastParent = 0;

 /* initialize the packet pool */
  for (i = 0; i < CTP_FORWARDING_SIZE; i++) {
    packetPool[i] = &forwardingPackets[i]; mos_udelay(1);
  }
  poolHead = 0;
  poolSize = CTP_FORWARDING_SIZE;
  
 /* initialize the routing table */ 
  for (i = 0; i < MAX_NEIGHBORS; i++) {
    neighbors[i].addr = BROADCAST_ADDR;
    neighbors[i].etx = 0xffff;
    neighbors[i].ib_lqi = 0xff;  
    neighbors[i].ob_lqi = 0xff;  
    neighbors[i].lastrecvdseqno = 0; 
    neighbors[i].prr = 0; 
    neighbors[i].freshness = 0;
    mos_udelay(2);
  }
  parent  = 0;
  bestEtx = is_base? 0 : 0xffff;
  nextHop = is_base? myid : BROADCAST_ADDR;
  originSeqNo = 0;
  
 /* ctp uses two backend threads */ 
  mos_thread_new(beacon_thread, 384, PRIORITY_NORMAL);
  mos_thread_new(send_thread, 384, PRIORITY_NORMAL);

 /* register ctp protocal */
  net_proto_register(CTP_PROTO_ID, net_ctp_proto_send, net_ctp_proto_recv, net_ctp_proto_ioctl);
}

void seed_random() {
  seed = mos_node_id_get() + 1;
}

static uint32_t rand_mlcg() {
  static uint32_t mlcg,p,q;
  static uint64_t tmpseed;

  tmpseed =  (uint64_t)33614U * (uint64_t)seed;
  q = tmpseed;  /* low */
  q = q >> 1;
  p = tmpseed >> 32 ;   /* hi */
  mlcg = p + q;
  if (mlcg & 0x80000000) { 
    mlcg = mlcg & 0x7FFFFFFF;
    mlcg++;
  }
  seed = mlcg;
  return mlcg; 
}

uint8_t inline getOptions() {
  uint8_t curoption;
  mos_mutex_lock(&optionlock);
    curoption = options;
  mos_mutex_unlock(&optionlock);
  return curoption;
}

uint8_t send_packet(comBuf* buf) {
  uint8_t ret;
  ctp_data_header_t* header = (ctp_data_header_t*)buf->data;

  ret = net_send(buf, CTP_PROTO_ID, buf->source, CTP_EXISTING_PACKET);

 /* if the packet cannot be routed out, re-enqueue the packet */
  if (ret && header->type == AM_CTP_DATA) 
  {
    if(debug) 
      printf("route not found yet. cannot send out data packet.\n");
    enqueuePacket(buf);
    return 1;
  }
  return ret;
}

void enqueueBeacon() {
  uint8_t i;

 /* Check if the packet is already in the queue */
  for (i = 0; i < CTP_QUEUE_SIZE; i++) {
    if (sendQueue[i] == &beaconPacket) {
      //busy
      break;
    }
  }

 /* If not, prep it and put it in the queue */ 
  if (i >= CTP_QUEUE_SIZE) {
    ctp_routing_header_t* header = (ctp_routing_header_t*)beaconPacket.data;
    header->type = AM_CTP_ROUTING;
    header->seqno = (beaconSeqNo++);

   /* options, parent, etx and leep entries will be updated when it is sent by send_thread. */
    header->options = 0;             
    header->parent = BROADCAST_ADDR; 
    header->etxHigh = 0xff;          
    header->etxLow  = 0xff;          
    header->leepEntries = 0;

    beaconPacket.size = CTP_ROUTING_HEADER_SIZE;
    beaconPacket.source = CTP_DUMMY_PORT;
    enqueuePacket(&beaconPacket);
  }
  else {
    mos_cond_signal(&sendQueueCond);
  }
}


void inline enqueuePacket(comBuf* buf) {
   if (queueSize < CTP_QUEUE_SIZE) {
    sendQueue[(queueHead + queueSize) % CTP_QUEUE_SIZE] = buf;
    queueSize++;
  }
  mos_cond_signal(&sendQueueCond);
}

comBuf* dequeuePacket() {
  if (queueSize == 0) {
    return NULL;
  }
  else {
    comBuf* rval = sendQueue[queueHead];
    sendQueue[queueHead] = NULL;
    queueHead = (queueHead + 1) % CTP_QUEUE_SIZE;
    queueSize--;
    return rval;
  }
}

comBuf* allocatePacket() {
  if (poolSize == 0) {
    return NULL;
  }
  else {
    comBuf* buf = packetPool[poolHead];
    poolHead++;
    poolSize--;
    return buf;
  }
}

void freePacket(comBuf* buf) {
  if(poolHead == 0) {
     if(debug)
       printf("CTP ERROR MSG: fatal error -- freePacket is called when no packet was allocated (packetPacket is full)\n");
     return;
  }   
  poolHead--;
  packetPool[poolHead] = buf;
  poolSize++;
}

int packetsLeft() {
  return poolSize;
}
 
void resetBeaconInterval() {
  beaconTimerCount = BEACON_TIMER_MIN;
  if (beaconTimer > beaconTimerCount) {
    beaconTimer = BEACON_TIMER_MIN;
  }
}

void beaconFast() {
  mos_mutex_lock(&beaconMutex);
  if(beaconTimer > 1) 
    beaconTimer = 1;
  mos_mutex_unlock(&beaconMutex);
}

boolean ctp_handle_data_packet(comBuf* buf, uint8_t port)
{
   uint8_t i;
   uint16_t myEtx;
   ctp_data_header_t* dataHeader;
   comBuf * fwd;

   if(debug) {
      dataHeader = (ctp_data_header_t *) buf->data;
      printDTheader(dataHeader, "received from", htons(buf->source));
   }

  /* if I am the base, pass the packet to the application port */
   if(is_base) {
    /* strip off the net layer header */
     for(i = CTP_DATA_HEADER_SIZE; i < buf->size; ++i)
       buf->data[i-CTP_DATA_HEADER_SIZE] = buf->data[i];
     buf->size -= CTP_DATA_HEADER_SIZE;
  
     return is_app_waiting_on(port);
   }

  /* Discard the packet if it is too old */
   if(dataHeader->thl < 100) 
     return false;

  /* otherwise consider forwarding to my next hop */
   fwd = NULL;
   mos_mutex_lock(&packetPoolMutex);       
   fwd = allocatePacket();
   mos_mutex_unlock(&packetPoolMutex); 
     
   mos_mutex_lock(&parentLock);     
   myEtx = bestEtx;
   mos_mutex_unlock(&parentLock);

   if (fwd != NULL) 
   {
    /* This node is not congested yet and able to forward this data packet */
     memcpy(fwd, buf, sizeof(comBuf));

    /* Extract the frame type*/
     dataHeader = (ctp_data_header_t*)fwd->data;

     if (dataHeader->options & CTP_OPT_PULL || dataHeader->options & CTP_OPT_ECN) {
        mos_mutex_lock(&beaconMutex);
        resetBeaconInterval();
        mos_mutex_unlock(&beaconMutex);
     }
     // looping detection
     if ( myEtx > ((dataHeader->etxHigh << 8) | (dataHeader->etxLow & 0xff)) ||
          myid == ((dataHeader->originHigh << 8) | (dataHeader->originLow & 0xff)) ) 
     {
      /* inconsistent path metrics */
        beaconFast();
     }

     dataHeader->thl++;

    /* store the PORT No. */
     fwd->source = port; 

     if(debug) {
       for(i = 0; i < myid_len; i++)
          fwd->data[fwd->size + myid_len - i - 1] = myid_toString[i];
       fwd->size += myid_len;
     }

     mos_mutex_lock(&sendQueueMutex);
     enqueuePacket(fwd);
     mos_mutex_unlock(&sendQueueMutex);
   }
   else 
   {
    /* This node is congested */
     printf("\nCTP STATS MSG: Sensor %d is congested. Dropping data packets.\n", myid);

    /* Broadcast my congestion status */
     mos_mutex_lock(&optionlock);
     options |= CTP_OPT_ECN;  // set congestion bit
     mos_mutex_unlock(&optionlock);
     beaconFast();
   }

  /* won't hand the packet to any application port */
   return false;
}

boolean ctp_handle_beacon_packet(comBuf* buf)
{
  uint16_t myEtx;
  ctp_routing_header_t* routingHeader = (ctp_routing_header_t*)buf->data;

  if(debug) printRTheader_recv(routingHeader, htons(buf->source));

 /* update my routing table */
  updateRoutingTable(htons(buf->source), routingHeader);

  if (routingHeader->options & CTP_OPT_PULL  || routingHeader->options & CTP_OPT_ECN) {
    mos_mutex_lock(&beaconMutex);
    resetBeaconInterval();
    mos_mutex_unlock(&beaconMutex);
  }

 /* Looping detection */ 
  if(routingHeader->parent == myid ) 
  {
    mos_mutex_lock(&parentLock);     
    myEtx = bestEtx;
    mos_mutex_unlock(&parentLock);

    if ( myEtx > ((routingHeader->etxHigh << 8) | (routingHeader->etxLow & 0xff)) ) 
    {
     /* inconsistent path metrics */ 
      beaconFast();
    }
  }
 /* won't hand the BEACON frame to any application */
  return false;
}


int8_t net_ctp_proto_send(comBuf *buf, va_list args) 
{
  uint8_t frame_type;
  uint8_t add_ctp_header;  

  ctp_data_header_t *   header;
  ctp_routing_header_t* rh;

  uint16_t dest, myEtx;
  uint8_t i, entries, index;
  
  if(buf==NULL) {
   /* a NULL packet will not actually sent out on link layer */
    return 0;
  }

 /* grab function parameters */
  add_ctp_header = (uint8_t)va_arg(args, int);
  frame_type = (add_ctp_header == CTP_NEW_PACKET)? (uint8_t)va_arg(args, int): buf->data[0];

  if(add_ctp_header == CTP_NEW_PACKET && frame_type == AM_CTP_DATA)
  {
   /* bail if packet is too big */
    if((buf->size + CTP_DATA_HEADER_SIZE) > COM_DATA_SIZE) {
      printf("CTP ERROR MSG: payload is too big.");
      return 1;
    }

   /* Shift the payload to insert the CTP header*/
    for(i = 1; i <= buf->size; i++)
      buf->data[buf->size + CTP_DATA_HEADER_SIZE -i] = buf->data[buf->size -i];
    buf->size = CTP_DATA_HEADER_SIZE + buf->size;
  }

  if(frame_type == AM_CTP_DATA) 
  {
    mos_led_toggle(2);
   /* Extract the CTP Data frame header */
    header = (ctp_data_header_t*)buf->data;  
      
    if(add_ctp_header == CTP_NEW_PACKET)
    {
      header->type = AM_CTP_DATA;      
      header->originSeqNo = (++originSeqNo);
      header->originHigh = myid >> 8;
      header->originLow= myid & 0xff;
      header->thl = 0;
      header->collectId = AM_OSCILLOSCOPE; 
    }

   /* get my bestEtx and nextHop, as well as the SeqNo for the current data packet */
    mos_mutex_lock(&parentLock);
    dest = nextHop;
    myEtx = bestEtx;      
    mos_mutex_unlock(&parentLock); 

    header->options = getOptions();
    if(header->options & CTP_OPT_ECN) myEtx = 0xffff;

    header->etxHigh = myEtx >> 8;
    header->etxLow = myEtx & 0xff;
    
   /* store nexthop addr in this unused comBuf field */
    buf->signal = htons(dest);        
    if(debug && dest!=BROADCAST_ADDR) printDTheader(header, "sent to", dest);    
    
   /* if routing is not found yet, data packet cannot be sent out. */
    return (dest == BROADCAST_ADDR ? 1 : 0);   
  }
  else if (frame_type == AM_CTP_ROUTING) 
  {
    mos_led_toggle(3);
   /* Beacon frame always has no payload*/

   /* Extract the CTP BEACON frame header */
    rh = (ctp_routing_header_t*)buf->data;

    if(add_ctp_header == CTP_NEW_PACKET)
    {
      rh->type = AM_CTP_ROUTING;
      rh->seqno = (beaconSeqNo++);
    }

    mos_mutex_lock(&parentLock); 
   /* Populate Leep Entries */
    for (i = 0, entries = 0; i < MAX_NEIGHBORS && entries <= MAX_LIENTRIES; i++) 
    {
      index = (lastParent + i) % MAX_NEIGHBORS;
      
      if (neighbors[index].addr != BROADCAST_ADDR) {
       /* copy node id of the neighbor */
        rh->data[3 * entries] = (neighbors[index].addr >> 8);  //node_idHigh  
        rh->data[3 * entries + 1] = (neighbors[index].addr & 0xff); //node_idLow
        
       /* copy neighbor's in-bound lqi (lqi from the neighbor to me) */
        rh->data[3 * entries + 2] = (uint8_t)neighbors[index].ib_lqi; 

        entries++;
      }
    }
    lastParent = (lastParent + i) % MAX_NEIGHBORS; 
    rh->leepEntries = entries << 4;
    buf->size = CTP_ROUTING_HEADER_SIZE + LEEP_ENTRY_SIZE*entries;

   /* get my bestEtx and nextHop */
    dest  = nextHop;
    myEtx = bestEtx;
    mos_mutex_unlock(&parentLock); 

    rh->options = getOptions();   

    if(rh->options & CTP_OPT_ECN) myEtx = 0xffff;
    rh->etxHigh = myEtx >> 8;
    rh->etxLow = myEtx & 0xff;
    rh->parent = dest;

   /* store nextHop address in this unused comBuf field */
    buf->signal = BROADCAST_ADDR; //always BROADCASTing beacon

    if(debug) printRTheader_send(rh);  
  }
  else {
    printf("CTP ERROR MSG: unknown frame type, cannot send out the packet.\n");
    return 1;
  }

 /* send it */
  return 0;
}


boolean net_ctp_proto_recv(comBuf *buf, uint8_t **footer, uint8_t port) 
{
  uint8_t *bytes, realrssi;
       
  if(buf==NULL) {
    printf("RADIO RECV Interface busy.\n"); 
   /* prevent net_recv_packet() from freeing a NULL packet*/
    return true;
  }
  if(htons(buf->source) == myid) {
    if(debug) printf("discard loopbacked packet.\n");
   /* Loopback packet. free it. */
    return false;
  }

 /* Extract the rssi from "link quality indice" reported by link layer */
  bytes  = (uint8_t *) &(buf->signal);      
  realrssi = (int8_t)(bytes[0]) + 128;

 /* The first byte of the header indicates the frame type*/
  if (buf->data[0] == AM_CTP_DATA)
  {
    mos_led_toggle(1);    

    if (realrssi < RSSI_THRESHOLD)
    {
      if(debug) 
        printf("bad rssi %d, discard the DATA frame from %d\n", realrssi, htons(buf->source));
    }
    else
      return ctp_handle_data_packet(buf, port);
  }
  else if (buf->data[0] == AM_CTP_ROUTING) 
  {
    mos_led_toggle(0);    
      
    if (realrssi < RSSI_THRESHOLD) 
    {
      if(debug) 
        printf("bad rssi %d, discard the BEACON frame from %d", realrssi, htons(buf->source));
    }
    else
      return ctp_handle_beacon_packet(buf);
  }
  else {
    printf("Unexpected error: unknow frame type received from %d.\n", htons(buf->source));
  }
   
 /* free packet memory */
  return false;
}


int8_t net_ctp_proto_ioctl(uint8_t request, va_list args) {    
  uint16_t power;

 /* do requested action */
  switch(request) {
   /* make this sensor a base station */
    case CTP_SET_IS_BASE:      
      is_base = true;
      parent  = 0;
      bestEtx = 0;
      nextHop = myid;
      printf("Set sensor %d to be the base station.\n", mos_node_id_get());
      break;
   /* make this sensor a node */    
    case CTP_SET_IS_NODE:
      is_base = false;
      parent  = 0;
      bestEtx = 0xffff;
      nextHop = BROADCAST_ADDR;
      printf("Set sensor %d to be a node.\n", mos_node_id_get());
      break;
   /* set sensor's RF power level */   
    case CTP_SET_POWER:
      power = (uint16_t)va_arg(args, int);
#ifdef PLATFORM_TELOSB
      com_ioctl_IFACE_RADIO(CC2420_TX_POWER, power);
      printf("Set sensor %d power level to be %04x\n", mos_node_id_get(), power);
#else
      printf("SET_POWER option is now supported on only PLATFORM_TELOSB.\n");
#endif
      break;
   /* reduce the senor's RF power to the lowest */       
    case CTP_SET_TESTBED:
#ifdef PLATFORM_TELOSB
      com_ioctl_IFACE_RADIO(CC2420_LOW_POWER_MODE);
      printf("Set sensor %d power level to be the lowest.\n", mos_node_id_get());
#else
      printf("SET_POWER option is now supported on only PLATFORM_TELOSB.\n");
#endif
      break;         
    default:
      printf("Invalid CTP IOCTL option.\n");
      return 1;
  }
  return 0;
}


uint32_t temp_in_app_send;
comBuf   dummyPacket;
comBuf   dataPacket;

void app_send()
{   
  myid_toString[0] = ' ';
  myid_toString[1] = (myid%10) + '0'; 
  myid_len = 2;
  temp_in_app_send = myid/10;
  while(temp_in_app_send && myid_len < 5) {
    myid_toString[myid_len] = (temp_in_app_send%10) + '0';
    temp_in_app_send /= 10;
    myid_len ++;
  }   

  if(debug) {
    dataPacket.size = 0;
    while(dataPacket.size < myid_len) {
      dataPacket.data[dataPacket.size] = myid_toString[myid_len-dataPacket.size -1];
      dataPacket.size++;
    }
  }
  else {
    dataPacket.data[0] = 'H';
    dataPacket.data[1] = 'I';
    dataPacket.data[2] = '\0';
    dataPacket.size = 3;
  }
  memcpy(&dummyPacket, &dataPacket, sizeof(comBuf));

  while (1) {
     printf("\napp_send thread\n");     
     temp_in_app_send = net_send(&dataPacket, CTP_PROTO_ID, CTP_LISTENING_PORT, CTP_NEW_PACKET, AM_CTP_DATA);
     //mos_led_toggle (1);
     if(temp_in_app_send) 
       if(debug) printf("route not found yet. cannot send out data packet.\n");

     memcpy(&dataPacket, &dummyPacket, sizeof(comBuf));
     mos_thread_sleep (1000);
  }  
}

void app_recv()
{
   comBuf * buffer;

   while (1)
   {
      printf("\napp_recv thread\n");
      buffer = net_recv(CTP_LISTENING_PORT);
      //mos_led_toggle (1);      
      if(debug) buffer->data[buffer->size-1] = '\0';
      printf("Received App packet. msg: %s\n", buffer->data);
      com_free_buf(buffer);
   }
}


void start(void)
{
  /* must start the net thread */
   net_init();
   
  /* start the CTP backends */
   ctp_proto_init();

  /* wait a while till the routing is possibly established */     
   mos_mdelay(10); 
  
   //net_ioctl(CTP_PROTO_ID, CTP_SET_IS_NODE);
   net_ioctl(CTP_PROTO_ID, CTP_SET_TESTBED);

  /* This is the basic app thread */
   if(is_base)
     mos_thread_new(app_recv, 384, PRIORITY_NORMAL);
   else
     mos_thread_new(app_send, 384, PRIORITY_NORMAL);  
}

#endif
