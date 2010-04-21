#include "ctp_plus.h"
#include "net.h"
#include "node_id.h"


#define b2w(high,low)           ((uint16_t)(((high)<<8)|((low)&0xff)))
#define computeETX(etx, ob_lqi) ((((ob_lqi) <= MAX_LQI) && ((uint16_t)(etx) < (0xffff-(ob_lqi)))) ? ((uint16_t)((etx) + (ob_lqi))) : 0xffff)    
#define min(x,y)                ((x)<(y)? (x):(y))
#define max(x,y)                ((x)>(y)? (x):(y))

#define BASE_ALIAS              0xffff

#define PRR_MSB_MASK            (1 << (PRR_WINDOWSIZE-1))
#define PRR_MASK                ((1 << PRR_WINDOWSIZE) -1)
#define PRR_HALF_WINDOWSIZE     (PRR_WINDOWSIZE >> 1)
#define PRR_RHALF_WINDOW        ((1 << PRR_HALF_WINDOWSIZE) -1)

#define RM_LIST_SIZE            3

uint8_t CTP_TRANSMISSION_TIMEOUT = (BEACON_INTERVAL * CTP_NETWORK_DIAMETER)/CTP_SEND_INTERVAL;

#ifdef MAX_RETRANSMIT_ATTEMPTS
uint8_t max_retransmit_attempts = MAX_RETRANSMIT_ATTEMPTS;
#else
uint8_t max_retransmit_attempts = 0;
#endif


boolean is_base = false;
boolean ctp_append_hops = true;

uint16_t myid;
uint32_t seed;

/* mutexes used by CTP */
mos_mutex_t sendMutex;
mos_mutex_t beaconMutex;
mos_mutex_t neighborTableLock;
mos_mutex_t optionlock;
mos_mutex_t dataQueueMutex;
mos_cond_t  dataQueueCond;
mos_mutex_t cacheMutex;

/*-- protected by cacheMutex --*/
#define CACHE_SIZE   10
uint8_t cache_tail = 0;
cache_entry_t cache[CACHE_SIZE];
/*-- protected by cacheMutex --*/

/*-- protected by dataQueueMutex --*/
uint8_t dataQueueHead = 0;
uint8_t dataQueueSize = 0;
data_queue_entry_t dataQueue[CTP_DATA_QUEUE_SIZE];
/*-- protected by dataQueueMutex --*/

/*-- protected by optionLock --*/
uint8_t options = CTP_OPT_PULL;
/*-- protected by optionLock --*/

/*-- protected by beaconMutex --*/
comBuf beaconPacket;
uint16_t beaconTimer = BEACON_TIMER_MAX;
uint16_t beaconTimerCount = BEACON_TIMER_MAX;
uint8_t beaconSeqNo = 0;
/*-- protected by beaconMutex --*/

/*-- protected by neighborTableLock --*/
// neighbor table
routing_entry_t neighbors[MAX_NEIGHBORS];
uint16_t bestETX;
uint16_t nextHop;
uint8_t  parent;
uint16_t meanETX[MAX_NEIGHBORS];
uint16_t meanETXdl[MAX_NEIGHBORS][MAX_NUM_DEST];

// downstream routing table
destnode_t dest_nodes[MAX_NUM_DEST];  
uint8_t  num_dest;

/*-- Xmit power optimization -- */
#ifdef CROSS_LAYER
uint16_t dlPower = 15; // max power 31, midrange = 15
uint16_t ulPower = 15; // max power 31, midrange = 15
#endif

// beacon system for downstream routings
uint8_t  destforbeacon[MAX_NUM_DEST];

uint8_t originSeqNo = 0;
uint8_t lastParent = 0;

uint16_t rm_list[RM_LIST_SIZE];
uint8_t  rm_list_head, rm_list_size;
/*-- protected by neighborTableLock --*/

#ifdef CTP_PLUS_DEBUGt
#include "ctp_plus_debug.h"
#define DEBUG_STACK_SIZE     128
static stackval_t debug_stack[DEBUG_STACK_SIZE];
#endif

#if defined(CTP_PLUS_DATA_DEBUG) || defined(CTP_PLUS_ROUTING_DEBUG) || defined(CTP_PLUS_PRINT_FOOTER)
#include "ctp_plus_debug.h"
#endif



typedef enum CTP_DropCondition {DATA_OVERFLOW = 0, DATA_TXFAIL, DATA_TIMEOUT, PHY_ERROR} CTP_DropCondition;
void handle_dataQ_overflow(const comBuf * buf, CTP_DropCondition status);
uint8_t enqueueData(const comBuf* buf, uint8_t port, uint8_t attempts);
uint8_t dequeueData();


void initNeighborEntry(routing_entry_t * ent)
{
  ent->addr = BROADCAST_ADDR;
  ent->etx = 0xffff;
  ent->ib_lqi = 0xff;  
  ent->ob_lqi = 0xff;  
  ent->lastrecvdseqno = 0;
  ent->prr = 0; 
  ent->d_prr = 0;  
  ent->freshness = 0;  
  ent->flag = 0;
  
  uint8_t i;
  for(i = 0; i < MAX_NUM_DEST; ++i)    
    ent->r_etx[i] = 0xffff;
}

void initDLDestEntry(destnode_t * destEnt)
{
  if(num_dest && (destEnt->id != BROADCAST_ADDR)) 
      --num_dest;

  if(destEnt->r_parent < MAX_NEIGHBORS) 
    if(neighbors[destEnt->r_parent].flag)
      --neighbors[destEnt->r_parent].flag;	
	
  destEnt->id = BROADCAST_ADDR;
  destEnt->freshness = 0;
  destEnt->r_parent  = MAX_NEIGHBORS;
  destEnt->r_bestETX = 0xffff;
  destEnt->r_nextHop = BROADCAST_ADDR;
  destEnt->inqueue   = 0;   
  
  destEnt->beaconTimer = BEACON_TIMER_MIN;
  destEnt->beaconTimerCount = BEACON_TIMER_MIN;  

  uint8_t i, j;
  for(i = 0, j = (destEnt - dest_nodes); i < MAX_NEIGHBORS; ++i) {
     neighbors[i].r_etx[j] = 0xffff;    
  }  
}

void initDataQueueEntry(data_queue_entry_t * dent)
{ 
	memset(dent, 0, sizeof(data_queue_entry_t));	
  // memset(dent, 0xff, sizeof(data_queue_entry_t));
}

uint8_t chooseParent(uint8_t dest_indx)
{
  uint8_t i, j;
  uint16_t etx, getbestetx;
  
 /* (dest_indx >= MAX_NUM_DEST) implies that we are computing forward ETX to Base */  
  dest_indx = min(dest_indx, MAX_NUM_DEST);
  
  for (i = 0, getbestetx = 0xffff, j= MAX_NEIGHBORS; i < MAX_NEIGHBORS; ++i) 
  {
   /* ETX to downstream destination or ETX to base */ 
    etx = (dest_indx < MAX_NUM_DEST) ? neighbors[i].r_etx[dest_indx] : neighbors[i].etx;
    
    etx = computeETX(etx, neighbors[i].ob_lqi);    
    if (etx < getbestetx) {
      getbestetx = etx;
      j = i;
    }
  }  
  return j;  
}


/* Update my parent, bestEtx and nextHop for upstream cast */
void updateForwardParent() 
{
  uint8_t j;

 /* update parent for upstream cast */  
  j = chooseParent(MAX_NUM_DEST); 
  if(j < MAX_NEIGHBORS) 
  {
    /* ETX increased by >= 1 transmission or parent is changed*/
    if((computeETX(neighbors[j].etx, neighbors[j].ob_lqi) > bestETX) || (j != parent))
    {
      mos_mutex_lock(&beaconMutex);
      beaconFast(NULL);
      mos_mutex_unlock(&beaconMutex);
    } 
    
    bestETX = computeETX(neighbors[j].etx, neighbors[j].ob_lqi);        
    nextHop = bestETX > MAX_ETX ? BROADCAST_ADDR : neighbors[j].addr;  
    if(bestETX > MAX_ETX) bestETX = 0xffff;
  }     
  else {
    bestETX = 0xffff;      
    nextHop = BROADCAST_ADDR;
  }    
  if (j != parent) {
  	/* update parent's flag */
    if(j < MAX_NEIGHBORS) 
      ++neighbors[j].flag;
    if(parent < MAX_NEIGHBORS && neighbors[parent].flag)
      --neighbors[parent].flag;
    
    /* change the parent */  
    parent = j;   
    
#ifdef CTP_PLUS_DEBUGt    
    printf("parent changed. current bestETX %d, current nextHop %d\n", bestETX, nextHop);
#endif    
  }

  mos_mutex_lock(&optionlock);
  /* set up PULL bit if there is no valid route to base */
  if(nextHop == BROADCAST_ADDR) {    
    options |= CTP_OPT_PULL;     // set PULL bit
  }
  else {
    options &= (~CTP_OPT_PULL);  // unset PULL bit
  }
  mos_mutex_unlock(&optionlock);   
  
}

void updateReverseParent(uint8_t i)
{
   uint8_t j;

   if(i >= MAX_NUM_DEST) return;
   
   j = chooseParent(i);  
   if(j < MAX_NEIGHBORS) 
   {
      /* ETX increased by >= 1 transmission or parent is changed */
      if((computeETX(neighbors[j].r_etx[i], neighbors[j].ob_lqi) > dest_nodes[i].r_bestETX) || (j != dest_nodes[i].r_parent))
      {
         beaconFast(&dest_nodes[i]);
      } 
      
      dest_nodes[i].r_bestETX = computeETX(neighbors[j].r_etx[i], neighbors[j].ob_lqi);        
      dest_nodes[i].r_nextHop = dest_nodes[i].r_bestETX > MAX_ETX ? BROADCAST_ADDR : neighbors[j].addr;
      if(dest_nodes[i].r_bestETX > MAX_ETX) dest_nodes[i].r_bestETX = 0xffff;
   }     
   else {
      dest_nodes[i].r_bestETX = 0xffff;      
      dest_nodes[i].r_nextHop = BROADCAST_ADDR;
   } 
   if (j != dest_nodes[i].r_parent) {
   	 /* update the downstream route parent's flag */
     if(j < MAX_NEIGHBORS) 
       ++neighbors[j].flag;
     if(dest_nodes[i].r_parent < MAX_NEIGHBORS && neighbors[dest_nodes[i].r_parent].flag)
       --neighbors[dest_nodes[i].r_parent].flag;
       
     /* change the parent for this downstream route */  
     dest_nodes[i].r_parent = j;     

#ifdef CTP_PLUS_DEBUGt      
     printf("parent for dest %d changed. current bestETX %d, current nextHop %d\n", 
                dest_nodes[i].id, dest_nodes[i].r_bestETX, dest_nodes[i].r_nextHop);
#endif    
   }
}

/* Update my parent, bestEtx and nextHop for downstream cast */
void updateAllReverseParents() 
{
  uint8_t i;

 /* update parents for downstream cast to different destination nodes */ 
  for(i = 0, num_dest = 0; i < MAX_NUM_DEST ; ++i) 
  {
   /* skip the empty entry */	
    if(dest_nodes[i].id == BROADCAST_ADDR) continue;
    
   /* number of the destionations we are currently tracking */
    ++num_dest;    
    
    updateReverseParent(i);
  }  
}

#ifdef CTP_PLUS_DEBUGt
  #define PRINT_INTERVAL  40
  uint8_t beacon_print_timer = PRINT_INTERVAL;
#endif

void beacon_thread(void) {
  static uint8_t i, j, prev_options = 0; 
  static uint16_t ul_nexthop; 
  static uint8_t parentChanged[(MAX_NUM_DEST>>3) + 1], flags, ulBeaconRequest, dlBeaconRequest;
  
  while(1) {
  	
#ifdef CTP_PLUS_DEBUGa  	
    printBeaconTimer();  
#endif
     
   /* minimal beacon interval (ms) */
    mos_thread_sleep(BEACON_INTERVAL);

   /* update routingTable with freshness and parent */
    mos_mutex_lock(&neighborTableLock);
    
   /* fresh the reverse routing table */
    if(num_dest > 0) {
      for(i = 0; i < MAX_NUM_DEST; ++i) {
      	/* skip the empty dest entry */
        if(dest_nodes[i].id == BROADCAST_ADDR)
          continue;

        if (dest_nodes[i].freshness > 0) 
           --dest_nodes[i].freshness;
          
        if(dest_nodes[i].beaconTimer > 0)
           --dest_nodes[i].beaconTimer;       
      }
    }    
    
   /* refresh the neighbor table */   
    bzero(parentChanged, sizeof(parentChanged));
    for (i = 0; i < MAX_NEIGHBORS; ++i) 
    {
    	/* skip the empty neighbor entry */
      if (neighbors[i].addr == BROADCAST_ADDR)
        continue;
        
      if (neighbors[i].freshness == 0) 
      {
        flags = neighbors[i].flag;
        if(flags && parent == i) {
          parentChanged[MAX_NUM_DEST >> 3] |= (1 << (MAX_NUM_DEST & 0x07));
          --flags;
        }
        for(j = 0; j < MAX_NUM_DEST && flags; ++j) 
        {
          if(dest_nodes[j].r_parent == i) 
          {
            parentChanged[j >> 3] |= (1 << (j & 0x07));
            --flags;
          }
        }        
        initNeighborEntry( &neighbors[i]); 
      }
      else {
        --neighbors[i].freshness;
      }      
    }  
   /* choose New Forward Parent, only applied to node */
    if (!is_base && (parentChanged[MAX_NUM_DEST >> 3] & (1 << (MAX_NUM_DEST & 0x07))) ) {     
      updateForwardParent();      
    }    
    ul_nexthop = nextHop;
    
   /* choose New Reverse Parents */        
    for(i = 0; i < MAX_NUM_DEST; ) {     
      if(parentChanged[i >> 3] == 0) {
      	i += 8;
      	continue;
      }	
      if(parentChanged[i >> 3] & (1 << (i & 0x07)))
        updateReverseParent(i);           
      ++i;  
    }    

#ifdef CTP_PLUS_DEBUGt        
    if((beacon_print_timer--)==0) {printTable(neighbors,"Updated freshness", 0); beacon_print_timer=PRINT_INTERVAL;}       
#endif    
    mos_mutex_unlock(&neighborTableLock);   
    
    mos_mutex_lock(&optionlock);         
    /* node's condition changes */
    ulBeaconRequest = ((options & CTP_OPT_PULL) || ((prev_options ^ options) & CTP_OPT_ECN));  
    prev_options = options;
    mos_mutex_unlock(&optionlock);         
    
   /* update beacon system for upstream cast to base */  
    mos_mutex_lock(&beaconMutex);   
    if(beaconTimer > 0) {
       --beaconTimer;   
    }
    if (ul_nexthop == BROADCAST_ADDR) {
      resetBeaconInterval(NULL);
    }                  
    if (beaconTimer <= 0) {
      ulBeaconRequest |= 0x01;

      if (ul_nexthop != BROADCAST_ADDR) {
        increaseBeaconInterval(NULL);
      }
    }      
    mos_mutex_unlock(&beaconMutex);   


   /* update beacon system for each downstream destination */ 
    mos_mutex_lock(&neighborTableLock);

    dlBeaconRequest = (rm_list_size > 0) ? min(rm_list_size, 2) : 0;
    bzero(destforbeacon, sizeof(destforbeacon));
    if(num_dest > 0) {
      for(j = 0, i = 0; (j < MAX_NUM_DEST * 2) && (dlBeaconRequest < MAX_DESTENTRIES); ++j, i = j % MAX_NUM_DEST) 
      { 
        if((j == MAX_NUM_DEST) && ((dlBeaconRequest | ulBeaconRequest) == 0)) 
           break;
        /* if it is the empty entry or the beacon request for this entry is already registered */
        if((dest_nodes[i].id == BROADCAST_ADDR) || destforbeacon[i])
           continue;
        /* if it is a replaceable entry */        
        if (dest_nodes[i].freshness == 0)
           continue;        

        /* beacon aggregation */
        if(j < MAX_NUM_DEST)
        {           
           if ((dest_nodes[i].r_nextHop == BROADCAST_ADDR) && (dest_nodes[i].freshness > NEW_DL_FRESH - 5)) {
                resetBeaconInterval(&dest_nodes[i]);
           }           

           if(dest_nodes[i].beaconTimer <= 0)
           {
             ++dlBeaconRequest;
             destforbeacon[i] = 1;                         

             if(dest_nodes[i].r_nextHop != BROADCAST_ADDR) {
                increaseBeaconInterval(&dest_nodes[i]);
             }
           }           
        }
        else if (dest_nodes[i].beaconTimer < 8) 
        {
           ++dlBeaconRequest;
           destforbeacon[i] = 1;
           if (dest_nodes[i].r_nextHop != BROADCAST_ADDR) 
              increaseBeaconInterval(&dest_nodes[i]);           
        }   
      }
    }

    mos_mutex_unlock(&neighborTableLock);      
       
    if(ulBeaconRequest || dlBeaconRequest)
    {
      mos_mutex_lock(&sendMutex);
      if(dlBeaconRequest) 
        /* produce a beacon packet including both upstream and downstream info */      
        net_send(&beaconPacket, CTP_PROTO_ID, CTP_DUMMY_PORT, CTP_NEW_PACKET, AM_CTP_DL_ROUTING);   
      else 
        /* produce a normal beacon packet if there are no downdream beacons needed */ 
        net_send(&beaconPacket, CTP_PROTO_ID, CTP_DUMMY_PORT, CTP_NEW_PACKET, AM_CTP_ROUTING);
      mos_thread_sleep((rand_mlcg_8() & 0x1f) + 8);
      mos_mutex_unlock(&sendMutex);  
    } 
  }
}

uint8_t inline getNeighborIndex(uint16_t addr)
{
  uint8_t i;
  for(i = 0; i < MAX_NEIGHBORS && (neighbors[i].addr != addr); ++i);
  return i;
}

uint8_t inline getDestIndex(uint16_t dest) 
{
  uint8_t i;
  for(i = 0; i < MAX_NUM_DEST && (dest_nodes[i].id != dest); ++i);
  return i;
}

uint8_t insertNewDest(uint16_t dest)
{
  uint8_t i = MAX_NUM_DEST, j;
  
  /* If dest is a broadcast address, no need to insert it to the table */
  if(dest == BROADCAST_ADDR)
     return MAX_NUM_DEST;

  /* Check if dest_nodes table has empty entry */
  if((i = getDestIndex(BROADCAST_ADDR)) >= MAX_NUM_DEST) {

    /* Otherwise, seek an expired/replaceable entry. Entry with invalid route is preferred. */
    for(i = 0; i < MAX_NUM_DEST && dest_nodes[i].freshness > 0; ++i);
    if(i < MAX_NUM_DEST - 1) {
       for(j = i; j < MAX_NUM_DEST && (dest_nodes[j].freshness || (dest_nodes[j].r_nextHop != BROADCAST_ADDR)); ++j); 
       if(j < MAX_NUM_DEST) {
         i = j;
       } 
    }  
    /* seek the route entry that is not to be used by any packets in the forwarding data queue */
    if(i >= MAX_NUM_DEST) {
      for(i = 0; i < MAX_NUM_DEST && dest_nodes[i].inqueue; ++i);
      if(i < MAX_NUM_DEST - 1) {
        for(j = i; (j < MAX_NUM_DEST) && (dest_nodes[j].inqueue || (dest_nodes[j].r_nextHop != BROADCAST_ADDR)); ++j);
        if(j < MAX_NUM_DEST)
           i = j;
      }   
    }
  }
  if(i < MAX_NUM_DEST) {  	
    if((dest_nodes[i].id != BROADCAST_ADDR) && (dest_nodes[i].r_nextHop != BROADCAST_ADDR)) {
    	 /* Broadcast the removal of an existing dest entry with valid route in the near future */
       if(rm_list_size < RM_LIST_SIZE) 
       {
          for(j = 0; (j < rm_list_size) && (rm_list[(rm_list_head + j) % RM_LIST_SIZE] != dest_nodes[i].id); ++j);
          if(j == rm_list_size) {
             rm_list[(rm_list_head + j) % RM_LIST_SIZE] = dest_nodes[i].id;
             ++rm_list_size;
          }
       }
       /* cannot remove this entry for now due to the overflow of to-be-removed list */
       else {       	  
          return MAX_NUM_DEST;
       }
    }
    initDLDestEntry(&dest_nodes[i]);
    ++num_dest;

    dest_nodes[i].id = dest;
    dest_nodes[i].freshness = NEW_DL_FRESH;

    if((j = getNeighborIndex(dest)) < MAX_NEIGHBORS)
      neighbors[j].r_etx[i] = 0;    
  }
  /* return the index of inserted new dest, or MAX_NUM_DEST if the insertion fails. */
  return i;
}


void updateRoutingTable_sup(uint16_t source, ctp_data_header_t* rh) {
  uint8_t j;
  uint16_t addr; 
  
  mos_mutex_lock(&neighborTableLock);

  if((j = getNeighborIndex(source)) < MAX_NEIGHBORS)
  {
     neighbors[j].freshness = NEW_FRESH;

     /* Error correction */
     if(rh->thl == 0) 
     {  // get originator
        addr = b2w(rh->originHigh, rh->originLow);
        if((addr != source) && (getNeighborIndex(addr) >= MAX_NEIGHBORS)) 
        {
          rh->originHigh = (source >> 8) & 0xff;
          rh->originLow  = source & 0xff;
#ifdef ENABLE_CTP_PRINTFS
printf("\n!!! corrected originator %d -> %d\n", addr, b2w(rh->originHigh, rh->originLow));
#endif
        }     
     }
  }    
  mos_mutex_unlock(&neighborTableLock);  
}


/* investigate the attached leep frame at the footer */
uint8_t extractLqi(uint8_t nentry, const uint8_t * data, uint8_t nindex)
{
  uint8_t i, j, lqi = 0;
  leep_entry_t * et;  
  
 /* Be careful not to exceed MAX_LIENTRIES */
  for(i = 0 ; i < min( nentry, MAX_LIENTRIES ); i++) {
    et = (leep_entry_t *)(data + i * 3);
    /* filtered the obviously corrupted lqi value */
    if(et->lq < HOP_COUNT)
      continue;
    if( b2w( et->node_idHigh, et->node_idLow) == myid ) {
      lqi = et->lq;        
    }
    else if((j = getDestIndex(b2w( et->node_idHigh, et->node_idLow))) < MAX_NUM_DEST) {
    	if(dest_nodes[j].r_nextHop == BROADCAST_ADDR)
         neighbors[nindex].r_etx[j] = et->lq;
    }  
  }
  return lqi;  
} 

/* compute moving average of LQI */
uint8_t inline compute_MAV_LQI(uint8_t lqi1, uint8_t lqi2)
{
  /* if newest lqi is too bad, invalidate the lqi */
  if(lqi2 > MAX_LQI)
    return 0xff;
    
  lqi1 = min(lqi1, MAX_LQI+1);
  lqi1 = ((MAV_ALFA * lqi1 + MAV_BETA * lqi2 + 5) / 10 ) & 0xff;
  return (lqi1 > MAX_LQI) ? 0xff : lqi1;
}

void updateReverseETX(const ctp_dl_routing_header_t * rrh, routing_entry_t * ent)
{
   uint8_t i, j, k;
   uint16_t dest;
   dest_entry_t * dt;
   
  /* If rrh->leepEntries is not a valid number, don't bother then. */
   if( (rrh->leepEntries>>4) > MAX_LIENTRIES)
     return;
   
   k = (rrh->leepEntries >> 4) * 3;
   for(i = 0 ; i < (min(rrh->destEntries, MAX_DESTENTRIES)); i++) {
     dt = (dest_entry_t *)&(rrh->data[k + i * 6]);
     dest = b2w( dt->node_idHigh, dt->node_idLow ); 

     if((dest == myid) || (dest == BROADCAST_ADDR))
       continue;

    /* If the dest is already in the table, update it */
     if( (j = getDestIndex(dest)) < MAX_NUM_DEST) {

       ent->r_etx[j] = b2w(dt->r_parentHigh, dt->r_parentLow) == myid ? 0xffff : b2w(dt->r_etxHigh, dt->r_etxLow);
     }      
    /* else get an available downstream routing entry, if there's any */
     else if(!is_base) {
       /* removal message signature: (dest, parent, ETX) == (dest, dest, 0xffff)*/
       if((b2w(dt->r_parentHigh, dt->r_parentLow) == dest) && (b2w(dt->r_etxHigh, dt->r_etxLow) > MAX_ETX))
         continue;  
       /* route request message signature: (dest, parent, ETX) == (dest, 0xffff, 0xffff) */  
       if( (j = insertNewDest(dest)) < MAX_NUM_DEST) {  
         ent->r_etx[j] = b2w(dt->r_parentHigh, dt->r_parentLow)==myid ? 0xffff : b2w(dt->r_etxHigh, dt->r_etxLow);
       }
     }                                                          
   }
}

uint8_t validateSource(uint16_t source, const ctp_routing_header_t* rh)
{
  uint8_t i, k;
  leep_entry_t * et;  
  dest_entry_t * dt;
  ctp_dl_routing_header_t * rrh;

  if((b2w(rh->parentHigh, rh->parentLow) == source) && (b2w(rh->etxHigh, rh->etxLow) != 0) && (!(rh->options & CTP_OPT_ECN))) {
#ifdef ENABLE_CTP_PRINTFS
	  printf("\n!#! error parent\n"); 
#endif
	  return 1; 
  }
   
  k = rh->leepEntries>>4;
  if(k > MAX_LIENTRIES) {
#ifdef ENABLE_CTP_PRINTFS
     printf("\n!#! erroreous number of leep entries\n"); return 1;
#endif
  } 

  for(i = 0 ; i < min(k, MAX_LIENTRIES); ++i) {
    et = (leep_entry_t *)(rh->data + i * 3);

    if( b2w( et->node_idHigh, et->node_idLow ) == source ) {
#ifdef ENABLE_CTP_PRINTFS
        printf("\n!#! erroreous neighbor\n");
#endif
        return 1;      
    }  
  }  
  if(rh->type == AM_CTP_DL_ROUTING) {
    rrh = (ctp_dl_routing_header_t *) rh;    
    
    if(rrh->destEntries > MAX_DESTENTRIES) {
#ifdef ENABLE_CTP_PRINTFS
       printf("\n!#! erroreous number of dest entries\n"); 
#endif
       return 1;
    }     	
    k *= 3;
    for(i = 0 ; i < min(rrh->destEntries, MAX_DESTENTRIES); i++) {
      dt = (dest_entry_t *)&(rrh->data[k + i * 6]);

      if((b2w(dt->node_idHigh, dt->node_idLow) == source) || (b2w(dt->r_parentHigh, dt->r_parentLow) == source)) {
#ifdef ENABLE_CTP_PRINTFS
         printf("\n!#! errroreous dest/parent\n");
#endif
         return 1;
      }   
    }
  }  
  return 0;  
}

void updateRoutingTable(uint16_t source, const ctp_routing_header_t* rh) {
  uint8_t i, j, lqi = 0;
  uint16_t worstMetric;    
  ctp_dl_routing_header_t * rrh;    

  if(source == BROADCAST_ADDR) return;

  mos_mutex_lock(&neighborTableLock);
 /* If the neighbor is already in the table, update it */
  i = getNeighborIndex(source);
  
 /* If it's not in the table, find an empty entry or 
    replace an entry with a worst ETX if this one is better. */  
  if (i == MAX_NEIGHBORS) {       
    worstMetric = (num_dest == 0) ? b2w( rh->etxHigh, rh->etxLow ) : NEW_FRESH;     
    
    for (j = 0; j < MAX_NEIGHBORS; j++) {     
     /* get an empty(unused) table entry */
      if (neighbors[j].addr == BROADCAST_ADDR) {
        i = j;        
        break;
      }
      if(num_dest == 0) {
       /* only applied to node */ 
        if (!is_base && worstMetric < neighbors[j].etx) {
          worstMetric = neighbors[j].etx;
          i = j;
        }
      }
      else {
        if( (neighbors[j].flag == 0) && (worstMetric > neighbors[j].freshness)) {        
          worstMetric = neighbors[j].freshness;
          i = j;
        }
      }
    } 
    if (i < MAX_NEIGHBORS) {  
      if(neighbors[i].addr != BROADCAST_ADDR)
        initNeighborEntry( &neighbors[i] );
        
      neighbors[i].addr = source;
      if((j = getDestIndex(source)) < MAX_NUM_DEST)
        neighbors[i].r_etx[j] = 0;
    }
  }
  if (i < MAX_NEIGHBORS) {    
    neighbors[i].freshness = NEW_FRESH;

   /* Update PRR */
    neighbors[i].prr  = (neighbors[i].prr) >> ((rh->seqno > neighbors[i].lastrecvdseqno) ? 
                 (rh->seqno-neighbors[i].lastrecvdseqno) : (0xff-neighbors[i].lastrecvdseqno + rh->seqno+1)); 
    neighbors[i].prr |= PRR_MSB_MASK;    // set the msb to be 1
    neighbors[i].lastrecvdseqno = rh->seqno;
   
   /* Update my in-bound lqi from neighbor[i] to me */
    for(j = (neighbors[i].prr >> PRR_HALF_WINDOWSIZE), lqi = (neighbors[i].prr & PRR_RHALF_WINDOW); j; j >>= 1)
       lqi += ((j & 0x01) << PRR_HALF_WINDOWSIZE);
    neighbors[i].ib_lqi = (lqi == 0) ? 0xff : ((MAX_LINK_ETX + (lqi >> 1))/lqi + HOP_COUNT);
    
    if(rh->type == AM_CTP_ROUTING) {            
     /* Update my out-bound lqi to neighbor[i].addr */     
      if( (lqi = extractLqi((rh->leepEntries) >> 4, &(rh->data[0]), i) ) > 0 ) {
        neighbors[i].ob_lqi = compute_MAV_LQI(neighbors[i].ob_lqi, lqi);  
      }
     
     /* only applied to node */
      if(!is_base) {
       /* Update neighbor's ETX */
      	neighbors[i].etx = b2w(rh->parentHigh, rh->parentLow) == myid ? 0xffff : b2w(rh->etxHigh, rh->etxLow);
        
       /* Update my parent, bestEtx and nextHop */ 
        updateForwardParent();
      } 
     /* Update reverse parent, bestEtx and nextHop */
      updateAllReverseParents();      
    }  
    else if(rh->type == AM_CTP_DL_ROUTING) { 
      rrh = (ctp_dl_routing_header_t *) rh;       
      if( (lqi = extractLqi((rrh->leepEntries) >> 4, &(rrh->data[0]), i) ) > 0 ) {
        neighbors[i].ob_lqi = compute_MAV_LQI(neighbors[i].ob_lqi, lqi);
      }
            
      if(!is_base) {
      	neighbors[i].etx = b2w(rrh->parentHigh, rrh->parentLow) == myid ? 0xffff : b2w(rrh->etxHigh, rrh->etxLow);
        updateForwardParent(); 
      }           
     /* Update my reverse ETX list */ 
      updateReverseETX( rrh, &neighbors[i]);
      
     /* Update reverse parent, bestEtx and nextHop */
      updateAllReverseParents();
    }
  }   
#ifdef CTP_PLUS_DEBUGt  
  printTable(neighbors, rh->type == AM_CTP_DL_ROUTING ? "recvd a DL RT from":"recvd a RT from", source);    
#endif  

  mos_mutex_unlock(&neighborTableLock);
 
}

/* update data driven PRR and LQI */
void update_DLQI(uint16_t nexthop, uint8_t num_retx_attempts)
{
  uint8_t i = 0, j, lqi;

  mos_mutex_lock(&neighborTableLock);
  /* If the neighbor is already in the table, update it */
  if( (nexthop != BROADCAST_ADDR) && ((i = getNeighborIndex(nexthop)) < MAX_NEIGHBORS) )
  {
    if(num_retx_attempts < max_retransmit_attempts)
       neighbors[i].freshness = NEW_FRESH;
       
    /* Update DATA DRIVEN PRR */
    neighbors[i].d_prr  >>= min(num_retx_attempts, max_retransmit_attempts) + 1; 
    if(num_retx_attempts <= max_retransmit_attempts) 
       neighbors[i].d_prr |= PRR_MSB_MASK;    // set the msb to be 1    

    /* Update my out-bound lqi */
    for(j = (neighbors[i].d_prr >> PRR_HALF_WINDOWSIZE), lqi = (neighbors[i].d_prr & PRR_RHALF_WINDOW); j; j >>= 1)
       lqi += ((j & 0x01) << PRR_HALF_WINDOWSIZE);
    lqi = (lqi == 0) ? 0xff : ((MAX_LINK_ETX + (lqi >> 1))/lqi + HOP_COUNT);

    neighbors[i].ob_lqi = compute_MAV_LQI(neighbors[i].ob_lqi, lqi); 
    
    /* Update my parent, bestEtx and nextHop */ 
    if(!is_base)
      updateForwardParent();
      
    /* Update reverse parent, bestEtx and nextHop */
    updateAllReverseParents();  
   
#ifdef CTP_PLUS_DEBUGt
    printTable(neighbors, "sent DL/DT to", nexthop);    
#endif    
  }
  mos_mutex_unlock(&neighborTableLock);   

}


void seed_random() {
  seed = mos_node_id_get() + 1;
}

uint8_t rand_mlcg_8() {
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
  return (mlcg % 256 ) & 0xff; 
}

uint8_t inline getOptions() {
  uint8_t curoption;
  mos_mutex_lock(&optionlock);
    curoption = options;
  mos_mutex_unlock(&optionlock);
  return curoption;
}

void inline resetBeaconInterval(destnode_t *dt) {
  if(dt != NULL) {
    dt->beaconTimerCount = BEACON_TIMER_MIN;
    if (dt->beaconTimer > BEACON_TIMER_MIN) {
        dt->beaconTimer = BEACON_TIMER_MIN;
    }
  } else {
    beaconTimerCount = BEACON_TIMER_MIN;
    if (beaconTimer > BEACON_TIMER_MIN) {
        beaconTimer = BEACON_TIMER_MIN;
    }
  } 
}

void inline increaseBeaconInterval(destnode_t *dt) {
  if(dt != NULL) {
    dt->beaconTimerCount *= 2;
    /* beaconTimerCount exceeds the maximum or overflows */
    if ((dt->beaconTimerCount == 0) || (dt->beaconTimerCount > BEACON_DL_TIMER_MAX)) {
        dt->beaconTimerCount = BEACON_DL_TIMER_MAX;
    }
    dt->beaconTimer = dt->beaconTimerCount;
  } else {    
    beaconTimerCount *= 2;
    if ((beaconTimerCount > BEACON_TIMER_MAX) || (beaconTimerCount == 0)) {
      beaconTimerCount = BEACON_TIMER_MAX;
    }
    beaconTimer = beaconTimerCount;
  }  
}  

void inline beaconFast(destnode_t *dt) {
  if(dt != NULL) {
    if( dt->beaconTimer > BEACON_TIMER_MIN) 
      dt->beaconTimer = BEACON_TIMER_MIN;
  } else {
    if(beaconTimer > BEACON_TIMER_MIN)
      beaconTimer = BEACON_TIMER_MIN;  
  }
}

CTP_frameCategory inline getFrameCat(uint8_t frametype)
{
  switch(frametype) {
    case AM_CTP_ROUTING:
    case AM_CTP_DL_ROUTING:
         return ROUTING_FRAME;   
    case AM_CTP_DATA:    
    case AM_CTP_DL_DATA:
         return DATA_FRAME;         
  }
  return UNKNOWN_FRAME; 
}

boolean inline isDestination(uint8_t frametype, uint16_t dest) 
{
  switch(frametype) {
    case AM_CTP_DATA:    return (is_base);
    case AM_CTP_DL_DATA: return ((dest == myid) || (dest == BROADCAST_ADDR));
    default:             return false;
  }     
}

uint16_t inline getNextHop(uint8_t frametype, uint16_t dest){
  uint8_t i;
  switch(frametype) {
    case AM_CTP_ROUTING : 
    case AM_CTP_DATA:
      return nextHop;  
    case AM_CTP_DL_ROUTING:
    case AM_CTP_DL_DATA:              
      if((i = getDestIndex(dest)) < MAX_NUM_DEST)  return dest_nodes[i].r_nextHop;
  }
  return BROADCAST_ADDR;         
}

uint16_t inline getETX(uint8_t frametype, uint16_t dest) {
  uint8_t i;
  switch(frametype) {
    case AM_CTP_ROUTING : 
    case AM_CTP_DATA:    
      return bestETX;
    case AM_CTP_DL_ROUTING:
    case AM_CTP_DL_DATA:
      if((i = getDestIndex(dest)) < MAX_NUM_DEST)  return dest_nodes[i].r_bestETX;
  }
  return 0xffff;
}

uint8_t inline getHeaderSize(uint8_t frametype) {
  switch(frametype) {
    case AM_CTP_ROUTING :     return CTP_ROUTING_HEADER_SIZE;
    case AM_CTP_DATA:         return CTP_DATA_HEADER_SIZE;
    case AM_CTP_DL_ROUTING:   return CTP_DL_ROUTING_HEADER_SIZE;
    case AM_CTP_DL_DATA:      return CTP_DL_DATA_HEADER_SIZE;
  }
  return 0;
} 

boolean isCached(uint16_t origin, uint8_t seqno, uint8_t thl)
{
   uint8_t i;
   mos_mutex_lock(&cacheMutex);
   for(i = 0; i < CACHE_SIZE; i++) {
      if((cache[i].thl == thl) && (cache[i].origin == origin) && (cache[i].seqno == seqno)) {
        mos_mutex_unlock(&cacheMutex);
        return true;
      }
   }
   cache[cache_tail].origin = origin;
   cache[cache_tail].seqno = seqno;
   cache[cache_tail].thl   = thl;
   cache_tail = (cache_tail + 1) % CACHE_SIZE;  
   mos_mutex_unlock(&cacheMutex); 
   return false;
}

boolean ctp_handle_data_packet(comBuf* buf, uint8_t port)
{
   uint8_t  i, ret;
   uint16_t myetx, dest;
   ctp_data_header_t* dataHeader;
 
  /* Extract the frame header */
   dataHeader = (ctp_data_header_t*)buf->data;  
 
  /* Extract the destination address of the frame */
   dest = (dataHeader->type == AM_CTP_DL_DATA)? 
        b2w(((ctp_dl_data_header_t*)dataHeader)->destHigh, ((ctp_dl_data_header_t*)dataHeader)->destLow) : BASE_ALIAS;

   
#ifdef CTP_PLUS_DEBUGt   
     if(dataHeader->type == AM_CTP_DATA) printDTheader(dataHeader, RECEIVED, buf->source);
     else  printDTheader_dl((ctp_dl_data_header_t*)dataHeader, RECEIVED, buf->source);   
#endif

  /* update my routing table */
   updateRoutingTable_sup(buf->source, dataHeader);
   
  /* check duplicate */   
   if(isCached(b2w(dataHeader->originHigh, dataHeader->originLow), dataHeader->originSeqNo, dataHeader->thl))
   {
#ifdef ENABLE_CTP_PRINTFS
      printf("\nCTP STATS MSG: discard the duplicate data in cache.\n");
#endif
      return false;
   }

  /* if I am the destinaton, pass the packet to the application port */
   if( isDestination(dataHeader->type,  dest) ) {
     if(ctp_append_hops) 
     {
#if defined(CTP_PLUS_DEBUGt) || defined(CTP_PLUS_PRINT_FOOTER)    	      
       printRouteFooter(buf);
#endif 

       /* strip off the ctp footer */
       ret = buf->data[buf->size-1];
       if(ret + getHeaderSize(dataHeader->type) < buf->size)
          buf->size -= ret;     
     }
   	
#ifndef KEEP_CTP_HEADER
    /* strip off the net layer header */      
     ret = getHeaderSize(dataHeader->type); 
     for(i = ret; i < buf->size; ++i)
        buf->data[i - ret] = buf->data[i];
     buf->size -= ret;
#endif
    /* else pass to the upper layer a complete the data frame including CTP header */
     return is_app_waiting_on(port);
   }
  
  /* otherwise consider forwarding to my next hop */  
   mos_mutex_lock(&neighborTableLock);
   myetx = getETX( dataHeader->type,  dest);
   mos_mutex_unlock(&neighborTableLock); 

   ret = 1;
   /* got PULL request? */
   if ((dataHeader->options & CTP_OPT_PULL) && (myetx <= MAX_ETX)) 
   {
     ret = 0;
     mos_mutex_lock(&beaconMutex);     
     beaconFast(NULL);
     mos_mutex_unlock(&beaconMutex);
   }

  /* looping detection */
   else if ( (myetx > b2w(dataHeader->etxHigh, dataHeader->etxLow) ) ||
        (myid == b2w(dataHeader->originHigh, dataHeader->originLow)) ) 
   {
     /* inconsistent path metrics or loopback data packet */
      if(ret && (dataHeader->type == AM_CTP_DATA)) {
        mos_mutex_lock(&beaconMutex);
        beaconFast(NULL);
        mos_mutex_unlock(&beaconMutex);
      }
      else if (dataHeader->type == AM_CTP_DL_DATA) {
        mos_mutex_lock(&neighborTableLock);
        if ((i = getDestIndex(dest)) < MAX_NUM_DEST)     
          beaconFast(&dest_nodes[i]);
        mos_mutex_unlock(&neighborTableLock);
      }        
   }  

  /* if the packet is not too old, forward it */
   if(dataHeader->thl < (CTP_NETWORK_DIAMETER*2)) 
   {
     dataHeader->thl++;  
     
     mos_mutex_lock(&dataQueueMutex); 
     ret = enqueueData(buf, port, 0);
     mos_mutex_unlock(&dataQueueMutex);
         
     /* If the dataQueue Buffer is overflowed */     
     if(!ret) {     	 
       handle_dataQ_overflow(buf, DATA_OVERFLOW);
     }
   }  
       
  /* won't hand the packet to any application port */
   return false;
}


boolean ctp_handle_beacon_packet(const comBuf* buf)
{
  uint8_t   i, j, k, ulrequest;
  uint16_t  myEtx;
  ctp_routing_header_t* routingHeader;
  ctp_dl_routing_header_t * rrh;
  
 /* Extract the frame header */
  routingHeader = (ctp_routing_header_t*)buf->data;

// #ifdef CTP_PLUS_DEBUGt
#ifdef CTP_PLUS_ROUTING_DEBUG
  	if (is_base == true) {
  		if(routingHeader->type == AM_CTP_ROUTING) printRTheader(routingHeader, RECEIVED, buf->source);
  		else printRTheader_dl((ctp_dl_routing_header_t*)routingHeader, RECEIVED, buf->source);
  	}
#endif  

  if(validateSource(buf->source, routingHeader))
    return false;
    
 /* update my routing table */
  updateRoutingTable(buf->source, routingHeader);  
  
 // downstream
  ulrequest = 0;
  mos_mutex_lock(&neighborTableLock);
  if(routingHeader->type == AM_CTP_DL_ROUTING) 
  {    
    rrh = (ctp_dl_routing_header_t *) routingHeader;
    dest_entry_t * dt;        
   
   /* If rrh->leepEntries is not a valid number, don't bother then. */
   /* Or if there's no LEEP Entries, indicating an emergent DL routing 
      message for dest removal, don't bother */ 
    if((rrh->leepEntries>>4) <= MAX_LIENTRIES ) 
    { 
      k = (rrh->leepEntries >> 4) * 3; 
      for(i = 0; i < min( rrh->destEntries, MAX_DESTENTRIES ); i++) 
      {
        dt = (dest_entry_t *)&(rrh->data[k + i * 6]);
        /* if I happen to be the destination for which my neighbor is seeking the route. */      
        if(myid == b2w(dt->node_idHigh, dt->node_idLow)) 
        {
          if(b2w(dt->r_etxHigh, dt->r_etxLow) > MAX_ETX ) 
             ulrequest |= 0x01;
        }
        else if((j = getDestIndex(b2w(dt->node_idHigh, dt->node_idLow))) < MAX_NUM_DEST)
        {	
          myEtx = dest_nodes[j].r_bestETX;
             
          if( /* if I have a valid route to this dest while my neighbor does not */
              ((myEtx <= MAX_ETX) && (b2w(dt->r_parentHigh, dt->r_parentLow) == BROADCAST_ADDR)) ||
              /* Looping detection */
              ((myid == b2w(dt->r_parentHigh, dt->r_parentLow)) && (myEtx >= b2w(dt->r_etxHigh, dt->r_etxLow)))
            )
          {        	
            beaconFast(&dest_nodes[j]);
            if(dest_nodes[j].freshness < 3)
               dest_nodes[j].freshness = 3;               
            ulrequest |= 0x02;   
          } 
        }  
      }     
    }
  }
  // upstream ETX to base 
  myEtx = getETX(AM_CTP_ROUTING, BASE_ALIAS); 
  mos_mutex_unlock(&neighborTableLock);      	
    
  // upstream   
  if( /* request for fastening upstream beacon only */
  	  ulrequest == 1 ||
      /* PULL request */ 
      ((routingHeader->options & CTP_OPT_PULL) && (myEtx <= MAX_ETX)) ||  
      /* Looping detection (inconsistent path metrics) */  
      ((myid == b2w(routingHeader->parentHigh, routingHeader->parentLow)) && 
       (myEtx >= b2w(routingHeader->etxHigh, routingHeader->etxLow)))
    )        	  
  {
      mos_mutex_lock(&beaconMutex);
      beaconFast(NULL);
      mos_mutex_unlock(&beaconMutex);    
  }
  
 /* won't hand the BEACON frame to any application */
  return false;
}

void handle_dataQ_overflow(const comBuf * buf, CTP_DropCondition status)
{
  static char * dropstatus[] = {"Overflow", "Link-fail", "Route-timeout", "Unknown-frame", "Radio-error"};
  
  /* TBD: may add congestion control later */
	
#ifdef CTP_PLUS_DEBUGt
  uint8_t lh = getHeaderSize(buf->data[0]);
  if(lh == CTP_DATA_HEADER_SIZE ) 
     printf("\nStatus: %s. Dropping DATA # %C (%d --> BASE)\n", dropstatus[status], 
              buf->data[lh+1],     // app data seq no.
              b2w(buf->data[lh-3],buf->data[lh-2])); 
  else
     printf("\nStatus: %s. Dropping ACK # %C (BASE %d --> %d)\n", dropstatus[status], 
              buf->data[lh+1],     // app ack no.
              b2w(buf->data[lh-5], buf->data[lh-4]), b2w(buf->data[lh-3], buf->data[lh-2])); 
#else
  /* This node may be congested */
#ifdef ENABLE_CTP_PRINTFS
  printf("\nCTP STATS MSG: %s. Dropping data packet.\n", dropstatus[status]);
#endif
#endif
    
}


uint8_t enqueueData(const comBuf * sentData, uint8_t port, uint8_t attempts)
{
  uint8_t dataQueueTail;
  uint8_t ret = 0;
  ctp_data_header_t * dh1, * dh2;

  if(dataQueueSize < CTP_DATA_QUEUE_SIZE)
  {
     ret = 1;
     dh1 = (ctp_data_header_t *) (sentData->data);     
     if(attempts == 0) {
       /* check duplicate */          
       for(ret = 0; ret < CTP_DATA_QUEUE_SIZE; ret++) {
         dh2 = (ctp_data_header_t *) (dataQueue[ret].data.data);
         if((dh1->thl == dh2->thl) && (dh1->originSeqNo == dh2->originSeqNo) && 
              (b2w(dh1->originHigh, dh1->originLow) == b2w(dh2->originHigh, dh2->originLow))) {
#ifdef ENABLE_CTP_PRINTFS
           printf("\nCTP STATS MSG: discard the duplicate data in the forwarding queue.\n");
#endif
           break;
         }
       }
       ret = (ret >= CTP_DATA_QUEUE_SIZE);
     }

     if(ret) 
     {
       dataQueueTail = (dataQueueHead + dataQueueSize) % CTP_DATA_QUEUE_SIZE;
       memcpy(&dataQueue[dataQueueTail].data, sentData, sizeof(comBuf));
       dataQueue[dataQueueTail].port = port;
       dataQueue[dataQueueTail].timeout = attempts;
       ++dataQueueSize;

       if(dh1->type == AM_CTP_DL_DATA) {     
         connectTo(b2w(((ctp_dl_data_header_t *)dh1)->destHigh, ((ctp_dl_data_header_t *)dh1)->destLow));
       }
       
#ifdef CTP_PLUS_DEBUGt
   qdestbuf[dataQueueTail] = (dh1->type == AM_CTP_DATA) ? 0xffff :
                           b2w(((ctp_dl_data_header_t *)dh1)->destHigh, ((ctp_dl_data_header_t *)dh1)->destLow);
   printDataQueueStatus();
#endif  
 
     }
  }  
  mos_cond_signal(&dataQueueCond); 

  if((dataQueueSize >= CTP_DATA_QUEUE_SIZE) && !(options & CTP_OPT_ECN)) {
    mos_mutex_lock(&optionlock);
    // set congestion bit if data queue is full   
    options |= CTP_OPT_ECN ;  
    mos_mutex_unlock(&optionlock);
  } 

  return ret;
}

uint8_t dequeueData()
{
   uint8_t prev_Qhead;
   prev_Qhead = dataQueueHead;
   dataQueueHead = (dataQueueHead + 1) % CTP_DATA_QUEUE_SIZE;
   dataQueueSize--;
   
   if((dataQueueSize < CTP_DATA_QUEUE_SIZE) && (options & CTP_OPT_ECN)) {
     mos_mutex_lock(&optionlock);
     // unset congestion bit if data queue is not full
     options &= (~CTP_OPT_ECN) ;  
     mos_mutex_unlock(&optionlock); 
   }
   return prev_Qhead;  
}

void send_thread()
{   
   static uint8_t rVal;
   static comBuf packet;
   static uint8_t dataQ_index, port, timeout;
   //static ctp_dl_data_header_t * dlhd;

   while(1) {
      mos_mutex_lock(&dataQueueMutex);
      while (dataQueueSize == 0 ) {
        mos_cond_wait(&dataQueueCond, &dataQueueMutex);
      }
      dataQ_index = dequeueData();
      memcpy(&packet, &dataQueue[dataQ_index].data, sizeof(comBuf));
      port    = dataQueue[dataQ_index].port;
      timeout = dataQueue[dataQ_index].timeout;

      mos_mutex_unlock(&dataQueueMutex);          

      mos_mutex_lock(&sendMutex);
      rVal = net_send(&packet, CTP_PROTO_ID, port, CTP_EXISTING_PACKET);

      /* introduce link transmission interval to avoid 'in-path self-collision' */
      if(packet.source != BROADCAST_ADDR) {
        mos_thread_sleep((rand_mlcg_8() & 0x1f) + 8);
      }
      mos_mutex_unlock(&sendMutex);

      /* address of nextHop is stored in ComBuf source */
      /* If route is available and transmission is preformed */
      if(packet.source != BROADCAST_ADDR) {
      	/* if link layer transmission fails */
        if(rVal > max_retransmit_attempts) 
          handle_dataQ_overflow(&packet, rVal == 0xff ? PHY_ERROR : DATA_TXFAIL);
        if(rVal < 0xff)          
          update_DLQI(packet.source, rVal);
      }          
      /* there's no route found */
      else 
      {
        /* if CTP transmission does not time out yet */
        if(timeout < CTP_TRANSMISSION_TIMEOUT) 
        { // try to store the packet back into the send Q
          mos_mutex_lock(&dataQueueMutex); 
          rVal = enqueueData(&packet, port, timeout+dataQueueSize+1);
          mos_mutex_unlock(&dataQueueMutex);
         
          /* If the dataQueue Buffer is overflowed */
          if(!rVal)
             handle_dataQ_overflow(&packet, DATA_OVERFLOW);   
        }
        else
           handle_dataQ_overflow(&packet, DATA_TIMEOUT);
      }      
      mos_thread_sleep(CTP_SEND_INTERVAL);    
   }
}

void inline appendHops(comBuf * buf, uint16_t mydata) 
{
   if(buf->size <= COM_DATA_SIZE -2) 
   { 
      uint8_t i = buf->data[buf->size-1];
      buf->data[buf->size-1] = (mydata >> 8);
      buf->data[buf->size++] = (mydata & 0xff);
      buf->data[buf->size++] = i + 2;
   }   
}


int8_t net_ctp_proto_send(comBuf *buf, va_list args) 
{  
    uint8_t frame_type, add_ctp_header, port;
    uint16_t dest_in_dl;
       
    ctp_data_header_t    *    header;
    ctp_dl_data_header_t *    dlheader;    
    ctp_routing_header_t *    rh;
    ctp_dl_routing_header_t * dlrh;
    
    uint16_t dest, myEtx, nextHopETX;
    uint8_t i, j, entries, index, dest_entries ;

    if(buf==NULL) {
        /* a NULL packet will not actually sent out on link layer */
        return 0;
    }
    
    /* grab function parameters */
    add_ctp_header = (uint8_t)va_arg(args, int);
    frame_type = (add_ctp_header != CTP_EXISTING_PACKET)? (uint8_t)va_arg(args, int): buf->data[0];
            
    if((add_ctp_header == CTP_NEW_PACKET) && (getFrameCat(frame_type) == DATA_FRAME)) 
    {
        index = (frame_type == AM_CTP_DL_DATA) ? CTP_DL_DATA_HEADER_SIZE : CTP_DATA_HEADER_SIZE;
        /* bail if packet is too big */
        if((buf->size + index) > COM_DATA_SIZE){
#ifdef ENABLE_CTP_PRINTFS
            printf("CTP ERROR MSG: payload is too big.");
#endif
            return 1;
        }        
        /* Shift the payload to insert the CTP DATA header*/
        for(i = 1; i <= buf->size; i++)
            buf->data[buf->size + index -i] = buf->data[buf->size -i];
        buf->size = index + buf->size; 
        
        /* if (route debug) append my id in the payload footer */
        if(ctp_append_hops && buf->size < COM_DATA_SIZE) 
           buf->data[buf->size++] = 1;
    }
    
    if(frame_type == AM_CTP_DATA) 
    {        
        /* Extract the CTP Data frame header */
        header = (ctp_data_header_t*)buf->data;
        
        if(add_ctp_header == CTP_NEW_PACKET) 
        {
            port = (uint8_t)va_arg(args, int);
        	  
            header->type = AM_CTP_DATA;
            header->originSeqNo = (originSeqNo++);
            header->originHigh = myid >> 8;
            header->originLow= myid & 0xff;
            header->thl = 0;
            header->collectId = AM_OSCILLOSCOPE;            

            mos_mutex_lock(&dataQueueMutex); 
            i = enqueueData(buf, port, 0);
            mos_mutex_unlock(&dataQueueMutex);
         
            /* If the dataQueue Buffer is overflowed */
            if(!i)
               handle_dataQ_overflow(buf, DATA_OVERFLOW);  
            return i? 1 : 2;                                   // 1: delayed, 2: dropped         
        }
        mos_led_toggle(2);        
        header->options = getOptions();
        
        /* get my bestEtx and nextHop, as well as the Option for the current data packet */
        mos_mutex_lock(&neighborTableLock);
        dest  = getNextHop(AM_CTP_DATA, BASE_ALIAS); 
        myEtx = getETX(AM_CTP_DATA, BASE_ALIAS);
        mos_mutex_unlock(&neighborTableLock);
        
#ifdef CROSS_LAYER
if (dest != BROADCAST_ADDR) {
        nextHopETX = myEtx - neighbors[getNeighborIndex(dest)].etx;
	meanETX[getNeighborIndex(dest)] = ALPHA(meanETX[getNeighborIndex(dest)]) + ALPHACOMP(nextHopETX);		

#ifdef CTP_PLUS_DATA_DEBUG
        printf("next hop = %d \n", dest);
        nextHopETX = myEtx - neighbors[getNeighborIndex(dest)].etx;
        printf("meanETX = %d, nextHopEtx = %d \n ", meanETX[getNeighborIndex(dest)], nextHopETX);
        printTable(neighbors, "Sending a AM_CTP_DATA frame", dest);
#endif 
	
	if(meanETX[getNeighborIndex(dest)]> OPT_ETX_HIGH && ulPower < MAX_POWER) {
		 printf("meanETX = %d, nextHopEtx = %d, increasing power \n ", meanETX[getNeighborIndex(dest)], nextHopETX);
		//printf("(%d) 1.1 - ulPower = %d",mos_node_id_get(),ulPower);
        	ulPower += ((MAX_POWER - ulPower + POWER_NUMERATOR) / POWER_DIVISOR);
		//printf("(%d) 1.2 - ulPower = %d",mos_node_id_get(),ulPower);
        	net_ctp_proto_ioctl(CTP_SET_POWER, &ulPower);
        }
        else if(meanETX[getNeighborIndex(dest)]< OPT_ETX_LOW && nextHopETX > 0 &&  ulPower > MIN_POWER) {
		 printf("meanETX = %d, nextHopEtx = %d, decreasing power \n ", meanETX[getNeighborIndex(dest)], nextHopETX);
		//printf("(%d) 2.1 - ulPower = %d",mos_node_id_get(),ulPower);
        	// ulPower -= ((ulPower - MIN_POWER + POWER_NUMERATOR) / POWER_DIVISOR);
		 	ulPower -= 1;
		//printf("(%d) 2.2 - ulPower = %d",mos_node_id_get(),ulPower);
        	net_ctp_proto_ioctl(CTP_SET_POWER, &ulPower);
	 }
}
else {
	// Increase transmit power if broadcasting
#ifdef CTP_PLUS_DATA_DEBUG
	printf("next hop = %d (broadcast) \n", dest);
	// printf("increasing power \n");
#endif
	/*
	ulPower += ((MAX_POWER - ulPower + POWER_NUMERATOR) / POWER_DIVISOR);
	net_ctp_proto_ioctl(CTP_SET_POWER, &ulPower);
	*/
}
#endif
		
        header->etxHigh = myEtx >> 8;
        header->etxLow = myEtx & 0xff;
        
        /* store nexthop addr in this unused comBuf field */
        buf->source = dest;
        
#ifdef CTP_PLUS_DEBUGt           
        printDTheader(header, (dest != BROADCAST_ADDR) ? SENT : DELAYED, dest);
#endif

       /* if (route debug) append my id in the payload footer */
       if(ctp_append_hops && (dest != BROADCAST_ADDR) && (buf->size <= COM_DATA_SIZE -2)) 
       {
         if((b2w(header->originHigh, header->originLow) == myid) )       
            appendHops(buf, myEtx);
         else   
            appendHops(buf, myid);
       }     
       
        /* if routing is not found yet, data packet cannot be sent out. */
        if(dest == BROADCAST_ADDR)
            return 1;                                          // 1: delayed, 2: dropped
    }    
    
    else if (frame_type == AM_CTP_DL_DATA){               
        /* Extract the CTP DL Data frame header */
        dlheader = (ctp_dl_data_header_t*)buf->data;
        
        if(add_ctp_header == CTP_NEW_PACKET) 
        {            
            dest_in_dl = (uint16_t)va_arg(args, int);
            port = (uint8_t)va_arg(args, int);
            buf->source = myid;
               
            dlheader->type = AM_CTP_DL_DATA;
            dlheader->originSeqNo = (originSeqNo++);
            dlheader->originHigh = myid >> 8;
            dlheader->originLow= myid & 0xff;
            dlheader->thl = 0;
            dlheader->collectId = AM_OSCILLOSCOPE;
            dlheader->destHigh = dest_in_dl >> 8;
            dlheader->destLow = dest_in_dl & 0xff;

            mos_mutex_lock(&dataQueueMutex); 
            i = enqueueData(buf, port, 0);
            mos_mutex_unlock(&dataQueueMutex);
         
            /* If the dataQueue Buffer is overflowed */
            if(!i)
               handle_dataQ_overflow(buf, DATA_OVERFLOW);

            return i? 1 : 2;                                   // 1: delayed, 2: dropped
        }
        else
          dest_in_dl = b2w(dlheader->destHigh, dlheader->destLow);      
        
        if(dest_in_dl == BROADCAST_ADDR) {
          myEtx = 0;
          dest  = BROADCAST_ADDR;
        } 
        	
        mos_mutex_lock(&neighborTableLock);                         
        /* get my bestEtx and nextHop, as well as the Option for the current data packet */
        /* the nextHop is stored in the source feild of the packet to be sent/forwarded */
        if((i = getDestIndex(dest_in_dl)) < MAX_NUM_DEST) {
           dest  = dest_nodes[i].r_nextHop;
           myEtx = dest_nodes[i].r_bestETX;      
        }
        else {
        	 dest = BROADCAST_ADDR;
        	 myEtx = 0xffff;
        }
        
        /* if this is the first attempt of routing the packet or no route available */ 
        if((dest != BROADCAST_ADDR) && (buf->source == BROADCAST_ADDR)) {
           /* wait for a while to allow the new route to be stablized */
           mos_mutex_unlock(&neighborTableLock);
           mos_thread_sleep(BEACON_INTERVAL); 
           mos_mutex_lock(&neighborTableLock);
             
           if((i = getDestIndex(dest_in_dl)) < MAX_NUM_DEST) {
              dest  = dest_nodes[i].r_nextHop;
              myEtx = dest_nodes[i].r_bestETX;      
           }
           else {
        	    dest = BROADCAST_ADDR;
        	    myEtx = 0xffff;
           }        
        }
        if((i < MAX_NUM_DEST) && (dest_nodes[i].inqueue > 0)) {
        	 --dest_nodes[i].inqueue;
        }
        mos_mutex_unlock(&neighborTableLock);
        
#ifdef CROSS_LAYER
if (dest!= BROADCAST_ADDR) {
        nextHopETX = myEtx - neighbors[getNeighborIndex(dest)].r_etx[getDestIndex(dest_in_dl)];		
	meanETXdl[getNeighborIndex(dest)][getDestIndex(dest_in_dl)] = ALPHA(meanETXdl[getNeighborIndex(dest)][getDestIndex(dest_in_dl)]) + ALPHACOMP(nextHopETX);

#ifdef CTP_PLUS_DATA_DEBUG
        printf("next hop = %d \n", dest);
        // printf("ETX of next hop = %d \n", getETX(AM_CTP_DL_DATA, dest));
        /*
        nextHopETX = myEtx - 
        	neighbors[getNeighborIndex(dest)].r_etx[getDestIndex(dest_in_dl)];
        printf("neighbor r_etx = %d, destIndex = %d \n", 
        		neighbors[getNeighborIndex(dest)].r_etx[getDestIndex(dest_in_dl)], 
        		getDestIndex(dest_in_dl));
        */ 
        /*
        uint8_t destIndex;
        for (destIndex = 0; destIndex < MAX_NUM_DEST; destIndex++) {
        	printf("neighbor r_etx[%d] = %d \n", destIndex, 
        			neighbors[getNeighborIndex(dest)].r_etx[destIndex]);  
        }
        */
        printf("meanETXdl = %d, nextHopEtx = %d \n ", meanETXdl[getNeighborIndex(dest)][getDestIndex(dest_in_dl)], nextHopETX);
        
        printTable(neighbors, "Sending a AM_CTP_DL_DATA frame", dest);
#endif
	
        if(meanETXdl[getNeighborIndex(dest)][getDestIndex(dest_in_dl)] > OPT_ETX_HIGH && dlPower < MAX_POWER) {
        	printf("meanETXdl = %d, nextHopEtx = %d, increasing power \n ", meanETXdl[getNeighborIndex(dest)][getDestIndex(dest_in_dl)], nextHopETX);
		//printf("(%d) 1.1 - dlPower = %d",mos_node_id_get(),dlPower);
        	dlPower += ((MAX_POWER - dlPower + POWER_NUMERATOR) / POWER_DIVISOR);
		//printf("(%d) 1.2 - dlPower = %d",mos_node_id_get(),dlPower);
        	net_ctp_proto_ioctl(CTP_SET_POWER, &dlPower);

        }
        else if(meanETXdl[getNeighborIndex(dest)][getDestIndex(dest_in_dl)] < OPT_ETX_LOW && nextHopETX > 0 &&  dlPower > MIN_POWER) {
        	printf("meanETXdl = %d, nextHopEtx = %d, decreasing power \n ", meanETXdl[getNeighborIndex(dest)][getDestIndex(dest_in_dl)], nextHopETX);
		//printf("(%d) 2.1 - dlPower = %d",mos_node_id_get(),dlPower);
        	// dlPower -= ((dlPower - MIN_POWER + POWER_NUMERATOR) / POWER_DIVISOR);
        	dlPower -= 1;
		//printf("(%d) 2.2 - dlPower = %d",mos_node_id_get(),dlPower);
        	net_ctp_proto_ioctl(CTP_SET_POWER, &dlPower);
	}
}
else {
	// Increase transmit power if broadcasting
#ifdef CTP_PLUS_DATA_DEBUG
	printf("next hop = %d (broadcast) \n", dest);
	// printf("increasing power \n");
#endif
	/*
	dlPower += ((MAX_POWER - dlPower + POWER_NUMERATOR) / POWER_DIVISOR);
	net_ctp_proto_ioctl(CTP_SET_POWER, &dlPower);
	*/
}
#endif        
        
        mos_led_toggle(4);
        dlheader->options = getOptions();
                   
        dlheader->etxHigh = myEtx >> 8;
        dlheader->etxLow = myEtx & 0xff;
        
        /* store nexthop addr in this unused comBuf field */
        buf->source = dest;
        
#ifdef CTP_PLUS_DEBUGt           
     printDTheader_dl(dlheader, ((dest != BROADCAST_ADDR) || (dest_in_dl == BROADCAST_ADDR)) ? SENT : DELAYED, dest);
#endif        
        
        /* if (route debug) append my id in the payload footer */
        if(ctp_append_hops && (dest != BROADCAST_ADDR) && (buf->size <= COM_DATA_SIZE -2)) 
        {
          if((b2w(dlheader->originHigh, dlheader->originLow) == myid) )       
            appendHops(buf, myEtx);
          else   
            appendHops(buf, myid);
        }       
        
        /* if routing is not found yet, data packet cannot be sent out. */
        if((dest == BROADCAST_ADDR) && (dest_in_dl != BROADCAST_ADDR))
          return 1;                                            // 1: delayed, 2: dropped
    }
    else if (frame_type == AM_CTP_DL_ROUTING)
    {
        mos_led_toggle(5);
        /* Beacon frame always has no payload*/        
        /* Extract the CTP DL BEACON frame header */
        dlrh = (ctp_dl_routing_header_t*)buf->data;        
        
        if(add_ctp_header == CTP_NEW_PACKET) {
            dlrh->type = AM_CTP_DL_ROUTING;
            dlrh->seqno = (beaconSeqNo++);            
        }            
        dlrh->options = getOptions();    
        
        mos_mutex_lock(&neighborTableLock);        
        /* Populate Leep Entries */
        for (i = 0, entries = 0; i < MAX_NEIGHBORS && entries < MAX_LIENTRIES; i++) {
           index = (lastParent + i) % MAX_NEIGHBORS;            
           if (neighbors[index].addr != BROADCAST_ADDR) {
               /* copy node id of the neighbor */
               dlrh->data[3 * entries] = (neighbors[index].addr >> 8);       
               dlrh->data[3 * entries + 1] = (neighbors[index].addr & 0xff);                 
               /* copy neighbor's in-bound lqi (lqi from the neighbor to me) */
               dlrh->data[3 * entries + 2] = (uint8_t)neighbors[index].ib_lqi;                
               entries++;
           }
        }
        lastParent = (lastParent + i) % MAX_NEIGHBORS;
        dlrh->leepEntries = entries << 4;
        
        /* Populate Dest Entries */
        j = LEEP_ENTRY_SIZE * entries;
        dest_entries = 0;                      
          
        for(i = 0; i < MAX_NUM_DEST && dest_entries < MAX_DESTENTRIES; ++i){ 
           if(destforbeacon[i]){              
              dlrh->data[j + 0] = (dest_nodes[i].id >> 8);
              dlrh->data[j + 1] = (dest_nodes[i].id & 0xff);
              dlrh->data[j + 2] =  dest_nodes[i].r_nextHop >> 8;
              dlrh->data[j + 3] =  dest_nodes[i].r_nextHop & 0xff;
              dlrh->data[j + 4] = (dlrh->options & CTP_OPT_ECN) ? 0xff:dest_nodes[i].r_bestETX >> 8;   
              dlrh->data[j + 5] = (dlrh->options & CTP_OPT_ECN) ? 0xff:dest_nodes[i].r_bestETX & 0xff; 
              ++dest_entries;
              j += DEST_ENTRY_SIZE;
           }
        }   
        
        /* emergent broadcast to advertise the removed dest entries */ 
        for(index = 0; (index < rm_list_size) && (dest_entries < MAX_DESTENTRIES); ++index) {              
           if(getDestIndex(rm_list[rm_list_head]) >= MAX_NUM_DEST) {       
              dlrh->data[j + 0] = dlrh->data[j + 2] = (rm_list[rm_list_head] >> 8);
              dlrh->data[j + 1] = dlrh->data[j + 3] = (rm_list[rm_list_head] & 0xff);
              dlrh->data[j + 4] = 0xff;
              dlrh->data[j + 5] = 0xff;
              ++dest_entries;
              j += DEST_ENTRY_SIZE;              
           }
           rm_list_head = (rm_list_head + 1) % RM_LIST_SIZE;  
        }
        rm_list_size -= index;
              
        dlrh->destEntries = dest_entries;        
        buf->size = CTP_DL_ROUTING_HEADER_SIZE + j;              
          
        /* get my bestEtx and nextHop for upstream cast */
        dest  = getNextHop(AM_CTP_ROUTING, BASE_ALIAS);  
        myEtx = (dlrh->options & CTP_OPT_ECN) ? 0xffff : getETX(AM_CTP_ROUTING, BASE_ALIAS);  

        mos_mutex_unlock(&neighborTableLock);    

#ifdef CROSS_LAYER
        if (false) {
        nextHopETX = myEtx - neighbors[getNeighborIndex(dest)].r_etx[getDestIndex(dest_in_dl)];	
	meanETXdl[getNeighborIndex(dest)][getDestIndex(dest_in_dl)] = ALPHA(meanETXdl[getNeighborIndex(dest)][getDestIndex(dest_in_dl)]) + ALPHACOMP(nextHopETX);	

        if(meanETXdl[getNeighborIndex(dest)][getDestIndex(dest_in_dl)] > OPT_ETX_HIGH && dlPower < MAX_POWER) {
		 printf("meanETXdl = %d, nextHopEtx = %d, increasing power \n ", meanETXdl[getNeighborIndex(dest)][getDestIndex(dest_in_dl)], nextHopETX);
		//printf("(%d) 1.1 - dlPower = %d",mos_node_id_get(),dlPower);
        	dlPower += ((MAX_POWER - dlPower + POWER_NUMERATOR) / POWER_DIVISOR);
		//printf("(%d) 1.2 - dlPower = %d",mos_node_id_get(),dlPower);
        	net_ctp_proto_ioctl(CTP_SET_POWER, &dlPower);
        }
        else if(meanETXdl[getNeighborIndex(dest)][getDestIndex(dest_in_dl)] < OPT_ETX_LOW && nextHopETX > 0 &&  dlPower > MIN_POWER) {
		 printf("meanETXdl = %d, nextHopEtx = %d, decreasing power \n ", meanETXdl[getNeighborIndex(dest)][getDestIndex(dest_in_dl)], nextHopETX);
		//printf("(%d) 2.1 - dlPower = %d",mos_node_id_get(),dlPower);
		 	// dlPower -= ((dlPower - MIN_POWER + POWER_NUMERATOR) / POWER_DIVISOR);
		 	dlPower -= 1;
		//printf("(%d) 2.2 - dlPower = %d",mos_node_id_get(),dlPower);
        	net_ctp_proto_ioctl(CTP_SET_POWER, &dlPower);
	 }
        }
#endif
        
        dlrh->etxHigh = myEtx >> 8;
        dlrh->etxLow = myEtx & 0xff;
        dlrh->parentHigh = dest >> 8;
        dlrh->parentLow  = dest & 0xff;
        
        /* store nextHop address in this unused comBuf field */
        buf->source = BROADCAST_ADDR; //always BROADCASTing beacon
        
#ifdef CTP_PLUS_DEBUGt           
        printRTheader_dl(dlrh, SENT, 0);
#endif        
    }
    else if (frame_type == AM_CTP_ROUTING) 
    {
        /* Beacon frame always has no payload*/
        /* Extract the CTP BEACON frame header */
        rh = (ctp_routing_header_t*)buf->data;
        
        if(add_ctp_header == CTP_NEW_PACKET) {
            rh->type = AM_CTP_ROUTING;
            rh->seqno = (beaconSeqNo++);
        }
        
        mos_led_toggle(3);
        rh->options = getOptions();
        
        mos_mutex_lock(&neighborTableLock);
        /* Populate Leep Entries */
        for (i = 0, entries = 0; i < MAX_NEIGHBORS && entries < MAX_LIENTRIES; i++) {
            index = (lastParent + i) % MAX_NEIGHBORS;            
            if (neighbors[index].addr != BROADCAST_ADDR) {
            	
                /* copy node id of the neighbor */
                rh->data[3 * entries] = (neighbors[index].addr >> 8);         //node_idHigh
                rh->data[3 * entries + 1] = (neighbors[index].addr & 0xff);   //node_idLow                
                
                /* copy neighbor's in-bound lqi (lqi from the neighbor to me) */
                rh->data[3 * entries + 2] = (uint8_t)neighbors[index].ib_lqi;                
                entries++;
            }
        }
        lastParent = (lastParent + i) % MAX_NEIGHBORS;
        rh->leepEntries = entries << 4;
        buf->size = CTP_ROUTING_HEADER_SIZE + LEEP_ENTRY_SIZE*entries;
        
        /* get my bestEtx and nextHop */
        dest  = getNextHop(AM_CTP_ROUTING, BASE_ALIAS);  
        myEtx = (rh->options & CTP_OPT_ECN) ? 0xffff : getETX(AM_CTP_ROUTING, BASE_ALIAS); 
#ifdef CROSS_LAYER

        nextHopETX = myEtx - neighbors[getNeighborIndex(dest)].etx;	
	meanETX[getNeighborIndex(dest)] = ALPHA(meanETX[getNeighborIndex(dest)]) + ALPHACOMP(nextHopETX);	

	 if(meanETX[getNeighborIndex(dest)]> OPT_ETX_HIGH && ulPower < MAX_POWER) {
		 printf("meanETX = %d, nextHopEtx = %d, increasing power \n ", meanETX[getNeighborIndex(dest)], nextHopETX);
		//printf("(%d) 1.1 - ulPower = %d",mos_node_id_get(),ulPower);
        	ulPower += ((MAX_POWER - ulPower + POWER_NUMERATOR) / POWER_DIVISOR);
		//printf("(%d) 1.2 - ulPower = %d",mos_node_id_get(),ulPower);
        	net_ctp_proto_ioctl(CTP_SET_POWER, &ulPower);
        }
        else if(meanETX[getNeighborIndex(dest)] < OPT_ETX_LOW && nextHopETX > 0 && ulPower > MIN_POWER) {
		 printf("meanETX = %d, nextHopEtx = %d, decreasing power \n ", meanETX[getNeighborIndex(dest)], nextHopETX);
		//printf("(%d) 2.1 - ulPower = %d",mos_node_id_get(),ulPower);
        	// ulPower -= ((ulPower - MIN_POWER + POWER_NUMERATOR) / POWER_DIVISOR);
		 	ulPower -= 1;
		//printf("(%d) 2.2 - ulPower = %d",mos_node_id_get(),ulPower);
        	net_ctp_proto_ioctl(CTP_SET_POWER, &ulPower);
	 }
#endif
               
        mos_mutex_unlock(&neighborTableLock);   

        rh->etxHigh = myEtx >> 8;
        rh->etxLow = myEtx & 0xff;
        rh->parentHigh = dest >> 8;
        rh->parentLow  = dest & 0xff;
        
        /* store nextHop address in this unused comBuf field */
        buf->source = BROADCAST_ADDR; //always BROADCASTing beacon
        
#ifdef CTP_PLUS_DEBUGt           
        printRTheader(rh, SENT, 0);
#endif        
    }
    else {
#ifdef ENABLE_CTP_PRINTFS
        printf("CTP STATS MSG: unknown-frame type %C. Dropping data packet.\n", frame_type);
#endif
        return 2;
    }   
    
    /* send it */
    return 0;
}


boolean net_ctp_proto_recv(comBuf *buf, uint8_t **footer, uint8_t port) 
{    
     
  if(buf==NULL) {
#ifdef ENABLE_CTP_PRINTFS
    printf("CTP STATS MSG: RADIO RECV Interface busy.\n");
#endif
   /* prevent net_recv_packet() from freeing a NULL packet*/
    return true;
  }
  if(buf->source == myid) {
#ifdef ENABLE_CTP_PRINTFS
    printf("CTP STATS MSG: discard loopbacked packet.\n");
#endif
   /* Loopback packet. free it. */
    return false;
  }
  
#ifdef GET_RSSI
  uint8_t *bytes, realrssi;

 /* Extract the rssi from "link quality indice" reported by link layer */
  bytes  = (uint8_t *) &(buf->signal);      
  realrssi = (int8_t)(bytes[0]) + 128;

  if (realrssi < RSSI_THRESHOLD)
  {
#ifdef ENABLE_CTP_PRINTFS
     printf("CTP STATS MSG: bad rssi %d, discard the %s frame from %d\n", 
                 realrssi, getFrameCat(buf->data[0]) == DATA_FRAME ? "DATA":"BEACON", buf->source);
#endif

   /* free packet memory */
    return false;
  }
#endif  
  
  if(buf->size < getHeaderSize(buf->data[0]) )
  {
#ifdef ENABLE_CTP_PRINTFS
     printf("CTP STATS MSG: invalid frame size %C, discard the %s frame from %d\n", 
                 buf->size, getFrameCat(buf->data[0]) == DATA_FRAME ? "DATA":"BEACON", buf->source);
#endif
     
   /* free packet memory */
    return false;
  }
  
 /* The first byte of the header indicates the frame type*/
  if (getFrameCat(buf->data[0]) == DATA_FRAME) 
  {
    mos_led_toggle(1); 
    return ctp_handle_data_packet(buf, port);
  }
  else if (getFrameCat(buf->data[0]) == ROUTING_FRAME) 
  {
    mos_led_toggle(0);    
    return ctp_handle_beacon_packet(buf);
  }
  else {
#ifdef ENABLE_CTP_PRINTFS
    printf("CTP STATS MSG: unknown-frame type %C from %d. Dropping data packet.\n", buf->source, buf->data[0]);
#endif
  }
   
 /* free packet memory */
  return false;
}

void ctp_proto_init(void) {  
  uint8_t i;

 /* initialize my node_id */
  myid = mos_node_id_get();   

 /* status msg: if I am the base or node */
#ifdef ENABLE_CTP_PRINTFS
  printf("CTP INIT MSG: Initializing %s sensor %d\n", is_base? "BASE":"NODE", myid);
#endif

  mos_mutex_init(&sendMutex);
  mos_mutex_init(&beaconMutex);
  mos_mutex_init(&neighborTableLock);
  mos_mutex_init(&optionlock);
  mos_mutex_init(&cacheMutex);
  mos_mutex_init(&dataQueueMutex);
  mos_cond_init(&dataQueueCond);

  seed_random();
  mos_led_off(0);
  mos_led_off(1);
  mos_led_off(2);
  
 /* initialize the option */
  options = is_base ? 0 : CTP_OPT_PULL;

 /* initialize beacon system */
  beaconTimerCount = BEACON_TIMER_MIN;
  beaconTimer = BEACON_TIMER_MIN;
  beaconSeqNo = 250;
  lastParent = 0;
  
 /* initialize the routing table */ 
  originSeqNo = 0;
  for (i = 0; i < MAX_NEIGHBORS; i++) {
    initNeighborEntry( &neighbors[i] );
    mos_udelay(2);
  }
  parent  = MAX_NEIGHBORS;
  bestETX = is_base? 0 : 0xffff;
  nextHop = is_base? myid : BROADCAST_ADDR;  
  
  num_dest = 0;
  for(i = 0; i < MAX_NUM_DEST; i++) {  
    initDLDestEntry(&dest_nodes[i]);  
  }

  rm_list_head = rm_list_size = 0;
  for(i = 0; i < RM_LIST_SIZE; i++) {
    rm_list[i] = BROADCAST_ADDR;
  }

 /* initialize the dataQueue */
  dataQueueSize = dataQueueHead = 0;
  for(i = 0; i < CTP_DATA_QUEUE_SIZE; i++) {
	 memset(&dataQueue[i], 0, sizeof(data_queue_entry_t));
     // initDataQueueEntry(&dataQueue[i]);
  }

 /* initialize the cache of the received data info */
  cache_tail = 0;
  for(i = 0; i < CACHE_SIZE; i++) {
    //cache[i].seqno = 0xff;
    //cache[i].thl   = 0xff;
    //cache[i].origin = BROADCAST_ADDR;
    memset(&cache[i], 0xff, sizeof(cache_entry_t));
  }

 /* ctp uses backend threads */ 
  mos_thread_new(beacon_thread, 320, PRIORITY_NORMAL);
  mos_thread_new(send_thread, 320, PRIORITY_NORMAL);

 /* register ctp protocal */
  net_proto_register(CTP_PROTO_ID, net_ctp_proto_send, net_ctp_proto_recv, net_ctp_proto_ioctl);

#ifdef CTP_PLUS_DEBUGt
  mos_thread_new_havestack(debug_thread, DEBUG_STACK_SIZE, debug_stack, PRIORITY_NORMAL);
#endif
 
}


int8_t net_ctp_proto_ioctl(uint8_t request, va_list args) {    
  uint16_t power;

 /* do requested action */
  switch(request) {
   /* make this sensor a base station */
    case CTP_SET_IS_BASE:      
      is_base = true;
      parent  = 0;
      bestETX = 0;
      nextHop = myid;
#ifdef ENABLE_CTP_PRINTFS
      printf("CTP IOCTL MSG: Set sensor %d to be the base station.\n", mos_node_id_get());
#endif
      break;
   /* make this sensor a node */    
    case CTP_SET_IS_NODE:
      is_base = false;
      parent  = 0;
      bestETX = 0xffff;
      nextHop = BROADCAST_ADDR;
#ifdef ENABLE_CTP_PRINTFS
      printf("CTP IOCTL MSG: Set sensor %d to be a node.\n", mos_node_id_get());
#endif
      break;
   /* set sensor's RF power level */   
    case CTP_SET_POWER:
      power = (uint16_t)va_arg(args, int);
#ifdef PLATFORM_TELOSB
      com_ioctl_IFACE_RADIO(CC2420_TX_POWER, power);
#ifdef ENABLE_CTP_PRINTFS
      printf("Set sensor %d power level to be %d\n", mos_node_id_get(), power);
#endif
#else
#ifdef ENABLE_CTP_PRINTFS
      printf("CTP IOCTL MSG: SET_POWER option is now supported on only PLATFORM_TELOSB.\n");
#endif
#endif
      break;
   /* reduce the senor's RF power to the lowest */       
    case CTP_SET_TESTBED:
#ifdef PLATFORM_TELOSB
      // com_ioctl_IFACE_RADIO(CC2420_HIGH_POWER_MODE);
      power = 2;
      com_ioctl_IFACE_RADIO(CC2420_TX_POWER, power);
      // printf("Set sensor %d power level to be low (2).\n", mos_node_id_get());
#ifdef ENABLE_CTP_PRINTFS
      printf("Set sensor %d power level to be low (2).\n", mos_node_id_get());
#endif
      // printf("Set sensor %d power level to be highest (31).\n", mos_node_id_get());
#else
#ifdef ENABLE_CTP_PRINTFS
      printf("CTP IOCTL MSG: SET_POWER option is now supported on only PLATFORM_TELOSB.\n");
#endif
#endif
      break;         
    default:
#ifdef ENABLE_CTP_PRINTFS
      printf("CTP ERROR MSG: Invalid CTP IOCTL option.\n");
#endif
      return 1;
  }
  return 0;
}


uint8_t connectTo(uint16_t dest)
{
   uint8_t i = MAX_NUM_DEST;    
   if(dest!= BROADCAST_ADDR) 
   {
      mos_mutex_lock(&neighborTableLock);
      /* If dest is not in the dest_nodes table yet */
      if( (i = getDestIndex(dest)) >= MAX_NUM_DEST ) 
      {
        /* if we are able to insert the new dest into the dest table */      
        if( (i = insertNewDest(dest)) < MAX_NUM_DEST) {              
           updateReverseParent(i); 
           ++dest_nodes[i].inqueue; 
        }    
      }
      /* If dest is already in the dest_nodes table */
      else 
      {
        if(dest_nodes[i].r_nextHop != BROADCAST_ADDR) 
           dest_nodes[i].freshness = NEW_DL_FRESH;
        ++dest_nodes[i].inqueue;
      }   	
      mos_mutex_unlock(&neighborTableLock);
   }
   
#ifdef CTP_PLUS_DEBUGt
   if(i < MAX_NUM_DEST)
     printf("\nconnect%s to dest %d\n", dest_nodes[i].r_nextHop == BROADCAST_ADDR ? "ing":"ed", dest);
   else
     printf("\ncannot insert dest %d\n", dest); 
#endif
     
   return i;
}

void close(uint16_t dest) {
    uint8_t i;
    
    if(dest== BROADCAST_ADDR) 
       return;   

    mos_mutex_lock(&neighborTableLock);
    if( (i = getDestIndex(dest)) < MAX_NUM_DEST ) {
       initDLDestEntry(&dest_nodes[i]);  
       updateReverseParent(i);
       mos_mdelay(5);
    }
    mos_mutex_unlock(&neighborTableLock);
}

