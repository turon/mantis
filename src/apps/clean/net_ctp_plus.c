#include "ctp_plus.h"
#include "net.h"
#include "node_id.h"

// NOTE: Keep consistency of address format.
// 1. com_send, com_recv on link layer won't take care of the address format. 
// 2. apps on network layer need to do htons conversion, if neccesary. 
//    e.g.  in ctp:
//          com_sendto(IFACE_RADIO, &send_buf, htons(dest))
//          sender's node id is: htons(recv_buf->source)
#define htons(v)                (((v) >> 8) | (((v) & 0xff) << 8))
#define b2w(high,low)           (((high)<<8)|((low)&0xff))
#define computeETX(etx, ob_lqi) (((etx) < (0xffff-(ob_lqi)) ) ? ((etx) + (ob_lqi)) : 0xffff);    
#define min(x,y)                ((x)<(y)? (x):(y))
#define max(x,y)                ((x)>(y)? (x):(y))

#define BASE_ALIAS  0xffff

boolean is_base = false;
boolean ctp_append_hops = false;

uint16_t myid; 
uint32_t seed;

/* mutexes used by CTP */
mos_mutex_t beaconMutex;
mos_mutex_t neighborTableLock;
mos_mutex_t optionlock;

/*-- protected by optionLock --*/
uint8_t options = CTP_OPT_PULL;
/*-- protected by optionLock --*/

/*-- protected by beaconMutex --*/
comBuf beaconPacket, beaconPacket2;
int16_t beaconTimer = BEACON_TIMER_MAX;
uint16_t beaconTimerCount = BEACON_TIMER_MAX;
uint8_t beaconSeqNo = 0;
/*-- protected by beaconMutex --*/

/*-- protected by neighborTableLock --*/
routing_entry_t neighbors[MAX_NEIGHBORS];
uint16_t bestETX;
uint16_t nextHop;
uint8_t  parent;
destnode_t dest_nodes[MAX_NUM_DEST]; // destinations of interest, assoicated with r_etx[] in the table.  
uint8_t  num_dest; 

/*-- Xmit power optimization -- */
uint8_t dlPower = 31; // max power 31
uint8_t ulPower = 31; // max power 31

uint8_t originSeqNo = 0;
uint8_t lastParent = 0;
uint8_t lastDest = 0;
/*-- protected by neighborTableLock --*/

#ifdef CTP_PLUS_DEBUG
  #include "ctp_plus_debug.h"
  #define DEBUG_STACK_SIZE     512
  static stackval_t debug_stack[DEBUG_STACK_SIZE];
#endif

void initNeighborEntry(routing_entry_t * ent)
{
  ent->addr = BROADCAST_ADDR;
  ent->etx = 0xffff;
  ent->ib_lqi = 0xff;  
  ent->ob_lqi = 0xff;  
  ent->lastrecvdseqno = 0;
  ent->prr = 0; 
  ent->freshness = 0;  
  ent->flag = 0;
  
  uint8_t i;
  for(i = 0; i < MAX_NUM_DEST; i++)    
    ent->r_etx[i] = 0xffff;
}

void initDLDestEntry(destnode_t * destEnt)
{
  destEnt->id = BROADCAST_ADDR;
  destEnt->freshness = 0;
  destEnt->r_parent  = MAX_NEIGHBORS;
  destEnt->r_bestETX = 0xffff;
  destEnt->r_nextHop = BROADCAST_ADDR;
    
  //resetBeaconInterval(destEnt);
  destEnt->beaconTimer = BEACON_TIMER_MIN;
  destEnt->beaconTimerCount = BEACON_TIMER_MIN;  
}

uint8_t chooseParent(uint8_t dest_indx)
{
  uint8_t i, j;
  uint16_t etx, getbestetx;
  
 /* (dest_indx >= MAX_NUM_DEST) implies that we are computing forward ETX to Base */  
  dest_indx = min(dest_indx, MAX_NUM_DEST);
  
  for (i = 0, getbestetx = 0xffff, j= MAX_NEIGHBORS; i < MAX_NEIGHBORS; i++) {  
   /* unset the flag bit associated with this dest */
    neighbors[i].flag &= (~(1 << dest_indx));                
   
   /* ETX to downstream destination or ETX to base */ 
    etx = (dest_indx < MAX_NUM_DEST) ? neighbors[i].r_etx[dest_indx] : neighbors[i].etx;
    
    etx = computeETX(etx, neighbors[i].ob_lqi);    
    if (etx < getbestetx) {
      getbestetx = etx;
      j = i;
    }
  }  
  
 /* set the flag bit associated with this dest for the parent */
  if(j < MAX_NEIGHBORS)   
    neighbors[j].flag |= (1 << dest_indx);        
    
  return j;  
}


/* Update my parent, bestEtx and nextHop for upstream cast */
void updateForwardParent() 
{
  uint8_t j;  
  uint8_t set_power;

 /* update parent for upstream cast */  
  j = chooseParent(MAX_NUM_DEST); 
  if(j < MAX_NEIGHBORS) 
  {
    bestETX = computeETX(neighbors[j].etx, neighbors[j].ob_lqi);        
    nextHop = bestETX > MAX_ETX ? BROADCAST_ADDR : neighbors[j].addr;  
    if(bestETX > MAX_ETX) bestETX = 0xffff;
  }     
  else {
    bestETX = 0xffff;      
    nextHop = BROADCAST_ADDR;
  }    
  if (j != parent) {
    parent = j;   
    
#ifdef CTP_PLUS_DEBUG    
    printf("parent changed. current bestETX %d, current nextHop %d\n", bestETX, nextHop);
#endif    

  }
}


/* Update my parent, bestEtx and nextHop for downstream cast */
void updateReverseParent() 
{
  uint8_t i, j;

 /* update parents for downstream cast to different destination nodes */ 
  for(i = 0, num_dest = 0; i < MAX_NUM_DEST ; ++i) 
  {
    if(dest_nodes[i].id == BROADCAST_ADDR) continue;
   /* number of the destionations we are currently tracking */
    ++num_dest;

    if(dest_nodes[i].id == myid) {
      dest_nodes[i].r_parent =  MAX_NEIGHBORS;  //useless
      dest_nodes[i].r_bestETX = 0;
      dest_nodes[i].r_nextHop = myid;
      continue;
    }    
    j = chooseParent(i);  
    if(j < MAX_NEIGHBORS) 
    {
      dest_nodes[i].r_bestETX = computeETX(neighbors[j].r_etx[i], neighbors[j].ob_lqi);        
      dest_nodes[i].r_nextHop = dest_nodes[i].r_bestETX > MAX_ETX ? BROADCAST_ADDR : neighbors[j].addr;
      if(dest_nodes[i].r_bestETX > MAX_ETX) dest_nodes[i].r_bestETX = 0xffff;
    }     
    else {
      dest_nodes[i].r_bestETX = 0xffff;      
      dest_nodes[i].r_nextHop = BROADCAST_ADDR;
    } 
    if (j != dest_nodes[i].r_parent) {
      dest_nodes[i].r_parent = j;     

#ifdef CTP_PLUS_DEBUG      
      printf("parent for dest %d changed. current bestETX %d, current nextHop %d\n", dest_nodes[i].id, dest_nodes[i].r_bestETX, dest_nodes[i].r_nextHop);
#endif      
    }
  }  
}

#ifdef CTP_PLUS_DEBUG
  #define PRINT_INTERVAL   10
  uint8_t beacon_print_timer = PRINT_INTERVAL;
#endif

void beacon_thread(void) {
  static uint8_t i, j, parentChanged, hasNoLIEntry;
  static uint16_t dest;
  static uint8_t destforbeacon[MAX_NUM_DEST], dlBeaconRequest;
  
  while(1) {
  	
#ifdef CTP_PLUS_DEBUG    	
    printf("\nbeacon thread: timer %d. dest0 (%d, %d), dest1 (%d, %d)%C", 
                      dest_nodes[0].id, dest_nodes[0].beaconTimer, dest_nodes[1].id, dest_nodes[1].beaconTimer, MAX_NUM_DEST==2?'\n':' ');
    for(j=2;j < MAX_NUM_DEST; j++) printf(", dest%C (%d, %d)%C", j, dest_nodes[j].id, dest_nodes[j].beaconTimer, j==MAX_NUM_DEST-1?'\n':' ');    
#endif
        
   /* minimal beacon interval (1000 ms) */
    mos_thread_sleep(BEACON_INTERVAL);

   /* update routingTable with freshness and parent */
    mos_mutex_lock(&neighborTableLock);
    
   /* fresh the forward routing table */
    for (i = 0, hasNoLIEntry = 1, parentChanged = 0; i < MAX_NEIGHBORS; i++) {
      if (neighbors[i].addr == BROADCAST_ADDR)
        continue;
        
      if (neighbors[i].freshness == 0) {        
        if (neighbors[i].flag) //if (i == parent)
          parentChanged = 1;
        initNeighborEntry( &neighbors[i] );  
      }
      else {
        neighbors[i].freshness--;
        
        //if(is_base) hasNoLIEntry = 0; else 
        if(neighbors[i].ob_lqi <= MAX_LQI) 
          hasNoLIEntry = 0;           
      }
    }    
   /* choose New Forward Parent, only applied to node */
    if (!is_base && parentChanged) {
      updateForwardParent();      
    } 
    dest = nextHop;
    
   /* fresh the reverse routing table */
    if(num_dest > 0) {
      for(i = 0; i < MAX_NUM_DEST; i++) {
        if(dest_nodes[i].id == BROADCAST_ADDR)
          continue;
        if (dest_nodes[i].freshness == 0) {
        	
          initDLDestEntry(&dest_nodes[i]);          
          for(j = 0; j < MAX_NEIGHBORS; j++) {
          	neighbors[j].r_etx[i] = 0xffff;
          }
          parentChanged = 1;
        }
        else {
          dest_nodes[i].freshness--;
          if(dest_nodes[i].beaconTimer > 0)
            dest_nodes[i].beaconTimer--;
        }
      }
     /* choose New Reverse Parents */
      if(parentChanged)
        updateReverseParent();
    }
   /* update beacon system for each downstream destination */ 
    dlBeaconRequest = 0;
    if(num_dest > 0) {
      for(i = 0; i < MAX_NUM_DEST; i++) {        
        destforbeacon[i] = 0;
        if(dest_nodes[i].id == BROADCAST_ADDR)
           continue;
        
        if (dest_nodes[i].beaconTimer <= 0) {
          dlBeaconRequest  = 1;
          destforbeacon[i] = 1;            

          if (dest_nodes[i].r_nextHop == BROADCAST_ADDR) {
            resetBeaconInterval(&dest_nodes[i]);
          }
          else {
            increaseBeaconInterval(&dest_nodes[i]);
          }
        }
        else if (dest_nodes[i].r_nextHop == BROADCAST_ADDR) {
          resetBeaconInterval(&dest_nodes[i]);
        }       
      }
    }
    
#ifdef CTP_PLUS_DEBUG        
    if((beacon_print_timer--)==0) {printTable(neighbors,"Updated freshness"); beacon_print_timer=PRINT_INTERVAL;}       
#endif
    
    mos_mutex_unlock(&neighborTableLock);   

    mos_mutex_lock(&optionlock);
   /* I have no valid LEEP info yet */
    if(hasNoLIEntry) { 
      options |= CTP_OPT_PULL;     // set PULL bit
    }
    else {
      options &= (~CTP_OPT_PULL);  // unset PULL bit
    }
   /* unset congestion bit here for this vanilla plain version of ctp */
    if(dest != BROADCAST_ADDR)
    {
      options &= (~CTP_OPT_ECN) ;  // unset congestion bit
    }  
    mos_mutex_unlock(&optionlock);         
    
   /* update beacon system for upstream cast to base */  
    mos_mutex_lock(&beaconMutex);
    if(beaconTimer > 0)
       beaconTimer--;
    
   /* produce a normal beacon packet */ 
    if (beaconTimer <= 0) {
      net_send(&beaconPacket, CTP_PROTO_ID, CTP_DUMMY_PORT, CTP_NEW_PACKET, AM_CTP_ROUTING);

      if (dest == BROADCAST_ADDR) {
        resetBeaconInterval(NULL);
      }
      else {
        increaseBeaconInterval(NULL);
      }
    }
    else if (dest == BROADCAST_ADDR) {
      resetBeaconInterval(NULL);
    }
   /* produce a beacon packet for downstream casts*/
    mos_mutex_unlock(&beaconMutex);        
    if(dlBeaconRequest)
       net_send(&beaconPacket2, CTP_PROTO_ID, CTP_DUMMY_PORT, CTP_NEW_PACKET, AM_CTP_DL_ROUTING, destforbeacon); 
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


void updateRoutingTable_sup(uint16_t source, ctp_data_header_t* rh) {
  uint8_t i, j;
  uint16_t dest; 
  
  mos_mutex_lock(&neighborTableLock);

 /* If the neighbor is already in the table, update it; otherwise, do bother then. */  
  if( (i = getNeighborIndex(source)) == MAX_NEIGHBORS ) {
     mos_mutex_unlock(&neighborTableLock);  
     return;
  }

  neighbors[i].freshness = NEW_FRESH;   

  /* only applied to node now */
  if(!is_base && (rh->type == AM_CTP_DATA)) {
    if ( neighbors[i].etx != b2w(rh->etxHigh, rh->etxLow) )  {
     /* Update neighbor's ETX */  
      neighbors[i].etx = b2w( rh->etxHigh, rh->etxLow ) ;
  
     /* Update my parent, bestEtx and nextHop for upstream cast */  
      updateForwardParent();  
    }
  }  
  if(rh->type == AM_CTP_DL_DATA) {
    ctp_dl_data_header_t * rrh = (ctp_dl_data_header_t *) rh;
    dest = b2w( rrh->destHigh, rrh->destLow );
      
   /* If the dest is already in the table, update it */    
    if( (j = getDestIndex(dest))  < MAX_NUM_DEST ) {
      dest_nodes[j].freshness = NEW_DL_FRESH;
      if( (dest != myid) && neighbors[i].r_etx[j] != b2w(rrh->etxHigh, rrh->etxLow) ) {
        neighbors[i].r_etx[j] = b2w(rrh->etxHigh, rrh->etxLow);

       /* Update my parent, bestEtx and nextHop for downstream cast */  
        updateReverseParent();           
      }
    }
  }
  
#ifdef CTP_PLUS_DEBUG  
  printTable(neighbors, "recvd a DT frame");    
#endif
  
  mos_mutex_unlock(&neighborTableLock);  
}

/* investigate the attached leep frame at the footer */
uint8_t extractLqi(uint8_t nentry, uint8_t * data)
{
  uint8_t i;
  leep_entry_t * et;  
  
 /* Be careful not to exceed MAX_LIENTRIES */
  for(i = 0 ; i < min( nentry, MAX_LIENTRIES ); i++) {
    et = (leep_entry_t *)(data + i * 3);
    if( b2w( et->node_idHigh, et->node_idLow ) == myid ) {
      return et->lq;        
    }
  }
  return 0;  
} 

void updateReverseETX( ctp_dl_routing_header_t * rrh, routing_entry_t * ent)
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

    /* If the dest is already in the table, update it */
     if( (j = getDestIndex(dest)) < MAX_NUM_DEST) {
       if(dest == myid) {
         dest_nodes[j].freshness = NEW_DL_FRESH;
       }
       else {
         ent->r_etx[j] = b2w(dt->r_etxHigh, dt->r_etxLow);
       
        /* we will let invalid etx expires sooner by reducing its freshness a bit */
         if(ent->r_etx[j] < MAX_ETX)
           dest_nodes[j].freshness = NEW_DL_FRESH; 
         else
           dest_nodes[j].freshness = (dest_nodes[j].freshness > 3) ? (dest_nodes[j].freshness - 3) : 0;
       }
     }      
    /* else get an empty r_Etx column, if there's any */
     else if(!is_base && dest != BROADCAST_ADDR) {  
       if( (j = getDestIndex(BROADCAST_ADDR)) < MAX_NUM_DEST) {
         initDLDestEntry(&dest_nodes[j]);
         if(dest != myid) {
           ent->r_etx[j] = b2w(dt->r_etxHigh, dt->r_etxLow);
         }
         dest_nodes[j].id = dest;
         dest_nodes[j].freshness = NEW_DL_FRESH;
       }
     }                                                          
   }
}

void updateRoutingTable(uint16_t source, ctp_routing_header_t* rh) {
  uint8_t i, j, lqi = 0;
  uint16_t worstMetric;    
  ctp_dl_routing_header_t * rrh;    

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
      initNeighborEntry( &neighbors[i] );
      neighbors[i].addr = source;
    }
  }
  if (i < MAX_NEIGHBORS) {     
    neighbors[i].freshness = NEW_FRESH;

   /* Update PRR */
    neighbors[i].prr  = (neighbors[i].prr) << ((rh->seqno > neighbors[i].lastrecvdseqno) ? 
                 (rh->seqno-neighbors[i].lastrecvdseqno) : (0xff-neighbors[i].lastrecvdseqno + rh->seqno+1)); 
    neighbors[i].prr |= 1;  // set the lsb to be 1
    neighbors[i].lastrecvdseqno = rh->seqno;
   
   /* Update my in-bound lqi from neighbor[i] to me */
    for(j = 0, lqi = MAX_LQI+1; j < PRR_WINDOWSIZE; ++j) 
      lqi -= ((neighbors[i].prr >> j) & 0x01);  
    neighbors[i].ib_lqi = (lqi > MAX_LQI) ? 0xff : lqi;
    
    if(rh->type == AM_CTP_ROUTING) {            
     /* Update my out-bound lqi to neighbor[i].addr */     
      if( (lqi = extractLqi((rh->leepEntries) >> 4, &(rh->data[0])) ) > 0 ) 
        neighbors[i].ob_lqi = lqi;       
     
     /* only applied to node */
      if(!is_base) {
       /* Update neighbor's ETX */   
        neighbors[i].etx = b2w( rh->etxHigh, rh->etxLow );
        
       /* Update my parent, bestEtx and nextHop */ 
        updateForwardParent();
      } 
     /* Update reverse parent, bestEtx and nextHop */
      updateReverseParent();      
    }  
    else if(rh->type == AM_CTP_DL_ROUTING) { 
      rrh = (ctp_dl_routing_header_t *) rh;       
      if( (lqi = extractLqi((rrh->leepEntries) >> 4, &(rrh->data[0])) ) > 0 )
        neighbors[i].ob_lqi = lqi;
            
      if(!is_base) {    
        neighbors[i].etx = b2w( rrh->etxHigh, rrh->etxLow );
        updateForwardParent(); 
      }           
     /* Update my reverse ETX list */ 
      updateReverseETX( rrh, &neighbors[i]);
      
     /* Update reverse parent, bestEtx and nextHop */
      updateReverseParent();
    }
  }
  
#ifdef CTP_PLUS_DEBUG  
  printTable(neighbors, "recvd a RT frame");    
#endif
  
  mos_mutex_unlock(&neighborTableLock);
}


void ctp_proto_init(void) {  
  uint8_t i;

 /* initialize my node_id */
  myid = mos_node_id_get();   

 /* status msg: if I am the base or node */
  printf("CTP STATS MSG: Initializing %s sensor %d\n", is_base? "BASE":"NODE", myid);  

  mos_mutex_init(&beaconMutex);
  mos_mutex_init(&neighborTableLock);
  mos_mutex_init(&optionlock);

  seed_random();
  mos_led_off(0);
  mos_led_off(1);
  mos_led_off(2);
  
 /* initialize the option */
  options = CTP_OPT_PULL;

 /* initialize beacon system */
  beaconTimerCount = BEACON_TIMER_MIN;
  beaconTimer = BEACON_TIMER_MIN;
  beaconSeqNo = 250;
  lastParent = 0;
  lastDest = 0;
  
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
  for(i = 0; i < MAX_NUM_DEST; i++) {
    initDLDestEntry(&dest_nodes[i]);  
  } 

 /* ctp uses a backend thread */ 
  mos_thread_new(beacon_thread, 384, PRIORITY_NORMAL);

 /* register ctp protocal */
  net_proto_register(CTP_PROTO_ID, net_ctp_proto_send, net_ctp_proto_recv, net_ctp_proto_ioctl);

#ifdef CTP_PLUS_DEBUG   
  mos_thread_new_havestack(debug_thread, DEBUG_STACK_SIZE, debug_stack, PRIORITY_NORMAL);
#endif  
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
    if (dt->beaconTimerCount > BEACON_TIMER_MAX) {
        dt->beaconTimerCount = BEACON_TIMER_MAX;
    }
    dt->beaconTimer = beaconTimerCount;
  } else {    
    beaconTimerCount *= 2;
    if (beaconTimerCount > BEACON_TIMER_MAX) {
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
    case AM_CTP_DATA:    return (is_base == 1);
    case AM_CTP_DL_DATA: return (dest == myid);
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
   
#ifdef CTP_PLUS_DEBUG   
     (dataHeader->type == AM_CTP_DATA) ? printDTheader(dataHeader, RECEIVED, htons(buf->source)) : 
       printDTheader_dl((ctp_dl_data_header_t*)dataHeader, RECEIVED, htons(buf->source));   
#endif   

  /* update my routing table */
   updateRoutingTable_sup(htons(buf->source), dataHeader); 
  
  /* if I am the destinaton, pass the packet to the application port */
   if( isDestination(dataHeader->type,  dest) ) {
   	
#ifndef KEEP_CTP_HEADER
    /* strip off the net layer header */      
     ret = getHeaderSize(dataHeader->type); 
     for(i = ret; i < buf->size; ++i)
        buf->data[i-ret] = buf->data[i];
     buf->size -= ret;
#endif

    /* else pass to the upper layer a complete the data frame including CTP header */
     return is_app_waiting_on(port);
   }
 
  /* otherwise consider forwarding to my next hop */
   mos_mutex_lock(&neighborTableLock);
   myetx = getETX( dataHeader->type,  dest) ;   //bestETX;
   mos_mutex_unlock(&neighborTableLock);
 
   if (dataHeader->options & CTP_OPT_PULL || dataHeader->options & CTP_OPT_ECN) {
     mos_mutex_lock(&beaconMutex);
     resetBeaconInterval(NULL);
     mos_mutex_unlock(&beaconMutex);
   }

  /* looping detection */
   if ( (myetx > b2w(dataHeader->etxHigh, dataHeader->etxLow) ) ||
        (myid == b2w(dataHeader->originHigh, dataHeader->originLow)) ) 
   {
     /* inconsistent path metrics or loopback data packet */
      if(dataHeader->type == AM_CTP_DATA) {
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
   
     if(ctp_append_hops) { //append my id in the payload footer
       buf->data[buf->size] = (myid >> 8);
       buf->data[buf->size+1] = (myid & 0xff);
       buf->size += 2;
     }  

     ret = net_send(buf, CTP_PROTO_ID, port, CTP_EXISTING_PACKET);  
    /* If the routing is not established yet, the data packet cannot be forwarded out. */
     if(ret) 
     {
      /* This node is congested */
       printf("\nCTP STATS MSG: Sensor is 'congested'(not route). Dropping data packets from %d to %d.\n", 
                               b2w(dataHeader->originHigh, dataHeader->originLow), dest);        
       
       if(dataHeader->type == AM_CTP_DATA) {
        /* Broadcast my congestion status */
         mos_mutex_lock(&optionlock);
         options |= CTP_OPT_ECN;  // set congestion bit
         mos_mutex_unlock(&optionlock);
       
         mos_mutex_lock(&beaconMutex);
         beaconFast(NULL);
         mos_mutex_unlock(&beaconMutex);
       }
     }
   }    
  /* won't hand the packet to any application port */
   return false;
}


boolean ctp_handle_beacon_packet(comBuf* buf)
{
  uint16_t myEtx;
  ctp_routing_header_t* routingHeader;
  
 /* Extract the frame header */
  routingHeader = (ctp_routing_header_t*)buf->data;

#ifdef CTP_PLUS_DEBUG   
    if(routingHeader->type == AM_CTP_ROUTING) printRTheader(routingHeader, RECEIVED, htons(buf->source));
    else printRTheader_dl((ctp_dl_routing_header_t*)routingHeader, RECEIVED, htons(buf->source));
#endif  

 /* update my routing table */
  updateRoutingTable(htons(buf->source), routingHeader);

  if (routingHeader->options & CTP_OPT_PULL  || routingHeader->options & CTP_OPT_ECN) {
    mos_mutex_lock(&beaconMutex);
    resetBeaconInterval(NULL);
    mos_mutex_unlock(&beaconMutex);
  }

 /* Looping detection */ 
  if( myid == b2w(routingHeader->parentHigh, routingHeader->parentLow) ) 
  {
    mos_mutex_lock(&neighborTableLock);     
    myEtx = getETX(routingHeader->type,  BASE_ALIAS);
    mos_mutex_unlock(&neighborTableLock);
    if ( myEtx > b2w(routingHeader->etxHigh, routingHeader->etxLow) ) 
    {
     /* inconsistent path metrics */ 
      mos_mutex_lock(&beaconMutex);
      beaconFast(NULL);
      mos_mutex_unlock(&beaconMutex);
    }
  }
  if(routingHeader->type == AM_CTP_DL_ROUTING) {
    uint8_t i, j, k;
    ctp_dl_routing_header_t * rrh = (ctp_dl_routing_header_t *) routingHeader;
    dest_entry_t * dt;
   
   /* If rrh->leepEntries is not a valid number, don't bother then. */
    if( (rrh->leepEntries>>4) <= MAX_LIENTRIES) {      
      
      mos_mutex_lock(&neighborTableLock);
      k = (rrh->leepEntries >> 4) * 3; 
      for(i = 0 ; i < min( rrh->destEntries, MAX_DESTENTRIES ); i++) {
        dt = (dest_entry_t *)&(rrh->data[k + i * 6]); 
     
        if((myid == b2w(dt->r_parentHigh, dt->r_parentLow)) )
        {        
         if( ((j = getDestIndex(b2w(dt->r_parentHigh, dt->r_parentLow))) < MAX_NUM_DEST) &&
             (dest_nodes[j].r_bestETX  >  b2w(dt->r_etxHigh, dt->r_etxLow))  )         
           beaconFast(&dest_nodes[j]);
        }  
      }
      mos_mutex_unlock(&neighborTableLock);
    }
  } 
  
 /* won't hand the BEACON frame to any application */
  return false;
}


int8_t net_ctp_proto_send(comBuf *buf, va_list args) {
  
    uint8_t frame_type , add_ctp_header;
    uint16_t dest_in_dl;
    uint8_t* destforbeacon;
    
    ctp_data_header_t   *   header;
    ctp_routing_header_t*   rh;
    ctp_dl_data_header_t   *   dlheader;
    ctp_dl_routing_header_t*   dlrh;
    
    uint16_t dest, myEtx;
    uint8_t i, j, entries, index, dest_entries ;

#ifdef CTP_PLUS_DEBUG       
     printf("\nproto_send (comBuf %x)\n", buf);
#endif
       
    if(buf==NULL) {
        /* a NULL packet will not actually sent out on link layer */
        return 0;
    }
    
    /* grab function parameters */
    add_ctp_header = (uint8_t)va_arg(args, int);
    frame_type = (add_ctp_header == CTP_NEW_PACKET)? (uint8_t)va_arg(args, int): buf->data[0];
            
    if((add_ctp_header == CTP_NEW_PACKET) && (getFrameCat(frame_type) == DATA_FRAME)) {

        index = (frame_type == AM_CTP_DL_DATA) ? CTP_DL_DATA_HEADER_SIZE : CTP_DATA_HEADER_SIZE;
        /* bail if packet is too big */
        if((buf->size + index) > COM_DATA_SIZE){
            printf("CTP ERROR MSG: payload is too big.");
            return 1;
        }        
        /* Shift the payload to insert the CTP DL header*/
        for(i = 1; i <= buf->size; i++)
            buf->data[buf->size + index -i] = buf->data[buf->size -i];
        buf->size = index + buf->size;
    }
    
    if(frame_type == AM_CTP_DATA) {
        mos_led_toggle(2);
        /* Extract the CTP Data frame header */
        header = (ctp_data_header_t*)buf->data;
        
        if(add_ctp_header == CTP_NEW_PACKET) {
            header->type = AM_CTP_DATA;
            header->originSeqNo = (originSeqNo++);
            header->originHigh = myid >> 8;
            header->originLow= myid & 0xff;
            header->thl = 0;
            header->collectId = AM_OSCILLOSCOPE;
        }        
        header->options = getOptions();
        
        /* get my bestEtx and nextHop, as well as the Option for the current data packet */
        mos_mutex_lock(&neighborTableLock);
        dest = getNextHop(AM_CTP_DATA, BASE_ALIAS);
        myEtx = (header->options & CTP_OPT_ECN) ? 0xffff : getETX(AM_CTP_DATA, BASE_ALIAS);
        mos_mutex_unlock(&neighborTableLock);

        if(myEtx > OPT_ETX) {
	     ulPower += ((31 - ulPower) / 2);
    	     net_ctp_proto_ioctl(CTP_SET_POWER,ulPower)
        }
        else if(myEtx < OPT_ETX) {
	     ulPower = ulPower /2;
    	     net_ctp_proto_ioctl(CTP_SET_POWER,ulPower)
	 }
		
        header->etxHigh = myEtx >> 8;
        header->etxLow = myEtx & 0xff;	
        
        /* store nexthop addr in this unused comBuf field */
        buf->signal = htons(dest);
        
#ifdef CTP_PLUS_DEBUG           
        printDTheader(header, dest == BROADCAST_ADDR?DROPPED:SENT, dest);
#endif
        
        /* if routing is not found yet, data packet cannot be sent out. */
        if(dest == BROADCAST_ADDR)
            return 1;        
    }    
    //net_send(..,..,..,CTP_NEW_PACKET,CTP_DL_DATA,dest)
    else if (frame_type == AM_CTP_DL_DATA){
        mos_led_toggle(4);        
        /* Extract the CTP DL Data frame header */
        dlheader = (ctp_dl_data_header_t*)buf->data;
        
        if(add_ctp_header == CTP_NEW_PACKET) {    
            dest_in_dl = (uint16_t)va_arg(args, int);
                    
            dlheader->type = AM_CTP_DL_DATA;
            dlheader->originSeqNo = (originSeqNo++);
            dlheader->originHigh = myid >> 8;
            dlheader->originLow= myid & 0xff;
            dlheader->thl = 0;
            dlheader->collectId = AM_OSCILLOSCOPE;
            dlheader->destHigh = dest_in_dl >> 8;
            dlheader->destLow = dest_in_dl & 0xff;
        }
        else
          dest_in_dl = b2w(dlheader->destHigh, dlheader->destLow);       
        
        if(is_base) {
           connectTo(dest_in_dl);
        }

        dlheader->options = getOptions();
        
        /* get my bestEtx and nextHop, as well as the Option for the current data packet */
        mos_mutex_lock(&neighborTableLock);        
        dest = getNextHop(AM_CTP_DL_DATA, dest_in_dl);
        myEtx = getETX(AM_CTP_DL_DATA, dest_in_dl); //(dlheader->options & CTP_OPT_ECN) ? 0xffff : getETX(AM_CTP_DL_DATA,dest_in_dl);        
        mos_mutex_unlock(&neighborTableLock);

        if(myEtx > OPT_ETX) {
	     dlPower += ((31 - dlPower) / 2);
    	     net_ctp_proto_ioctl(CTP_SET_POWER,dlPower)
        }
        else if(myEtx < OPT_ETX) {
	     dlPower = dlPower /2;
    	     net_ctp_proto_ioctl(CTP_SET_POWER,dlPower)
	 }
		
        dlheader->etxHigh = myEtx >> 8;
        dlheader->etxLow = myEtx & 0xff;
        
        /* store nexthop addr in this unused comBuf field */
        buf->signal = htons(dest);
        
#ifdef CTP_PLUS_DEBUG           
        printDTheader_dl(dlheader, dest == BROADCAST_ADDR?DROPPED:SENT, dest);
#endif        
        
        /* if routing is not found yet, data packet cannot be sent out. */
        if(dest == BROADCAST_ADDR)
          return 1;
    }
    else if (frame_type == AM_CTP_DL_ROUTING){
        mos_led_toggle(5);
        /* Beacon frame always has no payload*/        
        /* Extract the CTP DL BEACON frame header */
        dlrh = (ctp_dl_routing_header_t*)buf->data;
        
        destforbeacon = (uint8_t*) va_arg(args, uint8_t*);
        
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
                dlrh->data[3 * entries] = (neighbors[index].addr >> 8);       //node_idHigh 
                dlrh->data[3 * entries + 1] = (neighbors[index].addr & 0xff); //node_idLow                
                /* copy neighbor's in-bound lqi (lqi from the neighbor to me) */
                dlrh->data[3 * entries + 2] = (uint8_t)neighbors[index].ib_lqi;                
                entries++;
            }
        }
        lastParent = (lastParent + i) % MAX_NEIGHBORS;
        dlrh->leepEntries = entries << 4;
        
        /* Populate Dest Entries */
        for(i = 0, dest_entries = 0; i<MAX_NUM_DEST && dest_entries < MAX_DESTENTRIES; i++){
           index = (lastDest + i) % MAX_NUM_DEST;  
           if(destforbeacon[index]){
              j = LEEP_ENTRY_SIZE * entries + DEST_ENTRY_SIZE * dest_entries;
              dlrh->data[j + 0] = (dest_nodes[index].id >> 8);
              dlrh->data[j + 1] = (dest_nodes[index].id & 0xff);
              dlrh->data[j + 2] =  dest_nodes[index].r_nextHop >> 8;
              dlrh->data[j + 3] =  dest_nodes[index].r_nextHop & 0xff;
              dlrh->data[j + 4] =  dest_nodes[index].r_bestETX >> 8;
              dlrh->data[j + 5] =  dest_nodes[index].r_bestETX & 0xff;
              dest_entries++;
           }
        }
        lastDest = (lastDest + i) % MAX_NUM_DEST;    
        dlrh->destEntries = dest_entries;        
        buf->size = CTP_DL_ROUTING_HEADER_SIZE + LEEP_ENTRY_SIZE*entries + DEST_ENTRY_SIZE * dest_entries;        
        
        /* get my bestEtx and nextHop for upstream cast */
        dest  = (dlrh->options & CTP_OPT_ECN) ? 0xffff : getNextHop(AM_CTP_ROUTING, BASE_ALIAS);
        myEtx = getETX(AM_CTP_ROUTING, BASE_ALIAS);

        mos_mutex_unlock(&neighborTableLock);      

        dlrh->etxHigh = myEtx >> 8;
        dlrh->etxLow = myEtx & 0xff;
        dlrh->parentHigh = dest >> 8;
        dlrh->parentLow  = dest & 0xff;
        
        /* store nextHop address in this unused comBuf field */
        buf->signal = BROADCAST_ADDR; //always BROADCASTing beacon
        
#ifdef CTP_PLUS_DEBUG           
        printRTheader_dl(dlrh, SENT, 0);
#endif        
    }
    else if (frame_type == AM_CTP_ROUTING) {
        mos_led_toggle(3);
        /* Beacon frame always has no payload*/
        
        /* Extract the CTP BEACON frame header */
        rh = (ctp_routing_header_t*)buf->data;
        
        if(add_ctp_header == CTP_NEW_PACKET) {
            rh->type = AM_CTP_ROUTING;
            rh->seqno = (beaconSeqNo++);
        }
        
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
        dest  = (rh->options & CTP_OPT_ECN) ? 0xffff : getNextHop(AM_CTP_ROUTING, BASE_ALIAS);
        myEtx = getETX(AM_CTP_ROUTING, BASE_ALIAS); 
               
        mos_mutex_unlock(&neighborTableLock);     

        rh->etxHigh = myEtx >> 8;
        rh->etxLow = myEtx & 0xff;
        rh->parentHigh = dest >> 8;
        rh->parentLow  = dest & 0xff;
        
        /* store nextHop address in this unused comBuf field */
        buf->signal = BROADCAST_ADDR; //always BROADCASTing beacon
        
#ifdef CTP_PLUS_DEBUG           
        printRTheader(rh, SENT, 0);
#endif        
    }
    else {
        printf("CTP ERROR MSG: unknown frame type, cannot send out the packet.\n");
        return 1;
    }
    
    /* Wait the random time before sending another to avoid collision */
    i = rand_mlcg_8();
    mos_udelay(i);
    
    /* send it */
    return 0;
}


boolean net_ctp_proto_recv(comBuf *buf, uint8_t **footer, uint8_t port) 
{
  uint8_t *bytes, realrssi;   

#ifdef CTP_PLUS_DEBUG     
  printf("receive thread (comBuf %x)\n", buf);       
#endif
     
  if(buf==NULL) {
    printf("CTP STATS MSG: RADIO RECV Interface busy.\n"); 
   /* prevent net_recv_packet() from freeing a NULL packet*/
    return true;
  }
  if(htons(buf->source) == myid) {
    printf("CTP STATS MSG: discard loopbacked packet.\n");
   /* Loopback packet. free it. */
    return false;
  }

 /* Extract the rssi from "link quality indice" reported by link layer */
  bytes  = (uint8_t *) &(buf->signal);      
  realrssi = (int8_t)(bytes[0]) + 128;

  if (realrssi < RSSI_THRESHOLD)
  {
#ifdef CTP_PLUS_DEBUG    
     printf("bad rssi %d, discard the %s frame from %d\n", 
                 realrssi, getFrameCat(buf->data[0]) == DATA_FRAME ? "DATA":"BEACON", htons(buf->source));
#endif                 
   /* free packet memory */
    return false;
  }
  
  if(buf->size < getHeaderSize(buf->data[0]) )
  {
#ifdef CTP_PLUS_DEBUG   
      printf("invalid frame size %C, discard the %s frame from %d\n", 
                 buf->size, getFrameCat(buf->data[0]) == DATA_FRAME ? "DATA":"BEACON", htons(buf->source));
#endif                 
   /* free packet memory */
    return false;
  }
  
 /* The first byte of the header indicates the frame type*/
  if (getFrameCat(buf->data[0]) == DATA_FRAME) //AM_CTP_DATA)
  {
    mos_led_toggle(1); 
    return ctp_handle_data_packet(buf, port);
  }
  else if (getFrameCat(buf->data[0]) == ROUTING_FRAME) //AM_CTP_ROUTING) 
  {
    mos_led_toggle(0);    
    return ctp_handle_beacon_packet(buf);
  }
  else {
    printf("CTP Error MSG: unknow frame type received from %d.\n", htons(buf->source));
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
      bestETX = 0;
      nextHop = myid;
      printf("Set sensor %d to be the base station.\n", mos_node_id_get());
      break;
   /* make this sensor a node */    
    case CTP_SET_IS_NODE:
      is_base = false;
      parent  = 0;
      bestETX = 0xffff;
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


uint8_t connectTo(uint16_t dest){
    uint8_t i, j, fresh;
    
    if(dest== BROADCAST_ADDR) 
       return MAX_NUM_DEST;  
    mos_mutex_lock(&neighborTableLock);
    if( (i = getDestIndex(dest)) >= MAX_NUM_DEST ) {
      if( (i = getDestIndex(BROADCAST_ADDR)) >= MAX_NUM_DEST) {
        for(j = 0, i = MAX_NUM_DEST, fresh = NEW_FRESH + 1; j < MAX_NUM_DEST; j++) {
          if(fresh > dest_nodes[j].freshness) {
            fresh = dest_nodes[j].freshness;
            i = j; 
          }
        }
      }    
      initDLDestEntry(&dest_nodes[i]);
      dest_nodes[i].id = dest;
      dest_nodes[i].freshness =  NEW_DL_FRESH;
      updateReverseParent();
      mos_mdelay(5);     
    }
    else {
    	dest_nodes[i].freshness =  NEW_DL_FRESH;
    }    	
    fresh = (dest_nodes[i].r_nextHop == BROADCAST_ADDR);
    mos_mutex_unlock(&neighborTableLock);
    
    if(fresh)
       mos_thread_sleep(3*BEACON_INTERVAL);
    return i;
}

boolean blockConnectTo(uint16_t dest){
    uint8_t dest_node_i = connectTo(dest);
    uint8_t count = 0;
    // may need to use neighborTableLock
    while((dest_nodes[dest_node_i].r_nextHop == BROADCAST_ADDR) && ((count++)< 10))
    {        
      mos_thread_sleep(1000);        
    }
    return (dest_nodes[dest_node_i].r_nextHop != BROADCAST_ADDR);
}


void close(uint16_t dest) {
    uint8_t i, j;
    
    if(dest== BROADCAST_ADDR) 
       return;   

    mos_mutex_lock(&neighborTableLock);
    if( (i = getDestIndex(dest)) < MAX_NUM_DEST ) {
          initDLDestEntry(&dest_nodes[i]);          
          for(j = 0; j < MAX_NEIGHBORS; j++) {
          	neighbors[j].r_etx[i] = 0xffff;
          }  
    }
    mos_mutex_unlock(&neighborTableLock);
}

