//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/* remote control sends commands to the shell server */

#include <inttypes.h>

#include "printf.h"
#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "com.h"
#include "serial.h"

comBuf packet;

void remote_control()
{
   uint8_t i;



   comBuf *recvd;
  
   com_mode(IFACE_SERIAL, IF_LISTEN); //tell the uart we want to listen
   while(1)
   {
/*      IF_ZERO(&set);                     //reset the set of interfaces
      IF_SET(IFACE_TERMINAL, &set);      //add stdin to set
      IF_SET(IFACE_SERIAL, &set);        //add serial to set
      IF_SET(IFACE_UDP, &set);           //become a server
      com_select(&set,COM_BLOCK);                //wait for a device to respond
*/    
      //     if(IF_ISSET(IFACE_SERIAL,&set)){      //something on serial line
//	 recvd = com_recv(IFACE_SERIAL);    //retrieve data from serial
      // com_send(IFACE_TERMINAL, recvd);   //send data to terminal
	 //TODO: forward traffic over the network
      // com_free_buf(recvd);             
      // fflush (stdout);                 //show the string incase no \n
	      
   // else if(IF_ISSET(IFACE_TERMINAL,&set)){//user typed something
   recvd = com_recv(IFACE_TERMINAL);   //get the data
   com_send(IFACE_UDP,recvd);
   com_free_buf(recvd);
	 // if(!command_parse(local_commands,recvd->data)){//local command?
	 // send_to_nymph(recvd->data); 
	 //}

   }
}

void start(void)
{
   mos_thread_new(remote_control, 128, PRIORITY_NORMAL);
}
