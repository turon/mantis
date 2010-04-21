//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    terminal_driver.c                                             */
/* Author     Charles Gruenwald III : gruenwal@cs.colorado.edu            */
/*   Date: 03/12/04                                                       */
/*                                                                        */
/* test app for the terminal driver                                       */
/**************************************************************************/
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "com.h"
#include "mos.h"

comBuf *packet;

uint8_t str_to_packet(comBuf * packet, char * string);

void test_recv(void)
{
  comBuf to_send;
  str_to_packet(&to_send,"hi mom\n");
  com_send(IFACE_TERMINAL, &to_send);

  str_to_packet(&to_send,"type something to test (it should echo)\n");
  com_send(IFACE_TERMINAL, &to_send);

  packet = com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      com_send(IFACE_TERMINAL,packet);
      com_free_buf(packet);
    }
}
uint8_t str_to_packet(comBuf * packet,char * string)
{
  char * cursor=string;

  if(strlen(string) > 64)
    return -1;

  //reset size
  packet->size=0;
  while(*cursor)
    packet->data[packet->size++]=*cursor++;

  packet->data[packet->size]='\0';
    
}
void start(void)
{
  mos_thread_new(test_recv,128,PRIORITY_NORMAL);
}
