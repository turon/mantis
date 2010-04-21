//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**
 * flash.c contains the functions to read and write flush
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "flash.h"
#include "serial.h"
#include "mcs.h"
#include "crc.h"
#include "com.h"

extern int go;

int send_byte(char byte);

int send_address(uint16_t addr)
{
   comBuf send;
   send.size=2;
   *(uint16_t *)&send.data[0] = addr;
   com_send(IFACE_SERIAL,&send);
   return 0;
}

int check_recv (int code)
{
   comBuf *recv_packet;
   recv_packet = com_recv (IFACE_SERIAL);
   if (recv_packet == NULL) {
      fprintf (stderr,"Got a bad packet\n");
      com_free_buf (recv_packet);
      return -1;
   }
   if (recv_packet->data[0] != code) {
      if(recv_packet->data[0] == LOADER_PRESENT) //oops node must have restarted
      {
	 usleep(10);
	 send_byte(SHELL_PRESENT);
	 com_free_buf (recv_packet);
	 return NODE_RESTARTED;
      }
      if(recv_packet->data[0] == LOADER_PRESENT_PONG) //node responding to extra shell_present
      {
	 usleep(10);
	 com_free_buf (recv_packet);
	 return check_recv(code);
      }
      if(recv_packet->data[0] == UNKNOWN && recv_packet->size==2)
      {
	 fprintf(stderr,"\nNode didn't understand command: %d\n",
		 recv_packet->data[1]);
      }
      else
      {
	 fprintf (stderr,"Got %d instead of %d\n", recv_packet->data[0], code);
	 if(recv_packet->size > 1)
	    fprintf (stderr,"Packet size [%d]: %d \"%s\"",recv_packet->size,
		    recv_packet->data[0],recv_packet->data);
      }
      com_free_buf (recv_packet);
      return COMMAND_MISS;
   }
   com_free_buf (recv_packet);
   return COMMAND_MATCH;
}

int send_byte(char byte)
{
   comBuf out_packet;
   out_packet.size=1;
   out_packet.data[0]=byte;
   com_send(IFACE_SERIAL,&out_packet);

   return 0;
}

int write_image(char *image, int len, update_callback callback_func)
{
   int page;
   int num_pages = len / 256;
   uint16_t *addr_p;
   uint16_t image_index, page_index;
   int ret;
   comBuf addr_packet, page_packet;

   /* Start the transfer by sending WRITE_IMAGE */
   addr_packet.size=3;
   addr_packet.data[0] = LOAD_IMAGE;
   addr_p = (uint16_t *)&addr_packet.data[1];
   addr_p[0] = len;
   com_flush(IFACE_SERIAL);

   com_send (IFACE_SERIAL, &addr_packet);

   ret=check_recv(LOAD_ACK);
   while(ret != COMMAND_MATCH)
   {
      switch(ret){
      case  NODE_RESTARTED:
	 fprintf(stderr,"node restarted (why here?) sending LOAD_IMAGE.\n");
	 com_flush(IFACE_SERIAL);
	 com_send (IFACE_SERIAL, &addr_packet);
	 break;
      case COMMAND_MISS:
	 return LOAD_COMMAND_FAILURE;
	 break;
      }						
      ret=check_recv(LOAD_ACK);
   }
   
   /* Must account for pages that are not full. */
   if((len % 256) > 0)
      num_pages++;
   
   if(callback_func == NULL) {
      fprintf(stderr,"Data: %d    Flash Pages: %d\n", len, num_pages);
      fprintf(stderr,"Pages Complete: 000");
   }
   
   fflush(stdout);
   
   image_index = 0;
   page_index = 0;
   page = 0;
   
   while(image_index < len)
   {
      /* Send the page address */
      addr_packet.size = 2;
      addr_p = (uint16_t *)addr_packet.data;
      //addr_p[0] = page * 128;
      uint16_t crc;
      if((len - image_index) >= PAGE_SIZE_BYTES) {
	 crc = crc_compute(&image[image_index], PAGE_SIZE_BYTES);
      } else {
	 crc = crc_compute (&image[image_index], len - image_index);
      }

      addr_p[0] = crc;
      com_send(IFACE_SERIAL, &addr_packet);
      
      /* Now send the page. */
      page_index = 0;
      while(page_index < PAGE_SIZE_BYTES)
      {
	 if((len - image_index - page_index) >=COM_DATA_SIZE){
	    memcpy (page_packet.data,
		    &image[image_index + page_index], COM_DATA_SIZE);
	    
	    page_packet.size = COM_DATA_SIZE;
	    com_send (IFACE_SERIAL, &page_packet);

	    
	    //if(check_recv(PACKET_ACK) != COMMAND_MATCH)
	    //  return PAGE_PACKET_FAILURE;
	    
	    page_index += COM_DATA_SIZE;
	 }
	 else // The last packet is possibly a partial comBuf size
	 {
	    memcpy(page_packet.data,
		   &image[image_index + page_index],
		   len - image_index - page_index);
	    
	    page_packet.size = len - image_index - page_index;
	    com_send(IFACE_SERIAL, &page_packet);
	    page_index += len -image_index - page_index;

	    //if(check_recv(PACKET_ACK) != COMMAND_MATCH)
	    //  return PAGE_PACKET_FAILURE;
	    
	    break;
	 }
      }
      image_index += page_index;      
      
      if(check_recv(CRC_OK) != COMMAND_MATCH) {
	 fprintf (stderr,"\nCRC Check failed\n");
	 break;
	 //return ret;
      }

      page++;
      if(callback_func != NULL)
	 callback_func(page, num_pages);
      else
	 fprintf(stderr,"\b\b\b%.3d", page);
	 
      fflush(stdout);
   }
   
   if (check_recv (IMAGE_ACK) != COMMAND_MATCH)
      return ret;

   if(callback_func == NULL)
      fprintf(stderr,"...Done.\n");
   
   return SUCCESS;
}

int read_fuses(char *arg)
{
   comBuf *recv_pkt;
   send_byte(READ_FUSES);
   
   recv_pkt = com_recv (IFACE_SERIAL);
   if (recv_pkt == NULL)
      return -1;
   fprintf(stderr,"Fuse Low Byte      = %.2x\n", recv_pkt->data[0]);
   fprintf(stderr,"Fuse High Byte     = %.2x\n", recv_pkt->data[1]);
   fprintf(stderr,"Fuse Extended Byte = %.2x\n", recv_pkt->data[2]);
   fprintf(stderr,"Lock Bits          = %.2x\n", recv_pkt->data[3]);
   com_free_buf (recv_pkt);

   return 0;
}

int8_t read_flash(char *arg)
{
   int val, i, j;
   uint8_t byte;

   comBuf *recv_pkt;
   
   fprintf(stderr,"Enter flash byte address: 0x");
   scanf("%x", &val);
   
   send_byte(READ_FLASH);
   
   val >>= 1;

   if(send_address((uint16_t)val) == -1)
      return -1;
   
   for(i = 0; i < 16; i++)
   {
      for(j = 0; j < 16; j++)
      {
	 recv_pkt = com_recv(IFACE_SERIAL);
	 byte = recv_pkt->data[0];
	 fprintf(stderr, "%.2x", byte);
	 com_free_buf(recv_pkt);
      }
      fprintf(stderr, "\n");
   }

   recv_pkt = com_recv(IFACE_SERIAL);
   byte = recv_pkt->data[0];
   if(byte != FLASH_READ_COMPLETE)
   {
      fprintf(stderr, "Error ending read: %d!\n", byte);
      return -1;
   }
   
   return 0;
}
