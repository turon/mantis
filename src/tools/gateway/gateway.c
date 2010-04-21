//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* File:    gateway.c                                                     */
/* Author     Adam Torgerson    adam.torgerson@colorado.edu               */
/*   Date: 03/11/04                                                       */
/*                                                                        */
/* The gateway software, to be connected to a relay node and forward      */
/*   packets on to connect visualization clients.                         */
/**************************************************************************/

#include <stdbool.h>

#include "common.h"

#include "msched.h"
#include "cortex.h"
#include "arg.h"
#include "attributes.h"
#include "snet.h"
#include "cortex/include/net_event.h"
#include "com.h"
#include "serial.h"
#include "net_event_list.h"

static int lines_pending = 0;
static gboolean read_eof = FALSE;

GSList *conn_list = NULL;
GMutex *conn_mutex = NULL;
GMutex *write_mutex = NULL;

static void server_func (GServer *server, GConn *conn, gpointer data);
static void client_func (GConn *conn, GConnEvent *event, gpointer data);
static gboolean stdin_func (GIOChannel *chan, GIOCondition cond, gpointer data);
static void conn_write_func (gpointer data, gpointer user_data);
void server_thread (void);

/* TODO: unghettify this */
#define BUF_SIZE 4096

/* TODO: a better way to get the serial fd? */
extern int serialfd;

typedef struct {
   uint16_t from;
   uint16_t to;
   uint16_t event;
} node_net_event_t;

/* watches the serial port and forwards data to all clients */
void serial_select (void)
{
   IF_SET set;
   comBuf *recv_pkt;
   light_temp_event_t ev;
   
   while (1) {
      //IF_ZERO (&set);
      //IF_SET (IFACE_SERIAL, &set);
      // wait on the serial port and receive a packet when ready
      //com_select (&set, COM_BLOCK);
      debug ("gonna com_recv");
      recv_pkt = com_recv (IFACE_SERIAL);
      debug ("com_recvd");
      
      // unmarshall the packet into the event struct
      node_net_event_t *event = (node_net_event_t *)recv_pkt->data;
      //event->buffer = recv_pkt->data+6;
      ev.node_id = event->from;
      ev.light = recv_pkt->data[6];
      ev.temp = recv_pkt->data[7];
      //event->buffer[1];
      
      printf ("Sensor Node: %d light: %d temp: %d\n",
	      ev.node_id, recv_pkt->data[6], recv_pkt->data[7]);
      com_free_buf (recv_pkt);
      
      // call the write func for each connected client, give them the
      // recvd packet 
      if (conn_list != NULL) {
	 g_mutex_lock (conn_mutex);
	 g_slist_foreach (conn_list, conn_write_func, &ev);
	 g_mutex_unlock (conn_mutex);
      } else {
	 printf ("No clients connected yet\n");
      }
   }
}

int main (int argc, char *argv[])
{
   int port;
   GServer *server;
   GMainLoop *main_loop;
   // GIOChannel *std_in;

   printf ("MANTIS Gateway\n");

   gnet_init ();
   port = MANTIS_DEFAULT_PORT;

   // create the main loop 
   main_loop = g_main_loop_new (NULL, FALSE);

   arg_init (argc, argv);

   // create the server 
   server = gnet_server_new (NULL, port, server_func, NULL);
   if (!server) {
      printf ("Could not start server\n");
      exit (-1);
   }

   /* hook up stdin to a gio channel */
   /*
     std_in = g_io_channel_unix_new (fileno (stdin));
     g_io_add_watch (std_in, G_IO_IN | G_IO_ERR | G_IO_HUP | G_IO_NVAL,
     stdin_func, NULL);
   */
   
   // manually init xmos since we need our own main ()
   sched_init ();
   com_init ();
   serial_init ();
   com_mode (IFACE_SERIAL, IF_LISTEN);

   conn_mutex = g_mutex_new ();
   write_mutex = g_mutex_new ();

   mos_thread_new (serial_select, 128, PRIORITY_NORMAL);

   printf ("Waiting for connections...\n");

   // start the gnet loop, waiting for events, etc
   g_main_loop_run (main_loop);
   
   return 0;
}

/* this function is a callback for g_slist_foreach
   it writes the buffer to each connection in the slist
*/
void conn_write_func (gpointer data, gpointer user_data)
{
   debug ("new write func");
   
   
   // data is automatically set to the conn (they are the list elements)
   GConn *conn = (GConn *)data;
   gchar *buf;
   time_t tv = time (NULL);
   gint size;

   
   // we passed in the comBuf packet struct
   light_temp_event_t *ev = (light_temp_event_t *)user_data;
   
   // pack the data, the recvd packet is node id, light, temp 
   size = gnet_pack_strdup ("!iiliii", &buf,
			    sizeof (light_temp_event_t), NET_EVENT_LIGHT_TEMP, tv,
			    ev->node_id, ev->light, ev->temp);
   if (size == -1) {
      printf ("Error packing during write\n");
      return;
   }

   // write out the packed buf 
   debug ("attempting to write %d buf %p conn %p", size, buf, conn);
   gnet_conn_write (conn, buf, size);
   debug ("written");
   g_free (buf);
   debug ("freed");
}

/* reads from stdin and writes to each device in the conn_list */
static gboolean stdin_func (GIOChannel *chan, GIOCondition cond, gpointer data)
{
   guint bytes_read;
   
   if (cond & G_IO_ERR) {
      printf ("Socket error on stdin??\n");
      exit (-1);
   }

   if (cond & G_IO_IN) {
      GIOError error;
      // TODO: something besides a buffer of 4096? 
      gchar *buf = g_malloc (BUF_SIZE);

      // read data into buffer, check for error 
      error = gnet_io_channel_readline (chan, buf, BUF_SIZE, &bytes_read);
      if (error != G_IO_ERROR_NONE) {
	 printf ("Read error [%d]\n", error);
	 exit (-1);
      } else if (bytes_read == 0) {
	 if (lines_pending == 0)
	    exit (0);
	 else
	    read_eof = TRUE;

	 g_free (buf);
	 return FALSE;
      }
      // no error? read it in 
      else {
	 /* write to all connections
	    TODO: implement some kind of connection table? */
	 printf ("Would have sent something to the clients\n");

	 
	 /* iterate through the list with our write callback */
	 /*
	   if (conn_list != NULL) {
	   g_mutex_lock (conn_mutex);
	   g_slist_foreach (conn_list, conn_write_func, &event);
	   g_mutex_unlock (conn_mutex);
	    
	   } else {
	   printf ("No clients connected yet\n");
	   }
	 */
	 //g_free (buf);
	 
	 lines_pending++;
      }
   }
   
   return TRUE;
}

/* this is the callback which occurs when a client first connects */
void server_func (GServer *server, GConn *conn, gpointer data)
{
   if (conn) {
      gnet_conn_set_callback (conn, client_func, NULL);
      debug ("Got a new connection from %s:%d",
	     gnet_inetaddr_get_canonical_name (conn->inetaddr),
	     gnet_inetaddr_get_port (conn->inetaddr));
      
      // add the client to our list of all client connections
      g_mutex_lock (conn_mutex);
      conn_list = g_slist_append (conn_list, conn);
      g_mutex_unlock (conn_mutex);
      
      gnet_conn_set_watch_error (conn, TRUE);
      gnet_conn_readn (conn, sizeof (net_event_header_t));
   } else {
      printf ("Error: conn was NULL?!\n");
      gnet_server_delete (server);
      exit (-1);
   }
}

typedef struct {
   gint node_id;
   gint event;
   gint arg;
} node_net_event_command_t;

/* this is the callback which occurs with some kind of client activity */
void client_func (GConn *conn, GConnEvent *event, gpointer data)
{
   static gint recv_state = STATE_IDLE;
   static net_event_header_t header;
   static light_temp_event_t ev;

   debug ("in client func");
   
   
   switch (event->type) {
      
      // done reading
   case GNET_CONN_READ: {

      //g_mutex_lock (conn_mutex);
      // printf ("Should not be reading...\n");
      
      if (recv_state == STATE_IDLE) {
	 int size;
	 // read in the header for the next go around
	 debug ("grabbing mutex");
	 
	 g_mutex_lock (conn_mutex);
	 
	 gnet_unpack ("!iil", event->buffer, event->length,
		      &header.size, &header.event_type, &header.time_seconds);
	 recv_state = STATE_GOT_SIZE;
	 size = header.size;
	 
	 printf ("got a header of type %d, len %d\n", header.event_type,
		 header.size);

	 g_mutex_unlock (conn_mutex);
	 gnet_conn_readn (conn, size);

      } else if (recv_state == STATE_GOT_SIZE) {
	 debug ("grabbing mutex");
	 g_mutex_lock (conn_mutex);

	 debug ("got normal packet");
	 // already got a header, see what to do with the packet
	 if (header.event_type == NET_EVENT_LIGHT_TEMP_RANDOM) {
	    debug ("it's a random light temp");
	    //g_mutex_lock (conn_mutex);
	    gnet_unpack ("!iii", event->buffer, event->length,
			 &ev.node_id, &ev.light, &ev.temp);
	    debug ("unpacked");
	    
	    if (conn_list != NULL) {
	       g_slist_foreach (conn_list, conn_write_func, &ev);
	    } else {
	       debug ("Not sending packet, no clients connected yet");
	    }
	    debug ("written");
	    //g_mutex_unlock (conn_mutex);
	 } else if (header.event_type == NET_EVENT_COMMAND) {
	    printf ("got a cmd packet: len is %d\n", event->length);

	    //g_mutex_lock (conn_mutex);
	    
	    comBuf send_pkt;
	    node_net_event_command_t command;

	    if (header.size == 12) {
	       gnet_unpack ("!iii", event->buffer, header.size,
			    &command.node_id, &command.event, &command.arg);
	    } else if (header.size == 8) {
	       gnet_unpack ("!ii", event->buffer, header.size,
			    &command.node_id, &command.event);
	    } else {
	       debug ("UNHANDLED HEADER SIZE");
	    }
	    
	    debug ("Sending a command packet: %d %d %d", command.event, header.size,
		   command.node_id);

	    //for(i=0;i<command.command_len);
	    node_net_event_t *net_event = (node_net_event_t *)send_pkt.data;
	    net_event->from = 0;
	    net_event->to = command.node_id;
	    net_event->event = command.event;
	    if (header.size == 12) {
	       send_pkt.data[6] = command.arg;
	       send_pkt.size = 7;
	    } else
	       send_pkt.size = 6;
	    
	    com_send (IFACE_SERIAL, &send_pkt);
	    debug ("sent command packet");
	    
	    //g_mutex_unlock (conn_mutex);

	 } else {
	    printf ("Received unknown event type %d\n", header.event_type);
	 }

	 recv_state = STATE_IDLE;
	 g_mutex_unlock (conn_mutex);

	 debug ("reading another header");
	 gnet_conn_readn (conn, sizeof (net_event_header_t));
	 debug ("done reading header");
      }
      //g_mutex_unlock (conn_mutex);
      
      break;
   }
      // done writing
   case GNET_CONN_WRITE: {
      debug ("wrote");
      break;
   }
   case GNET_CONN_CLOSE:
   case GNET_CONN_TIMEOUT:
   case GNET_CONN_ERROR: {
      // client has timed-out / disconnected
      //g_mutex_lock (conn_mutex);
      printf ("Client %s:%d disconnected\n",
	      gnet_inetaddr_get_canonical_name (conn->inetaddr),
	      gnet_inetaddr_get_port (conn->inetaddr));
      
      conn_list = g_slist_remove (conn_list, conn);
      if (!g_slist_length (conn_list)) {
	 debug ("Resetting conn_list to null");
	 conn_list = NULL;
      }
      gnet_conn_delete (conn);
      //g_mutex_unlock (conn_mutex);      

      recv_state = STATE_IDLE;
      break;
   }
   default:
      debug ("default case??");
      g_assert_not_reached ();
   }
}
