/* Echo client (GConn TCP)
 * Copyright (C) 2000-2003  David Helder
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glib.h>
#include <gnet.h>
#include "time.h"
#include "node_net_event.h"

/* gnet stuff */
static gboolean ob_in_iofunc (GIOChannel* iochannel, GIOCondition condition, 
			      gpointer data);

static void ob_conn_func (GConn* conn, GConnEvent* event, gpointer user_data);

/* file i/o routines */
static void open_file(char *filename);
static void log_data(char *buffer, int length);
static void close_file();
static GIOChannel *outfile;

static int lines_pending = 0;
static gboolean read_eof = FALSE;

#define MANTIS_PORT 4200
#define DEFAULT_HOSTNAME  "localhost"

#define STATE_IDLE 0
#define STATE_GOT_SIZE 1

int
main(int argc, char** argv)
{
   gchar hostname [] = DEFAULT_HOSTNAME;
   gint port;
   GMainLoop* main_loop;
   GConn* conn;
   GIOChannel* in;
   char *filename;

   gnet_init ();

   /* Parse args */
   if (argc != 2)
   {
      g_print ("usage: %s <filename>\n", argv[0]);
      exit(EXIT_FAILURE);
   }
   //hostname = argv[1];
   filename = argv[1];
   port = MANTIS_PORT;


   printf("Mantis Logger.\n");
   /* test FILE i/o */
   open_file(filename);

   /* Create the main loop */
   main_loop = g_main_new(FALSE);

   //log_data("log file\n",sizeof("logfile\n"));
   /* Create connection object */
   printf("Connecting to host: %s\n",hostname);
   conn = gnet_conn_new (hostname, port, ob_conn_func, NULL);
   g_assert (conn);

   /* Connect */
   gnet_conn_connect (conn);
   gnet_conn_set_watch_error (conn, TRUE);
   gnet_conn_timeout (conn, 30000);  /* 30 second timeout */

   printf("Connected.\n");
   /* Read from stdin */
   in = g_io_channel_unix_new (fileno(stdin));
   g_io_add_watch(in, G_IO_IN | G_IO_ERR | G_IO_HUP | G_IO_NVAL, 
		  ob_in_iofunc, conn);

   /* Start the main loop */
   g_main_run (main_loop);
   close_file();

   exit (EXIT_SUCCESS);
   return 0;
}


/* Read line from stdin asynchronously */
static gboolean
ob_in_iofunc (GIOChannel* iochannel, GIOCondition condition, 
	      gpointer data)
{
   GConn* conn = (GConn*) data;

   /* Check for socket error */
   if (condition & G_IO_ERR)
   {
      fprintf (stderr, "Error: Socket error\n");
      goto error;
   }

   /* Check for data to be read (or if the stdin was closed (?)) */
   if (condition & G_IO_IN)
   {
      GIOError error;
      gchar buffer[1024];
      guint bytes_read;

      /* Read the data into our buffer */
      error = gnet_io_channel_readline (iochannel, buffer, 
                                        sizeof(buffer), &bytes_read);

      /* Check for stdin error */
      if (error != G_IO_ERROR_NONE)
      {
         fprintf (stderr, "Read error (%d)\n", error);
         goto error;
      }

      /* Check for EOF */
      else if (bytes_read == 0)
      {
         if (lines_pending == 0)
            exit (EXIT_SUCCESS);
         else
            read_eof = TRUE;
         return FALSE;
      }

      /* Otherwise, we read something */
      else
      {
         gnet_conn_write (conn, buffer, bytes_read);
         lines_pending++;
      }
   }

   return TRUE;

error:
   exit (EXIT_FAILURE);
   return FALSE;
}


/* Handle GConn events */
static void
ob_conn_func (GConn* conn, GConnEvent* event, gpointer user_data)
{

   static gint recv_state = STATE_IDLE;
   
   switch (event->type)
   {
   case GNET_CONN_CONNECT:
   {
      gnet_conn_timeout (conn, 0);	/* reset timeout */
      gnet_conn_readn (conn, sizeof(net_event_header_t));
      break;
   }

   case GNET_CONN_READ:
   {
      light_temp_event_t ev;
      net_event_header_t hdr;

      /* new event */
      if(recv_state == STATE_IDLE){
      gnet_unpack ("!iil", event->buffer, event->length,
		   &hdr.size, &hdr.event_type, &hdr.time_seconds);

      printf ("Client: event type: %d time: %d",
	      hdr.event_type, hdr.time_seconds);
      
      recv_state = STATE_GOT_SIZE;
      gnet_conn_readn (conn, hdr.size);
      }
      else if(recv_state == STATE_GOT_SIZE)
      {
	 /* Unpack the event data into our structure. */
	 gnet_unpack("!iii", event->buffer, sizeof(light_temp_event_t),
		     &ev.node_id, &ev.light, &ev.temp);
	 
	 printf (" light: %d temp: %d from id: %d\n", ev.light, ev.temp, ev.node_id);
	 recv_state = STATE_IDLE;
      }
      
      event->buffer[event->length - 1] = '\n';
      //printf ("LOG:: [%s]", event->buffer);
      log_data(event->buffer, event->length);

      gnet_conn_readn (conn, sizeof(net_event_header_t));

      break;
   }

   case GNET_CONN_WRITE:
   {
      /* do nothing */
      break;
   }

   case GNET_CONN_CLOSE:
   {
      gnet_conn_delete (conn);
      exit (EXIT_SUCCESS);
      break;
   }

   case GNET_CONN_TIMEOUT:
   {
      gnet_conn_delete (conn);
      fprintf (stderr, "Connection timeout\n");
      exit (EXIT_FAILURE);
      break;
   }

   case GNET_CONN_ERROR:
   {
      gnet_conn_delete (conn);
      fprintf (stderr, "Connection failure\n");
      exit (EXIT_FAILURE);
      break;
   }

   default:
      g_assert_not_reached ();
   }
}
static void open_file(char *filename)
{
   gchar *fname;
   fname = strdup(filename);
   outfile = g_io_channel_new_file(fname,"a+",NULL);
  if(outfile == NULL) {
      g_error("Error opening file for output: [%s]",filename);
      return;
    }
   
}
static void log_data(char *buffer, int length)
{
     time_t rawtime;
  struct tm * timeinfo;
  char * string_ptr;
  
  time ( &rawtime );
  timeinfo = localtime ( &rawtime );
  /*
  YYYY-MM-DD HH:MM:SS Sensor1 Sensor2 Sensor3
  
  send_num_to_nymph(timeinfo->tm_year + 1900,10);
    send_to_nymph("\0");
  send_num_to_nymph(timeinfo->tm_mon+1,10);
    send_to_nymph("\0");
  send_num_to_nymph(timeinfo->tm_mday,10);
    send_to_nymph("\0");
  send_num_to_nymph(timeinfo->tm_hour+7,10);
    send_to_nymph("\0");
    send_num_to_nymph(timeinfo->tm_min,10);
  send_to_nymph("\0");
  time ( &rawtime ); //resync time to get seconds
  timeinfo = localtime ( &rawtime );
  send_num_to_nymph(timeinfo->tm_sec,10);
  send_to_nymph("\0");
  */
   gchar *message;
   GError *error_ret=NULL;
   int written_bytes=0;
   /* create the log text */
   message = g_strdup_printf("%d-%d-%d %d:%d:%d id:%d l:%d t:%d\n", timeinfo->tm_year + 1900,
			     timeinfo->tm_mon+1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min,
			     timeinfo->tm_sec, buffer[1],buffer[2],buffer[3]);
   
  g_io_channel_write_chars(outfile,
                           message,
                           -1,
                           &written_bytes,
                           NULL);
  g_io_channel_flush(outfile, &error_ret);
  /* clean up */
  g_free(message);
}
static void close_file()
{
   /* close file */
  g_io_channel_shutdown(outfile,TRUE,NULL);
}






