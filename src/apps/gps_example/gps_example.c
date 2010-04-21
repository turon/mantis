//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/** @file gps_example.c
 *  @author John Ledbetter
 *
 * This file demonstrates reading from the GPS Board,
 * parsing and displaying the data.  The GPS may require
 * up to 10 minutes to get an initial lock. Depending on
 * where you are testing (ie in a basement), you may not
 * ever get a lock.
 */

#include "mos.h"
#include "msched.h"
#include "led.h"
#include "clock.h"
#include "mica2-gps.h"
#include "dev.h"

// this structure represents a GGA (gps fix data) packet.
gps_gga_t gga;

void start(void)
{
   // get an exclusive lock on the GPS driver. this is
   // actually unnecessary, since no other threads are
   // present/trying to use the GPS, but it's good
   // practice.
   dev_open(DEV_MICA2_GPS);
   
   dev_mode(DEV_MICA2_GPS, DEV_MODE_ON);

   // tell the driver that we want to read a gps_gga_t
   // structure when we call dev_read().  The other
   // option is to read a string which may or may not
   // be GGA data (ie, might be some other packet type)
   dev_ioctl(DEV_MICA2_GPS, MICA2_GPS_READ_GGA);
   
   while(1)
   {
      // read the next available GGA packet into our structure.
      dev_read(DEV_MICA2_GPS, &gga, sizeof(gga));

      // only print gga packets which contain some information
      if (gga.satellite_count > 0)
      {
	 // print the data.  this function parses all the
	 // information for us, so that we don't have to.
	 mica2_gps_print_gga(&gga);
      }
      else
      {
	 printf("no satellites found; sleeping.\n");
	 // send the system to idle mode for 5 seconds.
	 mos_thread_sleep(5000);
      }
      
   }
   
   // if the while loop above were not infinite,
   // it would be good practice to turn off the device
   // and release our exclusive lock on it.
   dev_mode(DEV_MICA2_GPS, DEV_MODE_OFF);
   dev_close(DEV_MICA2_GPS);
}
