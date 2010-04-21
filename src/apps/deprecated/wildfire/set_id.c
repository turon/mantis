/*
  This file is part of MANTIS OS, Operating System for Nymph.
  See http://mantis.cs.colorado.edu/

  Copyright (C) 2003 University of Colorado, Boulder

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  (See http://www.gnu.org/copyleft/gpl.html)
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307,
  USA, or send email to mantis-users@cs.colorado.edu.
*/

/**************************************************************************/
/* File:    blink.c                                                       */
/* Author      Jeff Rose   :  rosejn@colorado.edu                         */
/*   Date: 12/11/03                                                       */
/* Edited by   Adam Torgerson: adam.torgerson@colorado.edu                */
/*   Date: 12/11/03                                                       */
/* Edited by   Charles Gruenwald  :  gruenwal@colorado.edu                */
/*   Date: 04/14/0/04                                                     */
/*                                                                        */
/* This is a simple demo app that flashes leds.                           */
/**************************************************************************/



#include <inttypes.h>
#include "mos.h"
#include "led.h"
#include "dev.h"
#include "msched.h"
#include "printf.h"




void set_id()
{
  mos_node_id_set(10);
 printf ("node id is now %d\n", mos_node_id_get());
}


/* The start function is automatically called by the operating system,
   for single-threaded applications, treat start() like main(). for multi-
   threaded applications, start each thread in the start() function.*/
void start(void)
{

   mos_thread_new (set_id, 128, PRIORITY_NORMAL);
}
