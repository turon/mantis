//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "mos.h"
#include "msched.h"
#include <string.h>

#ifndef PLATFORM_LINUX
#include "command_daemon.h"
#else
#include <stdlib.h>
#endif

#include "printf.h"

#include "fec.h"

#define ENCODE_TEXT "Nervously I loaded the twin ducks aboard the revolving platform."

char msg[sizeof (ENCODE_TEXT)];
char text [] = ENCODE_TEXT;
uint8_t parity_bytes[MAXIMUM_IMPLEMENTED_PARITY_BYTES];

//unsigned char codeword[256];

static void process_decode (uint8_t return_val)
{
   if (return_val != FEC_NO_ERRORS) {
      printf ("Message had errors, ");
      if (return_val == FEC_UNCORRECTABLE_ERRORS)
	 printf ("not correctable\n");
      else
	 printf ("correctable\n");
   }
}

void start (void)
{
   fec_init (8);
   strcpy (msg, text);
   fec_encode ((uint8_t *)msg, sizeof (text), parity_bytes);
   msg[12] = 169;
   msg[1] = 170;
   msg[24] = 171;
   parity_bytes[2] = 171;
   printf ("encoded data is \"%s\"\n", msg);
   //strcpy (&msg[sizeof (text)], parity_bytes);
   process_decode (fec_decode ((uint8_t *)msg, sizeof (text), parity_bytes));
   printf ("corrected: \"%s\"\n", msg);

   fec_init (4);
   fec_encode ((uint8_t *)msg, sizeof (text), parity_bytes);
   msg[23] = 1;
   msg[55] = 111;
   printf ("encoded data is \"%s\"\n", msg);
   //strcpy (&msg[sizeof (text)], parity_bytes);
   process_decode (fec_decode ((uint8_t *)msg, sizeof (text), parity_bytes));
   printf ("corrected: \"%s\"\n", msg);

   fec_init (2);
   fec_encode ((uint8_t *)msg, sizeof (text), parity_bytes);
   msg[55] = 111;
   printf ("encoded data is \"%s\"\n", msg);
   //strcpy (&msg[sizeof(text)], parity_bytes);
   process_decode (fec_decode ((uint8_t *)msg, sizeof (text), parity_bytes));
   printf ("corrected: \"%s\"\n", msg);
#ifndef PLATFORM_LINUX
   mos_thread_new (mos_command_daemon, MOS_COMMANDER_STACK_SIZE,
		   PRIORITY_NORMAL);
#else
   exit (0);
#endif
}
