//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "mos.h"

#ifdef PLATFORM_LINUX
#include <unistd.h>
#include <stdlib.h>
#endif

#include "printf.h"
#include "msched.h"
#include "arith-code.h"
#include "clock.h"

uint8_t quit_flag = 0;

#define BUF_SIZE 130
static uint8_t in_buf[BUF_SIZE], out_buf[BUF_SIZE];

void compress_thread (void)
{
   int16_t i;
   uint32_t *time_elapsed;

   printf ("starting compress %x %x\n", in_buf, out_buf);
   
   for (i = 0; i < BUF_SIZE - 2; i++) {
      in_buf[i] = i % 8;
   }
   //in_buf[i - 1] = EOF;

   for (i = 0; i < BUF_SIZE; i++) {
      printf ("i[%d]: i: %d o: %d\n", i, in_buf[i], out_buf[i]);
   }
   
   //real_timer_clear();
   arith_code_encode (in_buf, sizeof (in_buf), out_buf, sizeof (out_buf));
   //time_elapsed = real_timer_get_ticks();

#ifdef ARCH_AVR
   printf ("done compressing, time: %l\n", *time_elapsed);
#else
   printf ("done compressing, time: %uld\n", *time_elapsed);
#endif
   for (i = 0; i < BUF_SIZE; i++) {
      printf ("i[%d]: i: %d o: %d\n", i, in_buf[i], out_buf[i]);
   }

   quit_flag = 1;
}

void decompress_thread (void)
{
   int16_t i;
   uint32_t *time_elapsed;
   printf ("starting decompress\n");

   for (i = 0; i < BUF_SIZE; i++) {
      in_buf[i] = 16;
   }

   for (i = 0; i < BUF_SIZE; i++) {
      printf ("i[%d]: i: %d o: %d\n", i, in_buf[i], out_buf[i]);
   }
   //real_timer_clear();
   arith_code_decode (out_buf, sizeof (out_buf), in_buf, sizeof (in_buf));
   //time_elapsed = real_timer_get_ticks();
#ifdef ARCH_AVR
   printf ("done decompressing, time: %l\n", *time_elapsed);
#else
   printf ("done decompressing, time: %uld\n", *time_elapsed);
#endif
   for (i = 0; i < BUF_SIZE; i++) {
      printf ("i[%d]: i: %d o: %d\n", i, in_buf[i], out_buf[i]);
   }

   quit_flag = 1;
}

void start (void)
{
   //real_timer_init();
   //mos_thread_new (compress_thread, 128, PRIORITY_NORMAL);
   //mos_thread_new (decompress_thread, 128, PRIORITY_NORMAL);
   compress_thread ();
   decompress_thread ();

#ifndef ARCH_AVR
   exit (0);
#endif

   while (!quit_flag) {
#ifndef ARCH_AVR
      usleep (1000);
#endif
   }

   //fclose (input_file);
   //fclose (output_file);

}
