//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**
 * File:     fft_test.c      
 * Author:   Charles Gruenwald  :  gruenwal@colorado.edu
 * Date:     05-27-2004
 *
 * Description: This app is meant to test Alfredo del Rio's fft code...
 *
 */
#include "mos.h"
#include "msched.h"
void main_thread(void);
void fft_int(void);     
void start(void)
{
   mos_thread_new(main_thread, 128, PRIORITY_NORMAL);
}

/*

FFT Code Written in C Language for the 8052 Microcontroller
Written by Alfredo del Rio, University of Vigo, Spain
November, 13th, 2001

Note: This SW has been tested, and the Author thinks that it works well.
However the Author does not guarantee that the code is free of bugs.

This program computes the FFT from a set of 64 samples.
Each sample is a signed integer and is expected to be in the range 
-512 to 511, using two's complement notation. Samples can be obtained
using a 10-bit ADC, and substracting 512 (ADCs usually provide values
in binary-offset format). 
  
With a 12 MHz crystal it takes about 0.3 seconds to perform the fft, 
including the additional computation needed to find the amplitudes.

If samples exceed the range (-512, 511), an automatic scaling might be 
performed (this program can do the scaling when necessary, but the 
execution time increases). See 'NOTE 1' in the code list.
  
The array amplitude[n] gives the absolute amplitudes from f0 (DC) to f31,
but note that values are multiplied by 32. This improves resolution.

The array harmonic[n] gives the amplitudes of harmonics from f1 (n=1)
to f31 (f31), all relative to f1 in parts per thousand. Therefore
harmonic[1] shold always be 1000. Note that harmonic[0] is not computed.
  
In my application, I get 64 voltage samples per power-line period, using
a 10-bit ADC. The analog interface is designed so that 1 LSB is 1 volt.
Since the array amplitude[] gives 32x values, amplitude[1]= 4978 indicates
that the main frequency component (for example 60Hz) is about 156 Vpeak, 
or 110 Vrms. Additional math can be used to scale the amplitudes as needed.
For example amplitude[n] * 4 / 181 gives the result in tenths of volts rms.

Since the fft is computed using integer notation (not floating point), it
introduces rounding errors. Errors of about 0.1 percent should be expected.
  
Some special code has been included in order to improve speed. For example,
the function longroot() evaluates the square root of an unsigned long 
using a relatively fast algorithm, based on succesive aproximations.
*/

#include "stdlib.h"
#include <inttypes.h>
#ifndef PLATFORM_MICROBLAZE
#include "realtime.h"
#endif
#include "printf.h"
#define uchar unsigned char
#define ulong  unsigned long

#define limit 16383   /* when this limit is reached, scaling is invoked */

/* this table is used for the initial sample placing */
const unsigned char move[64]= {
   0x00,  0x20,  0x10,  0x30,  0x08,  0x28,  0x18,  0x38,
   0x04,  0x24,  0x14,  0x34,  0x0C,  0x2C,  0x1C,  0x3C,
   0x02,  0x22,  0x12,  0x32,  0x0A,  0x2A,  0x1A,  0x3A,
   0x06,  0x26,  0x16,  0x36,  0x0E,  0x2E,  0x1E,  0x3E,
   0x01,  0x21,  0x11,  0x31,  0x09,  0x29,  0x19,  0x39,
   0x05,  0x25,  0x15,  0x35,  0x0D,  0x2D,  0x1D,  0x3D,
   0x03,  0x23,  0x13,  0x33,  0x0B,  0x2B,  0x1B,  0x3B,
   0x07,  0x27,  0x17,  0x37,  0x0F,  0x2F,  0x1F,  0x3F
};

/* look-up table for fast sine function (maximum is 0x4000 = 16384) */
const int16_t sine[64]= {
   0x0000,  0x0646,  0x0C7C,  0x1294,  0x187E,  0x1E2B,  0x238E,  0x289A,
   0x2D41,  0x3179,  0x3537,  0x3871,  0x3B21,  0x3D3F,  0x3EC5,  0x3FB1,
   0x4000,  0x3FB1,  0x3EC5,  0x3D3F,  0x3B21,  0x3871,  0x3537,  0x3179,
   0x2D41,  0x289A,  0x238E,  0x1E2B,  0x187E,  0x1294,  0x0C7C,  0x0646,
   0x0000,  0xF9BA,  0xF384,  0xED6C,  0xE782,  0xE1D5,  0xDC72,  0xD766,
   0xD2BF,  0xCE87,  0xCAC9,  0xC78F,  0xC4DF,  0xC2C1,  0xC13B,  0xC04F,
   0xC000,  0xC04F,  0xC13B,  0xC2C1,  0xC4DF,  0xC78F,  0xCAC9,  0xCE87,
   0xD2BF,  0xD766,  0xDC72,  0xE1D5,  0xE782,  0xED6C,  0xF384,  0xF9BA
};

/* idem for cosine */
const int16_t cosine[64]= {
   0x4000,  0x3FB1,  0x3EC5,  0x3D3F,  0x3B21,  0x3871,  0x3537,  0x3179,
   0x2D41,  0x289A,  0x238E,  0x1E2B,  0x187E,  0x1294,  0x0C7C,  0x0646,
   0x0000,  0xF9BA,  0xF384,  0xED6C,  0xE782,  0xE1D5,  0xDC72,  0xD766,
   0xD2BF,  0xCE87,  0xCAC9,  0xC78F,  0xC4DF,  0xC2C1,  0xC13B,  0xC04F,
   0xC000,  0xC04F,  0xC13B,  0xC2C1,  0xC4DF,  0xC78F,  0xCAC9,  0xCE87,
   0xD2BF,  0xD766,  0xDC72,  0xE1D5,  0xE782,  0xED6C,  0xF384,  0xF9BA,
   0x0000,  0x0646,  0x0C7C,  0x1294,  0x187E,  0x1E2B,  0x238E,  0x289A,
   0x2D41,  0x3179,  0x3537,  0x3871,  0x3B21,  0x3D3F,  0x3EC5,  0x3FB1
};

int16_t in[64];           /* samples array */
uint16_t amplitude[32];   /* absolute amplitude for every frequency */
ulong harmonic[32];    /* idem, but relative to f1, in parts per thousand */
long work[128];        /* work array */

uchar scale;
uchar i_r, i_i, ip_r, ip_i;
uchar pass, di, dip, dk, k, rot;
long sink, cosk;
long x0, x1, x2, x3, x4, x5;
long real, imag;

void fft_int(void);                                 /* fft */
ulong longroot(unsigned long radic);    /* fast square root */
long mult(long x, long y);         /* fast multiply and shift */




//main()
void main_thread(void)
{
   uint8_t i;
   uint32_t the_time;
/* You can place here some sample loading for test purposes ... */

/* The following 'for' loop loads the sample vector with: 
   component    amplitude (Vpeak)   phase
   f1           256                 0 (cosine)
   f2           128                 0 (cosine)
   f3            85 aprox.          delayed 90 degrees (because sine)
*/  
   for(i=0;i<64;i++){ 
      in[i]= cosine[i]/2 + cosine[(2*i) & 63] / 4 + sine[(3*i) & 63] / 6;
      if(in[i]<0) in[i]+= 16;
      in[i]= in[i] >> 5;
   }
/* ... but you probably will include some ADC control routines instead */
#ifndef PLATFORM_MICROBLAZE
   real_timer_init();
#endif

   fft_int();   /* includes initial placing, fft and amplitude computing */
#ifndef PLATFORM_MICROBLAZE
   the_time=*real_timer_get_ticks();
#endif
   printf("sizes: \n");
   printf("char:\t%d\n",sizeof(char));
   printf("int:\t%d\n",sizeof(int));
   printf("long:\t%d\n",sizeof(long));
     

   printf("band \t amp \n");
   for(i=0;i<32;i++)
      printf("%d\t%d\n",i,amplitude[i]);

   printf("\n Approx Time: %l\n",the_time);
   printf("done.\n");

#ifdef PLATFORM_LINUX
   exit(1);
#else
   while(1);
#endif
  
}

/* fft_int() uses the following global variables:
   int in[64]              samples array
   int work[128]           work array (Real+Imaginary)
   uint amplitude[32]      output amplitudes array
   uint harmonic[32]       harmonics in parts per thousand
   uchar scale             indicates how many scalings have been performed
   i_r, i_i, ip_r, ip_i;   indices for work[] (butterflies)
   uchar pass, di, dip,    control indices (butterflies)
   uchar dk, k, rot;       idem
   int sink, cosk;         temporary copies of sine and cosine
   int x0, x1, x2, x3,     temporaries
   int x4, x5;             idem
   int real, imag;         idem

   .. and the look-up tables:
   move[64]                initial placing table
   sine[64]                sine table
   cosine[64]              cosine table

   Should you prefer to place some of this variables as locals, do it!, but
   not in the case of arrays.
*/
void fft_int()
{
   uchar i,j;                 // loop control
   char done;                 // scaling control
   ulong f1;                  // amplitude for f1
   ulong adjust = 0;          // auxiliar variable used for rounding
   long workipr, workipi;     // temporary copies for some work[] elements

   // initial sample placing   
   for(i=0;i<64;i++) {
      j= move[i];
      work[j]= in[i];
      work[64+j]= 0;     // imaginary parts are set to 0
   }
  
   scale = 0;
   di = 2;
   dip = 1;
   dk = 32;

   for(pass=0;pass<6;pass++){
      k= 0;
      for(rot=1;rot<=dip;rot++,k+=dk){
	 sink= sine[k];
	 cosk= cosine[k];
	 for(i_r=rot-1;i_r<64;i_r+=di){
	    ip_r= i_r + dip;
	    i_i= i_r + 64;
	    ip_i= i_i + dip;
	    workipr= work[ip_r];
	    workipi= work[ip_i];
	    done= 0;
	    do{
	       x1= mult(workipr,cosk);
	       x2= mult(workipi,sink);
	       real= x1 + x2;

/* NOTE 1: The following disabled code lines (forced to be comments) 
   must be enabled if scaling is expected to be needed 
   (see text at the beginning of file) */

/*          if(abs(real)>limit){ scaling(); continue;} */
	       x1= mult(workipi,cosk);
	       x2= mult(workipr,sink);
	       imag= x1 - x2;
/*          if(abs(imag)>limit){ scaling(); continue;} */
	       x1= work[i_r];
	       x2= x1 + real;
/*          if(abs(x2)>limit){ scaling(); continue;} */
	       x3= work[i_i];
	       x4= x3 + imag;
/*          if(abs(x4)>limit){ scaling(); continue;} */
	       x5= x1 - real;                 
/*          if(abs(x5)>limit){ scaling(); continue;} */
	       x0= x3 - imag;
/*          if(abs(x0)>limit){ scaling(); continue;} */

	       work[ip_i]= x0;
	       work[ip_r]= x5;
	       work[i_r]= x2;
	       work[i_i]= x4;
	       done= 1;
	    }
	    while(!done);
	 }
      }

      dk= dk >> 1;
      dip= di;
      di= di << 1;
   }

   f1= 0;
   for(i=0;i<32;i++){
      x1= abs(work[i]);
      x2= abs(work[i+64]);

/* The following aproximations are used to speed up amplitude computation, 
   calling the longroot() function only when really needed. If time is not 
   critical, use only 'x3= longroot(...)' */
    
      if((x1<8) && (x2<8)) x3= 0;      /* negligible amplitudes (math noise) */
      else if(x1 < (x2 >> 5)) x3= x2;  /* aprox. (x^2+y^2)= x^2, if x>32*y */
      else if(x2 < (x1 >> 5)) x3= x1;  /* reciprocal */
      else x3= longroot((unsigned long) x1*x1 + (unsigned long) x2*x2);
    
      if(i==1){ f1= x3; adjust= x3 >> 1;}
      else if(f1 != 0) harmonic[i]= (adjust + x3 * 1000L) / f1;
    
      if(scale) x3= x3 << scale;
      amplitude[i]= x3;
   }
}

void scaling()          
{
   uchar i;
   ulong x;
  
   scale++;
   for(i=0;i<128;i++){
      x= work[i];
      if(x>0) x++;
      if(x>limit) x--;
      x= x >> 1;
      work[i]= x;
   }
}

/* maximum permitted value for radic 1,073,676,289 (whose root is 32767) */
/* it takes about 4.62 ms to get the square root */
ulong longroot(unsigned long radic)
{
   ulong result;
   ulong mask, masknot;
   unsigned long acu;

   radic= radic << 2;    /* multiply radic by 4 */
   result= 0;
   mask= 0x8000;
   do{
      masknot= ~mask;
      result |= mask;
      acu= result;
      acu*= result;
      if(acu > radic){
	 result &= masknot;
      }
      mask= mask >> 1;
   }
   while(!(mask & 1));

/* round, adding 1 and dividing by 2 */
   result++;
   result= result >> 1;
   return result;
}

/*  fast multiply and shift:  int= ((long) x * y) >> 14   */
long mult(long x, long y)
{
   long temp;
   uchar templo;
   ulong  temphi;
  
   temp= (long) x * (long)y;
   temphi= *((ulong *) (&temp));         /* bytes 3 and 2 of temp */
   templo= *((uchar *) (&temp) + 2);    /* byte 1 of temp */
   temphi= temphi << 2;
   if(templo & 0x80) temphi |= 2;
   if(templo & 0x40) temphi |= 1;
   return temphi;
}

/* end of fft code (November, 13th, 2001) */


