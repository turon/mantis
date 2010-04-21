//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include <ctype.h>

#define SREC_MAX_LEN   78      // S-Records can be at most 78 bytes long
#define CODE_MAX_SIZE  131072  // Size of flash in bytes (128k)

static char *stripwhite (char *line)
{
   char *p = line;
   while (isspace (*p)) {
      p++;
   }
   return p;
}

/* Convert hex to byte. */
uint8_t htob(const uint8_t* p)
{
   uint8_t val = 0;
   if ((*p >= '0') && (*p <= '9'))
      val += *p-'0';
   else
      val += *p-'A'+10;
   
   val <<= 4; p++;    
   if ((*p >= '0') && (*p <= '9'))
      val += *p-'0';
   else
      val += *p-'A'+10;
   return val;
}

int srec_read(char *filename, char *image)
{
   int remaining, len;
   uint8_t srec[SREC_MAX_LEN];
   uint8_t *p;
   uint8_t rec_len, data_len;
   uint16_t addr;
   int *ptr;
   int i;
   FILE *file = fopen(filename, "r");

   len = 0;
   if(file == NULL)
   {
      printf("ERROR--> Could not open file: '%s'\n", filename);
      return -1;
   }
   
   fseek(file, 0, SEEK_END);
   remaining = ftell(file);
   fseek(file, 0, SEEK_SET);
   

   /* Init the whole image to 0xff, which is equal to erased flash.
      I do this with an int* just for speed on a 32-bit machine. */
   ptr = (int *)image;
   for(i = 0; i < CODE_MAX_SIZE; i+=4)
      *ptr++ = 0xffffffff;
   
   /* Now run through and read the srecord data into the image. */
   while(remaining > 0)
   {
      fgets(srec, 78, file); // Read a line (1 whole s-record)
      rec_len = strlen(srec);
      remaining -= rec_len;
      
      /* Make sure it's all upper case. */
      for(i = 0; i < rec_len; i++)
	 srec[i] = toupper(srec[i]);

      /* Verify this is a valid s-record. */
      if(srec[0] != 'S')
      {
	 printf("Invalid S-Record format!\n");
	 fclose(file);
	 return -1;
      }
      
      /* For now we just deal with the data records themselves. */
      if(srec[1] == '1')
      {
	 /* Get the length of the data in the current record. */
	 data_len = htob(srec+2);
	 data_len -= 3;    // Subtract for the address & checksum
	 len += data_len;  // Add the number of data bytes to len
	 data_len *= 2;    // We have two chars per byte
	 
	 /* Now get the address for this record. */
	 addr = (uint16_t)htob(srec+4) << 8;
	 addr |= (uint16_t)htob(srec+6);
	 
         /* Run through the record and copy the data into the image. */
	 p = srec+8;
	 for(i = 0; i < data_len; i+=2)
	    image[addr++] = htob(p+i);	 
      }
   }
   return len;
}
int main(int argc, char **argv)
{
   char *name, *s;
   char image[CODE_MAX_SIZE];
   int val, ret;
   int i;

   if(argc == 2)
      name=argv[1];
   else{
      name = (char *)malloc(128 * sizeof(char));
      fprintf(stderr,"Enter name of S-Record file: ");
      ret = scanf("%s", name);
   }
   s = stripwhite(name); // Strip the whitespace so fopen works
   
   // Read the srecord into memory
   val = srec_read(s, image);
   
   if(val == -1) {
      fprintf(stderr,"Did not load file...\n");
      return -1;
   }
   fprintf(stderr, "screc read, image is %d bytes!\n",val);
   for(i=0;i<val;i++){
      printf("%c", image[i]); 
   }
}
