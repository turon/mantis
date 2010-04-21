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

/*   file:  analyze.c  
 * Written:  Robert Havlik and Brianna Bethel
 *   date:  05/09/04
 */
//This program preforms statisticts and analysis of 
//collected radio data in file form

#include "config.h"
#include "msched.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#define UART1 1
#define UART0 0


void getStats(void);

void start(void)
{
  mos_thread_new(getStats,10240,PRIORITY_NORMAL);
}

void getStats()
{

  char FileName[24];
  char environment[16];
  char datain[16];
  FILE *statsfile;
  float distance;
  int sendpower;
  int recvpower;
  int numsamples;
  int frequency;
  int refnum = 33;
  int count =0;
  int droppedPackets=0;
  int i, j, k;
  
  statsfile= fopen("../../../../../src/apps/AutoRF/1/DistanceStats.txt", "w");
  
  fprintf(statsfile, "Setting Distance NumSamples Frequency SendPower RecvPower DroppedPackets\n");
   

  FILE *masterfile;
  masterfile= fopen("../../../../../src/apps/AutoRF/1/Master1.txt", "r");

while(fgets(FileName, 24, masterfile)!=NULL)
 {
     char location1[80]="../../../../../src/apps/AutoRF/1/";
     fgets(FileName, 17, masterfile);
     strcat(location1, FileName);
     
     FILE *currentData;
     currentData = fopen(location1, "r");
     printf("Opening file: %s\n", location1);
          
     if(currentData == 0)
     {
	 printf("Error opening File: %s", location1);
	 //printf("Error opening File: %s", FileName);
	 //IMPLEMENT FUNCTIONALITY HERE(PROBABLY A FUNCTION)
	 //TO DELETE AN ENTRY FROM THE MASTER THAT IS NOt PRESENT
	 exit(0);
     }
    
    
     fgets(environment, 11, currentData);
     fgets(datain,16,currentData);//nothing - just skips a line
     //fgets(datain,16,currentData);//nothing - just skips a line
     
     for(i=0; i<5; i++)
     {
	 fgets(datain,16, currentData);
	 if(i==0)
	 {
	     distance = atoi(datain); 
	 }
	 else if(i==1)
	 {
	     numsamples = atoi(datain);
	 }
	 else if(i==2)
	 {
	     frequency = atoi(datain);
	 }
	 else if(i==3)
	 {
		sendpower = atoi(datain);
	 }
	 else if(i==4)
	 {
	      recvpower = atoi(datain);
	 }
     }

     int data[numsamples+100];
     printf("%d\n", numsamples);
     
     //fill data array with numbers that will not be received
     for(i=0; i<numsamples;i++)//numsamples; i++)
     {
	 data[i]=999;
     }
 
     fgets(datain,16, currentData);  //nothing line for code?

     k=0;
     while(fgets(datain, 16, currentData)!=NULL)
     {
	 data[k]=atoi(datain);
	 //printf("%d\n", data[k]);
	 k=k+1;
     } 

     fclose(currentData);

      //figure out dropped packets
     count=0;

     //for(i=1000;i<2001;i++)
     // printf("%d\n", data[i]);

     while(count<numsamples)
     {
	 //printf(" Count is: %d",count);
	 //printf(" Num of Samples is :%d",numsamples);	      
	 
	 if(data[count]==999)
	 {
	     printf(" Count of 999 is: %d\n",count);
	     printf(" Num of Samples at 999 is : %d \n",numsamples);	      
	     droppedPackets = numsamples-count;
	     printf("%d\n",droppedPackets);
	     count=numsamples+10;
	    }
	 count = count +1;
     }
     
      //place stats in the file
      fprintf(statsfile, "%s %f %d %d %d %d %d\n", environment, distance, numsamples, frequency, sendpower, recvpower, droppedPackets);
      printf("I finished this...right?\n");
      
 }

 printf("Did I ever leave the loop\n");
 
 fclose(masterfile);
 printf("I closed the master file\n");
 fclose(statsfile);
 printf("done\n");
 return;
}


//stats function
void getStats2()
{

  char FileName[24];
  char location1[80]="1/";
  char environment[16];
  char datain[16];
  FILE *statsfile;
  float distance;
  int sendpower;
  int recvpower;
  int numsamples;
  int frequency;
  int refnum = 33;
  int count =0;
  int droppedPackets;
  int i, j, k;
  
  statsfile= fopen("../../../../../src/apps/AutoRF/1/DistanceStats.txt", "w");
  
  fprintf(statsfile, "Setting Distance NumSamples Frequency SendPower RecvPower DroppedPackets");
   

  FILE *masterfile;
  masterfile= fopen("../../../../../src/apps/AutoRF/1/Master1.txt", "r");

  while(fgets(FileName, 24, masterfile)!=NULL)
    {
      strcat(location1, FileName);

      FILE *currentData;
      //currentData = fopen(location1, "r");
      currentData = fopen("../../../../../src/apps/AutoRF/1/042104145926.txt", "r");      
      
      if(currentData == 0)
      {
	  printf("Error opening File: %s", location1);
          //printf("Error opening File: %s", FileName);
	  //IMPLEMENT FUNCTIONALITY HERE(PROBABLY A FUNCTION)
	  //TO DELETE AN ENTRY FROM THE MASTER THAT IS NOt PRESENT
	  exit(0);
	}
      
      fgets(environment,16, currentData);

      for(i=0; i<5; i++)
	{
	  fgets(datain,16, currentData);
	  if(i==0)
	    {
	      distance = atoi(datain); 
	    }
	  else if(i==1)
	    {
	      numsamples = atoi(datain);
	    }
	  else if(i==2)
	    {
	      frequency = atoi(datain);
	    }
	  else if(i==3)
	    {
	      sendpower = atoi(datain);
	    }
	  else if(i==4)
	    {
	      recvpower = atoi(datain);
	    }
	}
    
      int data[numsamples];
      int reference[numsamples];

      //fill data array with numbers that will not be received
      for(i=0; i<numsamples; i++)
	{
	  data[i]=999;
	}
      
      while(fgets(datain, 16, currentData)!=NULL)
	{
	  data[k]=atoi(datain);
	  k=k+1;
	  printf("%d\n", data[j]);
	  //this is for the frequency test
	  /*   if(datain == frequencyString) */
	  /* 	    { */
	  /* 	      fgets(datain, 16, currentData); */
	  /* 	      frequency = atoi(datain); */
	  /* 	    } */
	} 

      fclose(currentData);

      //create reference array

      for(i=0; i<numsamples; i++)
	{
	  reference[i]=refnum;
	  refnum = refnum+11;

	  if(refnum >= 255)
	    {
	      refnum= refnum-255;
	    }
	}

      //figure out dropped packets

      while(count<numsamples)
	{
	  if(data[count]==999)
	    {
	      droppedPackets = numsamples -count;
	      count=numsamples;
	    }
	  count = count +1;
	}

      //bit error finding, currently unimplemented

      /*     for(int i=0; i<numsamples; i++) */
      /*       { */
      /* 	if(data[i] != reference[j]) */
      /* 	  { */
	    
      /* 	    if(data[i+1]==reference[j+1]) */
      /* 	      { */
      /* 		bit_errors= bit_errors+1; */
      /* 	      } */
      /* 	  } */
      /* 	j=j+1; */
      /*       } */

      //place stats in the file
      fprintf(statsfile, "%s %f %d %d %d %d %d", environment, distance, numsamples, frequency, sendpower, recvpower, droppedPackets);
    }

  fclose(masterfile);
  fclose(statsfile);
  return;
}
