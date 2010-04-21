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

/*   file:  autorf.c  
 * Written:  Robert Havlik and Brianna Bethel
 *   date:  05/09/04
 */

//This program preforms statistics and analysis of 
//collected radio data in file form

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <inttypes.h>
#include "config.h"
#include "msched.h"
#include "led.h"
#include "uart.h"
#include "com.h"
#include "serial.h"
#define UART1 1
#define UART0 0


void tests(void);
void ApplySettings(int, int, int, int, int, int, int);
void base_receive(int, char FileName[24]);
void Distance_Test(void);
void Frequency_Test(void);
void getStats(void);
void Packet_Size_Test(void);

void packet_to_string(comBuf *packet, char *thestring);

void start(void)
{
#ifdef PLATFORM_LINUX
  mos_thread_new(tests, 10240, PRIORITY_NORMAL);
#endif
} 


#ifdef PLATFORM_LINUX

void tests()
{

  char FileName[24];
  char datain[16];
  char environment[16];
  char string1[16];
  char string2[16];
  char string3[16];
  
  int j=0, k=0, i=0;
  int numsamples;
  int referenceStart=33;
  int frequency;
  int choice;
  int power;
  int powerrecv;
  char tmpstrng[64];
  char *pointer2 = tmpstrng; 
  size_t eye;
  struct tm tim;
  time_t now;

  comBuf *packet;
  comBuf to_print;

  printf("\n\n");
  printf("Welcome to the MANTIS AutoRF Radio Test Suite\n");
  printf("\n");
  printf("1) Distance Performance Analysis\n");
  printf("2) Frequency Performance\n");
  //The following functions are still in development
  /*
  printf("3) Frequency Interference\n");
  printf("4) Packet Size Test\n");
  printf("5) RSSI Test\n\n"); 
  */
    
  printf("Enter number of test to run:\n");

  packet = com_recv(IFACE_TERMINAL);

  if(packet != NULL)
    {
      packet_to_string(packet, pointer2);
      choice = atoi(tmpstrng);
      com_free_buf(packet);
    }

  if(choice==1)
    {
      Distance_Test();
      tests();
    } 
  
  else if(choice==2)
    {
      Frequency_Test();
      tests();
    }
  else if(choice==4)
    {
      Packet_Size_Test();
      tests();
    }
  else
    return;
}

void packet_to_string(comBuf *packet, char *thestring)
{
  int counter =0;
  int packetsize;

  packetsize= packet->size;
 
  packet->size =0;
  while(counter<packetsize)
    {
      thestring[counter]= packet->data[packet->size++];
      counter = counter +1;
    }
  
  thestring[counter]= '\0';
}


void Distance_Test()
{
  float distances[100]={1};
  float temp=1;
  float adistance;
  double difference;
  int j=0, k=0;
  char datain[16];
  char environment[16];
  int numsamples;
  int referenceStart=33;
  int frequency;
  int choice=1;
  int power;
  int powerrecv;
  char presskey;
  size_t eye;
  struct tm tim;
  time_t now;
  int c;
  int testtmp;
  char tmpstrng[64];
  comBuf *packet; 
  char numstrng[12];
  int i=0;
  int distind =0;
  char *pointer = tmpstrng;  
  

  printf("Enter distances to test in meters (separated by spaces and end with a zero):\n");
  packet = com_recv(IFACE_TERMINAL);
  packet_to_string(packet, pointer);


  while(testtmp != 48)  //48 is zero in ascii  
    {  
      testtmp= tmpstrng[i];
      while(testtmp != 32)
	{
	  numstrng[k]=testtmp;
	  k = k+1;
	  i = i+1;
          testtmp = tmpstrng[i];
	}
      k=0;
      i = i+1;
      adistance= atoi(numstrng);
      distances[distind] =  adistance;
      distind = distind+1;
      testtmp = tmpstrng[i];
    }
  com_free_buf(packet);
   
  printf("Enter type of environment:\n");
  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, environment);
      com_free_buf(packet);
    }
  printf("Enter Number of samples at each distance:\n");
  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, pointer);
      numsamples=atoi(tmpstrng);
      com_free_buf(packet);
    }

  printf("Enter Fixed frequency for testing (index 1-30, type 0 for list):\n");
  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, pointer);
      frequency = atoi(tmpstrng);
      com_free_buf(packet);
    }
  printf("Enter Desired Power Setting (0-255) for SENDER:\n");
  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, pointer);
      power = atoi(tmpstrng);
      com_free_buf(packet);
    }  
  printf("Enter Desired Power Setting (0-255) for RECEIVER:\n");
  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, pointer);
      powerrecv = atoi(tmpstrng);
      com_free_buf(packet);
    }
  //TODO: An accept for these options
      
  ApplySettings(numsamples,frequency,power, powerrecv, choice, 0, 0);
  
    
  while(distances[k]>0 && k<100)
    {
       char DistancePath[75]="../../../../../src/apps/AutoRF/1/";
       char FileName[24]= {0}; 
      
      now = time(NULL);
      tim = *(localtime(&now));
      eye = strftime(FileName, 24, "%m%d%y%H%M%S",&tim); 
       
      strcat(FileName, ".txt");
       
      FILE *masterfile;
      masterfile = fopen("../../../../../src/apps/AutoRF/1/Master1","a");
      fprintf(masterfile, "%s\n", FileName);
      fclose(masterfile);
          
      strcat(DistancePath, FileName);
              
      FILE *current;
      current = fopen(DistancePath, "w");
      fprintf(current, "%s\n", environment);
      fprintf(current, "%f\n", distances[k]);
      fprintf(current, "%d\n", numsamples);
      fprintf(current, "%d\n", frequency); //TODO: implement a look up table to get real value in Hertz index corresponds to
      fprintf(current, "%d\n", power);
      fprintf(current, "%d\n", powerrecv);
      fclose(current);
       
      printf("Place units %f meters appart, press any number to continue (type something)\n\n", distances[k]);
     
  int temp = 0;

  while(temp == 0)
    {  
      packet= com_recv(IFACE_TERMINAL);
      if(packet != NULL)
	{
	  packet_to_string(packet, pointer);
	  temp = atoi(tmpstrng);
	  com_free_buf(packet);
	}
    }
      com_free_buf(packet); 
       
      printf("Be sure the receiving unit is connected to the serial port,\n"); 
      printf("then turn on the receiving unit first and then the sending unit.\n");
         
      base_receive(numsamples, DistancePath);
      k=k+1;
       
    }
  printf("You have completed Distance Analysis\n");

  return;
}

void Frequency_Test()
{
  char FrequencyPath[75]="../../../../../src/apps/AutoRF/2/";
  char environment[16];
  char FileName[24];
  int numsamples;
  int referenceStart=33;
  int frequency;
  int choice=2;
  int power;
  int powerrecv;
  float distance; 
  int k=0;
  size_t eye;
  struct tm tim;
  time_t now;
  int c;
  comBuf *packet;
  char tmpstrng[64];
  char *pointer = tmpstrng;

  printf("Enter the Distance Between the Nodes:\n");
  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, pointer);
      distance = atoi(tmpstrng);
      com_free_buf(packet);
    }
  printf("Enter type of environment:\n");
  
  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, environment);
      com_free_buf(packet);
    }
  printf("Enter Number of samples to take at each frequency:\n");

  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, pointer);
      numsamples = atoi(tmpstrng);
      com_free_buf(packet);
    }        
  printf("Enter Desired Power Setting (0-255) for SENDER:\n");
 
  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, pointer);
      power = atoi(tmpstrng);
      com_free_buf(packet);
    }
  printf("Enter Desired Power Setting (0-255) for RECEIVER:\n");
 
  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, pointer);
      powerrecv = atoi(tmpstrng);
      com_free_buf(packet);
    }
  //TODO: an accept for these options
      
  ApplySettings(numsamples,frequency,power, powerrecv, choice, 0, 0);
      
  now = time(NULL);
  tim = *(localtime(&now));
  eye = strftime(FileName, 24, "%m%d%y%H%M%S",&tim); 

  strcat(FileName, ".txt");

  FILE *masterfile;
  masterfile = fopen("../../../../../src/apps/AutoRF/2/Master2","a");

  fprintf(masterfile, "%s\n", FileName);
  fclose(masterfile);

  strcat(FrequencyPath, FileName);

  FILE *current;
  current = fopen(FrequencyPath, "w");
      
  frequency = 0; //put frequency to be zero in the header part=varrying

  fprintf(current, "%s\n", environment);
  fprintf(current, "%f\n", distance);
  fprintf(current, "%d\n", numsamples);
  fprintf(current, "%d\n", frequency); 
  fprintf(current, "%d\n", power);
  fprintf(current, "%d\n", powerrecv);
  fclose(current);
   
  printf("The settings are now applied.  In another shell, make the send and recv\n");
  printf("programs by typing make && mos_shell generator.srec and make && mos_shell receiver.srec \n\n");
  printf("Be sure the receiving unit is connected to the serial port,\n");
  printf(" then turn on the receiving unit first and then the sending unit.\n");
         
  base_receive(numsamples,FrequencyPath);
      
  return;
}


void Packet_Size_Test(void)
{
  char DistancePath[75]="../../../../../src/apps/AutoRF/4/";
  float distances[100]={0};
  float temp=1;
  double difference;
  int j=0, k=0;
  char FileName[24];
  char datain[16];
  char environment[16];
  int numsamples;
  int referenceStart=33;
  int frequency;
  int choice=1;
  int power;
  int powerrecv;
  char presskey;
  size_t eye;
  struct tm tim;
  time_t now;
  int c;
  int startpkt=8;
  int endpkt=16;
  int testtmp=1;
  int testtmpstr =1;
  int adistance;
  char numstrng[12];
  char tmpstrng[64];
  char *pointer = tmpstrng;
  comBuf *packet;
  int i=0;
  int distind=0;

  printf("Enter distances to test in meters (separated by spaces end with 0):\n");
  packet = com_recv(IFACE_TERMINAL);
  packet_to_string(packet, pointer);

  while(testtmp != 48)  //48 is zero in ascii  
    {  
      testtmp= tmpstrng[i];
      while(testtmp != 32)
	{
	  numstrng[k]=testtmp;
	  k = k+1;
	  i = i+1;
          testtmp = tmpstrng[i];
	}
      k=0;
      i = i+1;
      adistance= atoi(numstrng);
      distances[distind] =  adistance;
      distind = distind+1;
      testtmp = tmpstrng[i];
    }
  com_free_buf(packet);
      
  printf("Enter type of environment:\n");
  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, environment);
      com_free_buf(packet);
    }
  printf("Enter Number of samples at each distance:\n");
  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, pointer);
      numsamples= atoi(tmpstrng);
      com_free_buf(packet);
    }
  printf("Enter Fixed frequency for testing (index 1-30, type 0 for list):\n");
  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, pointer);
      frequency = atoi(tmpstrng);
      com_free_buf(packet);
    }
  printf("Enter Desired Power Setting (0-255) for SENDER:\n");
  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, pointer);
      power = atoi(tmpstrng);
      com_free_buf(packet);
    }
  printf("Enter Desired Power Setting (0-255) for RECEIVER:\n");
  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, pointer);
      powerrecv = atoi(tmpstrng);
      com_free_buf(packet);
    }
  printf("Enter Starting Packet Size:\n");
  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, pointer);
      startpkt = atoi(tmpstrng);
      com_free_buf(packet);
    }
  printf("Enter Max Packet Size:\n");
  packet= com_recv(IFACE_TERMINAL);
  if(packet != NULL)
    {
      packet_to_string(packet, pointer);
      endpkt= atoi(tmpstrng);
      com_free_buf(packet);
    }

  //TODO: An accept on these options
      
  ApplySettings(numsamples,frequency,power, powerrecv, choice, startpkt, endpkt);  
  
  while(distances[k]>0 && k<100)
    {

      now = time(NULL);
      tim = *(localtime(&now));
      eye = strftime(FileName, 24, "%m%d%y%H%M%S",&tim); 
       
      strcat(FileName, ".txt");
       
      FILE *masterfile;
      masterfile = fopen("../../../../../src/apps/AutoRF/4/Master4","a");
      fprintf(masterfile, "%s\n", FileName);
      fclose(masterfile);
          
      strcat(DistancePath, FileName);
              
      FILE *current;
      current = fopen(DistancePath, "w");
      fprintf(current, "%s\n", environment);
      fprintf(current, "%f\n", distances[k]);
      fprintf(current, "%d\n", numsamples);
      fprintf(current, "%d\n", frequency); 
      //do a lookup table or something to get the real value in Hz
      fprintf(current, "%d\n", power);
      fprintf(current, "%d\n", powerrecv);
      fclose(current);
       
      printf("Place units %f meters appart, press any key to continue\n\n", distances[k]);
       
      c = getchar();
       
      printf("Be sure the receiving unit is connected to the serial port,\n"); 
      printf("then turn on the receiving unit first and then the sending unit.\n");
         
      base_receive(numsamples, DistancePath);
       
      k=k+1;
       
    }
  printf("You have completed Packet Size Testing\n");

  return;
}



void base_receive(int numsamples, char Path[])  //receive number of samples and write them to the file
{

  uint8_t set =0;
  uint16_t counter=0;
  uint8_t value, value2, value3; 
  int i;
  int valuearray[30];
  int j=0;
  char c;
  char tmpstrng[64];
  int n=0;
  comBuf *ptr;
  comBuf *packet;
  comBuf *packettoterm;
  char *pointer = tmpstrng;

  printf("\n");
  printf("The Micas should  have been programmed:  wait for signal.\n");

  com_mode(IFACE_SERIAL, IF_LISTEN);

  FILE *bcurrent;
  bcurrent=fopen(Path, "a");

  char recvchar, recvchar2;
  value=0;
  counter=0;

  packet = com_recv(IFACE_SERIAL);  //get the first random packet
  com_free_buf(packet);
     
  while(counter!=999)
    {
      packet = com_recv(IFACE_SERIAL);
      value = packet->data[0];
      if(value==97)
	{
	  value2 = packet->data[1];
     	  if(value2==97) 
	    {
	      counter=999;
	      printf("I got the exit command\n");
	       com_mode(IFACE_SERIAL, IF_OFF);
	    }
	  else if(value2 ==70)
	    {
	      value3 = packet->data[2];
	      printf("New frequency is: %d\n",value3);
	      fprintf(bcurrent, "frequency = %d", value3);
	    }
	}

      
      com_free_buf(packet);

      printf("%d\n", value);
      
      if(counter != 999)
	{
	   fprintf(bcurrent, "%d\n", value);
	}
	
    }
  fclose(bcurrent);  
  return;
}


void ApplySettings(int numsamples, int frequency, int power, int powerrecv, int testnum, int startpkt, int endpkt)
{    
  //now the program will modify the revieve and generate files
  char newnumsample[75]="numsamples=";
  char numsamplekey[75]="//change numsamples\n";
  char frequencykey[75]="//change frequency\n";
  char newfrequency[75]="frequency=";
  char powerkey[75]="//change power\n";
  char newpower[75]="power=";
  char inputline[75];
  char testnumkey[75] ="//test number\n";
  char newtestnum[75] ="testnumber=";
 
  //set GENERATOR.C FILE
  FILE *gener;
  gener= fopen("../../../../../src/apps/SendRecv/generator.c", "r");
        
  FILE *newgen;
  newgen = fopen("../../../../../src/apps/SendRecv/generator2.c", "w");
  
  while(fgets(inputline, 75, gener)!=NULL)
    {

      fputs(inputline, newgen);
      if(strcmp(inputline, numsamplekey)==0)
	{
	  fprintf(newgen, "%s%d;", newnumsample, numsamples);
	}
      else if(strcmp(inputline, frequencykey)==0)
	{
	  fprintf(newgen, "%s%d;", newfrequency, frequency);
	}
      else if(strcmp(inputline, powerkey)==0)
	{
	  fprintf(newgen, "%s%d;", newpower, power); 
	}
      else if(strcmp(inputline, testnumkey)==0)
	{
	  fprintf(newgen, "%s%d;", newtestnum, testnum);
	}
    }

  fclose(gener);
  fclose(newgen);
  

  //set reciever.C FILE
  FILE *recvr;
  recvr=fopen("../../../../../src/apps/SendRecv/receiver.c", "r");
        
  FILE *newrecv;
  newrecv = fopen("../../../../../src/apps/SendRecv/receiver2.c", "w");
  
  int deleteme=0;
    
  while(fgets(inputline, 75, recvr)!=NULL)
    {
      fputs(inputline, newrecv);

      if(strcmp(inputline, frequencykey)==0)

	{
	  fprintf(newrecv, "%s%d;", newfrequency, frequency);
	}
      else if(strcmp(inputline, powerkey)==0)
	{
	  fprintf(newrecv, "%s%d;", newpower, powerrecv); 
	}
      else if(strcmp(inputline, testnumkey)==0)
	{
	  fprintf(newgen, "%s%d;", newtestnum, testnum);
	}
    }

  fclose(recvr);
  fclose(newrecv);

  printf("\n");
  printf("Settings Applied, now make and load generator.srec and reciever.srec\n");
   
  return;
}



//stats function--This will Currently be replaced by another, updated analyze program
void getStats()
{

  char FileName[24];
  char location1[80]="../../../../../src/apps/AutoRF/1/";
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
  masterfile= fopen("../../../../../src/apps/RadioSuite/1/Master1", "r");

  while(fgets(FileName, 24, masterfile)!=NULL)
    {
      strcat(location1, FileName);

      FILE *currentData;
      currentData = fopen(location1, "r");

      if(currentData == 0)
	{
	  printf("Error opening File: %s", FileName);
	  //IMPLEMENT FUNCTIONALITY HERE(PROBABLY A FUNCTION)
	  //TO DELETE AN ENTRY FROM THE MASTER THAT IS NO PRESENT
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

    
#endif
