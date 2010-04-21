//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "mos.h"
#include "com.h"
#include "uart.h"
#include "sem.h"

comBuf buf;

#define SINGLE_THREADED
//#define DOUBLE_THREADED

void init_serial()
{
   // Setup default baud rate
   UBRR0H = (uint8_t)(DEFAULT_BAUD_RATE >> 8);
   UBRR0L = (uint8_t)(DEFAULT_BAUD_RATE);

   // enable transmitter and receiver
   UCSR0B = (1 << RXEN0) | (1 << TXEN0);
   
   // no parity, 8 bits
   UCSR0C = (3 << UCSZ00);
}

/** @brief Send a byte over the UART.
 * @param c Byte to send
 */
void send_byte(uint8_t c)
{
   while (!(UCSR0A & (1 << UDRE0)));    // wait until reg clear
   UDR0 = c;                    // prepare transmission
}

/** @brief Receive a byte from the UART.
 * @return Byte received
 */
uint8_t recv_byte(void)
{
   while(!(UCSR0A & (1 << RXC0)));  // wait for data
   return UDR0;
}

/** @brief Check to see if data is available from the UART.
 * @return 1 if data is available
 */
int8_t check_recv(void)
{
   if(UCSR0A & (1 << RXC0))
      return 1;
   else
      return 0;
}

uint8_t send_packet(comBuf *pkt)
{
   uint8_t i;

   // Send the preamble
   for(i = 0; i < PREAMBLE_SIZE; i++)
      send_byte(PREAMBLE);

   // Now send size and data
   send_byte(pkt->size);
   for(i = 0; i < pkt->size; i++)
      send_byte(pkt->data[i]);

   return 0;
}

#ifdef SINGLE_THREADED
mos_sem_t count_sem;
#endif
#ifdef DOUBLE_THREADED
mos_sem_t count1_sem;
mos_sem_t count2_sem;
#endif

uint32_t num1;
uint32_t *p1;
#ifdef DOUBLE_THREADED
uint32_t num2;
uint32_t *p2;
#endif

uint32_t wait_loop1(void)
{
   num1 = 0;
   p1 = &num1;
   while(*p1 < 18000000) {
      (*p1)++;
   }
   return *p1;
}

#ifdef DOUBLE_THREADED
uint32_t wait_loop2(void)
{
   num2 = 0;
   p2 = &num2;
   while(*p2 < 18000000) {
      (*p2)++;
   }
   return *p2;
}
#endif

#ifdef SINGLE_THREADED
void count_thread(void)
{
   wait_loop1();
   wait_loop1();
   
   mos_sem_post(&count_sem);
}
#endif
#ifdef DOUBLE_THREADED
void count1_thread(void)
{
   wait_loop1();
   mos_sem_post(&count1_sem);
}

void count2_thread(void)
{
   wait_loop2();
   mos_sem_post(&count2_sem);
}
#endif
void start(void)
{
   init_serial();   

#ifdef SINGLE_THREADED
   mos_sem_init(&count_sem, 0);
   mos_thread_new(count_thread, 128, PRIORITY_NORMAL);
   mos_sem_wait(&count_sem);
#endif
#ifdef DOUBLE_THREADED   
   mos_sem_init(&count1_sem, 0);
   mos_sem_init(&count2_sem, 0);
   mos_thread_new(count1_thread, 128, PRIORITY_NORMAL);
   mos_thread_new(count2_thread, 128, PRIORITY_NORMAL);
   mos_sem_wait(&count1_sem);
   mos_sem_wait(&count2_sem);
#endif
   
   buf.data[0] = 'd';
   buf.data[1] = 'o';
   buf.data[2] = 'n';
   buf.data[3] = 'e';
   buf.data[4] = '\n';
   buf.data[5] = '\0';
   buf.size = 6;
   
   send_packet(&buf);
}
