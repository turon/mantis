//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include "mos.h"
#include "com.h"
#include "dev.h"
#include "uart.h"

comBuf buf;

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

comBuf send_buf;
comBuf *recv_buf;

void start(void)
{
   uint32_t i;
   
   init_serial();
   
   for(i = 0; i < sizeof(send_buf.data); i++)
      send_buf.data[i] = i;
   
   for(i = 0; i < 700000; i++) {
      com_send(IFACE_LOOPBACK, &send_buf);
      recv_buf = com_recv(IFACE_LOOPBACK);
      com_free_buf(recv_buf);
   }
   
   buf.data[0] = 'd';
   buf.data[1] = 'o';
   buf.data[2] = 'n';
   buf.data[3] = 'e';
   buf.data[4] = '\n';
   buf.data[5] = '\0';
   buf.size = 6;
   
   send_packet(&buf);
}
