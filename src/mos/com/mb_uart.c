//  This file is part of MOS, the MANTIS Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (c) 2002 - 2007 University of Colorado, Boulder
//
//   All rights reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are
//   met:
//
//       * Redistributions of source code must retain the above copyright
//         notice, this list of conditions and the following disclaimer.
//       * Redistributions in binary form must reproduce the above
//         copyright notice, this list of conditions and the following
//         disclaimer in the documentation and/or other materials provided
//         with the distribution. 
//       * Neither the name of the MANTIS Project nor the names of its
//         contributors may be used to endorse or promote products derived
//         from this software without specific prior written permission.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//   POSSIBILITY OF SUCH DAMAGE.

#include "mos.h"

#if defined(PLATFORM_MICROBLAZE)
#include "mb_uart.h"
#include "xuartlite.h"

enum {
    MB_UART_RECV_IDLE,
    MB_UART_RECV_PREAMBLE,
    MB_UART_RECV_SIZE,
    MB_UART_RECV_DATA
};

#define MB_UART_DEVICE_ID XPAR_RS232_UART_1_DEVICE_ID
#define MB_UART_INTERRUPT_ID XPAR_OPB_INTC_0_RS232_UART_1_INTERRUPT_INTR

void RecvHandler(void *CallBackRef, unsigned int EventData);
void SendHandler(void *CallBackRef, unsigned int EventData);

static XUartLite UartLite;
static uint8_t mb_uart_in_byte;

static uint8_t mb_uart_recv_state;
static uint8_t mb_uart_rx_index;
static comBuf *mb_rx_buf;

void uart_init()
{
    mb_uart_recv_state = MB_UART_RECV_PREAMBLE;
    mb_uart_rx_index = 0;
    com_swap_bufs(IFACE_SERIAL, (comBuf *)NULL, &mb_rx_buf);
}

void mb_uart_recv_state_handler()
{
    switch(mb_uart_recv_state)
    {
        case MB_UART_RECV_PREAMBLE:
            {   //check if the current byte is a preamble byte
                if(mb_uart_in_byte == PREAMBLE)
                    mb_uart_rx_index++;
                else
                    mb_uart_rx_index = 0;

                //if we've gotten enough preamble bytes, move to receiving size
                if(mb_uart_rx_index == PREAMBLE_SIZE)
                {
                    if(mb_rx_buf) //make sure we have a buf to store the data in
                        mb_uart_recv_state = MB_UART_RECV_SIZE;
                    else
                    {   //if not get one
                        com_swap_bufs(IFACE_SERIAL, (comBuf *)NULL, &mb_rx_buf);
                    }

                }
                XUartLite_Recv(&UartLite, &mb_uart_in_byte, 1);
            }
            break;
        case MB_UART_RECV_SIZE:
            {   //get the size
                mb_rx_buf->size = mb_uart_in_byte;
                if(mb_rx_buf->size > COM_DATA_SIZE)
                    mb_uart_recv_state = MB_UART_RECV_PREAMBLE;
                else
                {
                    mb_uart_recv_state = MB_UART_RECV_DATA;
                    mb_uart_rx_index = 0;
                }
                XUartLite_Recv(&UartLite, &mb_uart_in_byte, 1);
            }
            break;
        case MB_UART_RECV_DATA:
            {   //get the next byte
                mb_rx_buf->data[mb_uart_rx_index++] = mb_uart_in_byte;
                if(mb_uart_rx_index == mb_rx_buf->size) //end of packet
                { //reset to waiting for preamble
                    mb_uart_recv_state = MB_UART_RECV_PREAMBLE;
                    mb_uart_rx_index = 0;
                    com_swap_bufs(IFACE_SERIAL, mb_rx_buf, &mb_rx_buf);
                }
                XUartLite_Recv(&UartLite, &mb_uart_in_byte, 1);
            }
            break;
    }
}

void mb_uart_int_init()
{
    XUartLite_Initialize(&UartLite, MB_UART_DEVICE_ID);

    mb_register_and_enable_interrupt(MB_UART_INTERRUPT_ID,
            (XInterruptHandler)XUartLite_InterruptHandler,
            &UartLite);

    XUartLite_SetRecvHandler(&UartLite, RecvHandler, &UartLite);
    XUartLite_SetSendHandler(&UartLite, SendHandler, &UartLite);


    XUartLite_EnableInterrupt(&UartLite);

    XUartLite_Recv(&UartLite, &mb_uart_in_byte, 1);
}
void SendHandler(void *CallBackRef, unsigned int EventData)
{
}
void RecvHandler(void *CallBackRef, unsigned int EventData)
{
    mb_uart_recv_state_handler();
//    mos_led_display(0xFFFF);
//    printf("recvd\r\n");
}

uint8_t com_send_IFACE_SERIAL(comBuf* buf)
{
    uint8_t *p = buf->data;
    uint8_t i = PREAMBLE;
    XUartLite_Send(&UartLite, &i, sizeof(i));
    while (XUartLite_IsSending(&UartLite));
    XUartLite_Send(&UartLite, &i, sizeof(i));
    while (XUartLite_IsSending(&UartLite));
    XUartLite_Send(&UartLite, &(buf->size), 1);
    while (XUartLite_IsSending(&UartLite));
//    XUartLite_Send(&UartLite, buf->data, buf->size);
//    while (XUartLite_IsSending(&UartLite));
    for(i = 0; i < buf->size; i++)
    {
        XUartLite_Send(&UartLite, p++, 1);
        while (XUartLite_IsSending(&UartLite));
    }
    return buf->size;
}

void com_mode_IFACE_SERIAL (uint8_t md)
{
}

/** @brief Generic io control for this driver.
 * @param request i/o request
 * @param ap Arguements
 * @return Always returns 0
*/
void com_ioctl_IFACE_SERIAL (uint8_t request, ...)
{
    int arg;
    va_list ap;
    va_start (ap, request);

    switch (request) {
        default:
            arg = va_arg (ap, int);
            break;
    }

    va_end (ap);
}


#endif
