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

/* File mb_adc.c
 * Author: Charles Gruenwald III
 * Date: 04/26/2007
 */

#include "mos.h"
#include "dev.h"

#ifdef PLATFORM_MICROBLAZE

#include "mb-spi.h"
#include "xspi.h"
#include "xspi_l.h"
#include "xilinx_help.h"

static XSpi Spi; /* The instance of the SPI device */
volatile static Xboolean spi_finished;
#define SPI_ID XPAR_OPB_SPI_0_DEVICE_ID

#define SPI_INTERRUPT_ID XPAR_OPB_INTC_0_OPB_SPI_0_IP2INTC_IRPT_INTR

XSpi_StatusHandler spi_status(void *CallBackRef, Xuint32 StatusEvent,
        unsigned int ByteCount);

void spi_init()
{
    /*
    mb_register_and_enable_interrupt(SPI_INTERRUPT_ID,
            (XInterruptHandler)XSpi_InterruptHandler,
            (void *)&Spi);
*/
    Xuint16 Control;
    spi_finished = false;
    XSpi_Initialize(&Spi, SPI_ID);
    Control = (XSP_CR_MASTER_MODE_MASK);
    XSpi_mSetControlReg(Spi.BaseAddr, Control);
    XSpi_SetOptions(&Spi, XSP_MASTER_OPTION | XSP_MANUAL_SSELECT_OPTION);
//    XSpi_SetSlaveSelect(&Spi, 0x01);
    XSpi_mSetSlaveSelectReg(Spi.BaseAddr, (0xFF));

//    XSpi_SetStatusHandler(&Spi, (void *)&spi_finished, (XSpi_StatusHandler)spi_status);
//    XSpi_Start(&Spi);

//    XSpi_mEnable(Spi.BaseAddr);
}

XSpi_StatusHandler spi_status(void *CallBackRef, Xuint32 StatusEvent,
        unsigned int ByteCount)
{
    //    printf("spi status: %d bytes: %d\r\n", StatusEvent, ByteCount);
    *(Xboolean *)CallBackRef = true;
    return NULL;
}

uint8_t dev_mode_DEV_SPI(uint8_t new_mode)
{
    return DEV_UNSUPPORTED;
}

uint16_t dev_read_DEV_SPI(void *buf, uint16_t count)
{
    Xuint16 ret;
    uint16_t i;
    for(i = 0; i < count; i++)
    {
        XSpi_mSendByte(Spi.BaseAddr, 0);
//        XSpi_mEnable(Spi.BaseAddr);
        while (!(XSpi_mGetStatusReg(Spi.BaseAddr) & XSP_SR_TX_EMPTY_MASK));
        ((uint8_t *)buf)[i] = XSpi_mRecvByte(Spi.BaseAddr);
//        XSpi_mDisable(Spi.BaseAddr);
    }

    return count;

//    XSpi_Transfer(&Spi, (Xuint8 *)&ret, (Xuint8 *)&ret, count);

    //Wait for transfer to finish
//    while(spi_finished == false);
//    spi_finished = false;
//    XSpi_mSetSlaveSelectReg(Spi.BaseAddr, (0xFF));

//    *(uint16_t *)buf = ret;
//    return count;
}

uint16_t mb_spi_send_byte(uint8_t byte)
{
//    mos_udelay(1);
    XSpi_mSendByte(Spi.BaseAddr, (uint8_t)byte);
    while (!(XSpi_mGetStatusReg(Spi.BaseAddr) & XSP_SR_TX_EMPTY_MASK));
    uint8_t garbage;
    garbage = XSpi_mRecvByte(Spi.BaseAddr);
//    mos_udelay(1);
    return 0;
}

uint8_t mb_spi_get_byte()
{
//    mos_udelay(1);
    XSpi_mSendByte(Spi.BaseAddr, 0);
    while (!(XSpi_mGetStatusReg(Spi.BaseAddr) & XSP_SR_TX_EMPTY_MASK));
//    mos_udelay(1);
    return XSpi_mRecvByte(Spi.BaseAddr);
}

uint16_t dev_write_DEV_SPI(const void *buf, uint16_t count)
{
    uint16_t i;
    uint8_t garbage;
    for(i = 0; i < count; i++)
    {
        volatile uint8_t byte_to_send = ((uint8_t *)buf)[i];
        XSpi_mSendByte(Spi.BaseAddr, byte_to_send);
        //            XSpi_mEnable(Spi.BaseAddr);
        while (!(XSpi_mGetStatusReg(Spi.BaseAddr) & XSP_SR_TX_EMPTY_MASK));
        garbage = XSpi_mRecvByte(Spi.BaseAddr);
        //            XSpi_mDisable(Spi.BaseAddr);
    }

    /*
    Xuint16 ret;
    XSpi_Transfer(&Spi, (Xuint8 *)&ret, NULL, count);

    //Wait for transfer to finish
    while(spi_finished == false);
    spi_finished = false;
    */
//    XSpi_mSetSlaveSelectReg(Spi.BaseAddr, (0xFF));

//    *(uint16_t *)buf = ret;
    return count;
}

uint8_t dev_ioctl_DEV_SPI(int8_t request, ...)
{
    va_list ap;
    int arg;
    va_start (ap, request);

    switch (request)
    {
        case SPI_SET_SLAVE_SELECT:
            {
                XSpi_mEnable(Spi.BaseAddr);
                arg = va_arg (ap, int);
                switch(arg)
                {
                    case MB_SPI_ADC_SLAVE_SELECT:
                        XSpi_mSetSlaveSelectReg(Spi.BaseAddr, ~(0x01));
                        mos_udelay(1);
                        break;
                    case CC2420_SLAVE_SELECT:
                        XSpi_mSetSlaveSelectReg(Spi.BaseAddr, ~(0x02));
                        mos_udelay(1);
                        break;
                    default:
                        va_end(ap);
                        return DEV_BAD_IOCTL;
                };
            }
            break;
        case SPI_CLEAR_SLAVE_SELECT:
            {
                mos_udelay(1);
                XSpi_mSetSlaveSelectReg(Spi.BaseAddr, ~(0x00));
                XSpi_mDisable(Spi.BaseAddr);
            }
            break;
        default:
            printf("error: bad ioctl on mb spi\r\n");
            va_end(ap);
            return DEV_BAD_IOCTL;
    }

    va_end(ap);

    return DEV_OK;
}

#endif
