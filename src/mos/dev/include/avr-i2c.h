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

/** @file avr-i2c.h
 * @brief I2C driver for AVR.
 * @author Charles Gruenwald III
 * @date Created: 11/23/2004
 */


#ifndef _AVR_I2C_H
#define _AVR_I2C_H

#define avr_i2c_set_brr( new_twbr )     TWBR = new_twbr
#define avr_i2c_start_condition()       TWCR |= (1 << TWSTA)
#define avr_i2c_stop_condition()        TWCR |= (1 << TWSTO)
#define avr_i2c_interrupt_enable()      TWCR |= (1 << TWIE)
#define avr_i2c_interrupt_disable()     TWCR &= ~(1 << TWIE)
#define avr_i2c_enable()                TWCR |= (1 << TWEN)
#define avr_i2c_disable()               TWCR &= ~(1 << TWEN)
#define avr_i2c_get_status()            (TWSR & ~3)
#define avr_i2c_put_byte( new_byte )    TWDR = new_byte
#define avr_i2c_get_byte()              TWDR
#define avr_i2c_set_slave_addr( new_slave )  TWAR = (new_slave << 1)
#define avr_i2c_clear_int_flag()        TWCR |= (1 << TWINT)
#define avr_i2c_check_int_flag()        TWCR & (1 << TWINT)
#define avr_i2c_enable_general_call()   TWAR |= (1 << TWGCE)
#define avr_i2c_disable_general_call()  TWAR &= ~(1 << TWGCE)



#define I2C_ENABLE_ACK 0
#define I2C_DISABLE_ACK 1
#define I2C_DEST_ADDR 3
#define I2C_SLAVE_ADDR 4
#define I2C_SET_BRR 5

#define I2C_IDLE 0
#define I2C_READING 1
#define I2C_WRITING 2


//Status Codes for Master Transmitter Mode
#define START 0x08 //A START condition has been transmitted.
#define START_REPEAT  0x10 //A repeated START condition has been transmitted.
#define MT_SLA_ACK 0x18 //SLA+W has been transmitted; ACK has been received
#define MT_SLA_NACK 0x20 //SLA+W has been transmitted; NOT ACK received
#define MT_DATA_ACK 0x28 //DATA has been transmitted, ACK has been received
#define MT_DATA_NACK 0x30 //DATA has been transmitted, NOT ACK received
#define MT_ARB_LOST 0x38 //Arbitration lost in SLA+W or data bytes
#define MT_DATA_DONE 0x01

//Status Codes for Master Receiver Mode
#define MR_SLA_ACK 0x40 //SLA+R has been transmitted, ACK has been received
#define MR_SLA_NACK 0x48 //SLAR+R has been transmitted, NOT ACK received
#define MR_DATA_ACK 0x50 //DATA byte has been received, ACK returned
#define MR_DATA_NACK 0x58 //DATA has been received, NOT ACK returned

//Status Codes for Slave Receiver Mode
#define OWN_SLAW_RECEIVED_ACK 0x60 //Own SLA+W has been received; ACK returned
#define ARBITRATION_LOST_SLARW 0x68 //Arbitration lost in SLA+R/W as master; own SLA+W has been received; ACK has been returned
#define GENERAL_ADDR_RECEIVED 0x70 //General call address has been received; ACK has been returned
#define ARBITRATION_LOST_SLARW_GEN 0x78 //Arbitration lost in SLA+R/W as master; General call address has been received; ACK has been returned
#define PREV_ADDR_DATA_RECEIVED_ACK 0x80 //Previously addressed with own SLA+W; data has been received ACK has been returned
#define PREV_ADDR_DATA_RECEIVED_NACK 0x88 //Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define PREV_GENERAL_ADDR_DATA_RECEIVED_ACK 0x90 //Previously addressed with general call; data has been received; ACK has been returned
#define PREV_GENERAL_ADDR_DATA_RECEIVED_NACK 0x98 //Previously addressed with general call; data has been received; NOT ACK has been returned
#define STOP_REPEATED_START 0xA0 //A STOP condition or repeated START condition has been received while still addressed as slave

//Status Codes for Slave Transmitter Mode
#define SLAR_RECEIVED_ACK 0xA8 //Own SLA+R has been received; ACK has been returned.
#define ARBITRATION_LOST_SLARW_ACK 0xB0 //Arbitration lost in SLA+R/W as master; own SLA+R has been received; ACK has been returned
#define SLAVE_DATA_XMITTED_ACK 0xB8 //Data byte in TWDR has been transmitted; ACK hs been received
#define SLAVE_DATA_XMITTED_NACK 0xC0 //Data byte in TWDR has been transmitted; NOT ACK has been received
#define LAST_BYTE_XMITTED 0xC8 //Last data byte in TWDR has been transmitted (TWEA = "0"); ACK has been received

//Miscellaneous States
#define NO_RELEVANT_STATE 0xF8 //No relevant state information available; TWINT="0"
#define BUS_ERROR 0x00 //Bus error due to an illegal START or STOP condition

#define I2C_MT 0  //master transmitter
#define I2C_MR 1  //master receiver
#define I2C_ST 2  //slave transmitter
#define I2C_SR 3  //slave receiver

/** @brief Initialize the eeprom driver
 */
void avr_i2c_init();

#endif
