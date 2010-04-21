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

#if defined(PLATFORM_MICA_ANY)


#include "mica2-gps.h"
#include "avr-i2c.h"
#include "dev.h"
#include "mutex.h"
#include "com.h"
#include "uart.h"
#include "printf.h"
#include <string.h>
#include <stdlib.h>

#if defined(MICA2_GPS) || !defined(SCONS)

mos_mutex_t mica2_gps_mutex;
static uint8_t gps_mode;
static uint8_t read_type;

// helper functions.


void gps_disable_bits(uint8_t mask);
void gps_enable_bits(uint8_t mask);
void gps_clear_bits();


/** @brief turn on the GPS */
void gps_on(void);
/** @brief turn off the GPS */
void gps_off(void);
/** @brief determines if a GPS packet is a GGA packet.
 * @param a string holding a GPS packet.
 * @return TRUE if str points to a GGA packet.
 */
boolean mica2_gps_is_gga(char* str);

/** @brief parses a string into a gps_gga_t structure.
 *The string passed to parse_gga is destroyed.
 * @param str a GPS GGA packet.  This string will be destroyed.
 * @param out a pointer to a gps_gga_t structure to fill in with data from the string.
 * @return true if the structure has been filled in; false otherwise.
 */
boolean mica2_gps_parse_gga(char* str, gps_gga_t* out);
/** @brief parse a string into a timestamp.
 * @param str the string containing a timestamp to parse.
 * @return the timestamp in timestamp_t form.
 */
timestamp_t get_timestamp(const char* str);
/** @brief parse a GGA substring into a latitude or longitude.
 * @param part1 the degrees.minutes string.
 * @param part2 the direction character ('N', 'E', etc..).
 * @param lat a boolean value indicating that this is a latitude; otherwise, it is a longitude.
 * @return a latlong_t with the degrees/minutes and direction specified by the strings.
 */
latlong_t get_latlong(const char* part1, const char* part2, boolean lat);
/** @param converts strings of the form 'whole.decimal' and 'Unit' into a measurement_t struct.
 * @param part1 the 'whole.decimal' string.
 * @param part2 the unit string.
 * @return a measurement_t with the appropriate values.
 */
measurement_t get_measurement(const char* part1, const char* part2);
//


// this buffer is used to hold GPS data temporarily when using
// dev_read in READ_GGA mode.
#define GPS_BUFFER_SIZE 96
static uint8_t gps_buffer[GPS_BUFFER_SIZE];



#define UART1_WAIT_FOR_TXC()                            \
   while(!(UCSR1A & (1 << TXC1)));			\
   UCSR1A |= (1 << TXC1);



// sscanf lives in stdio.h, but we don't want to include stdio.h
// because it would override our printf/putc implementations.
extern int sscanf(const char* str, const char* fmt, ...);


boolean mica2_gps_is_gga(char* str)
{
   return (strlen(str) >= 6 &&
	   str[3] == 'G' &&
	   str[4] == 'G' &&
	   str[5] == 'A');
}

void split_commas(char* str, char** values, uint16_t* value_count)
{
   uint16_t i = strlen(str);
   char* end = str + i;
   char* p;

   *value_count = 0;

   // replace all the commas with nulls to create a bunch of litte strings.
   // this will discard the first string ($GPGGGA)
   // and the last string (the checksum).
   for(p = str; p < end; ++p)
   {
      if (*p == ',')
      {
	 *p = '\0';
	 // keep a pointer to each little string
	 // in our array
	 values[(*value_count)++] = p + 1;
      }
   }
}


boolean mica2_gps_parse_gga(char* str, gps_gga_t* gga)
{
   //printf("%s\n", str);
   
   memset(gga, 0, sizeof(gps_gga_t));
   
   // need to pass this as the 'unit' for the horiz. dilution.
   char dummy = '\0';
   
   uint16_t value_count = 0;
   char* values[20];
   split_commas(str, values, &value_count);
   


   // GGA message should have 14(?) fields
   if (value_count != 14)
   {
      //printf("not enough fields in GGA data\n");
      return false;
   }
   else
   {
      gga->utc = get_timestamp(values[0]);
      gga->latitude = get_latlong(values[1], values[2], true);
      gga->longitude = get_latlong(values[3], values[4], false);
      gga->fix_quality = atoi(values[5]);
      gga->satellite_count = atoi(values[6]);
      gga->horiz_dilution = get_measurement(values[7], &dummy);
      gga->altitude = get_measurement(values[8], values[9]);
      gga->geoid_height = get_measurement(values[10], values[11]);
      gga->since_last_update = atoi(values[12]);
      gga->dgps_id = atoi(values[13]);
      return true;
   }
   
}


// reads a timestamp of the format HHMMSS
timestamp_t get_timestamp(const char* str)
{
   timestamp_t result;
   memset(&result, 0, sizeof(result));
   if (*str == 0)
      return result;
   
   result.hours = (str[0] - '0') * 10 + (str[1] - '0');
   result.minutes =  (str[2] - '0') * 10 + (str[3] - '0');
   result.seconds = (str[4] - '0') * 10 + (str[5] - '0');

   return result;
}

// reads a latitude/longitude of the format
// degrees.minutes,direction
//
latlong_t get_latlong(const char* part1, const char* part2, boolean lat)
{
   latlong_t result;
   memset(&result, 0, sizeof(result));
   
   if (*part1 != 0)
   {
      sscanf(part1,  (lat ? "%02d%d.%d" : "%03d%d.%d"),
	     &result.degrees,
	     &result.minutes.whole,
	     &result.minutes.decimal); 
   }
   
   if (*part2 != 0)
      result.direction = part2[0];
   else
      result.direction = '?';
   

   return result;
}

// reads a measurement value of the format "whole.decimal,units"
// fix: units can only be one character.  it's possible that the
// gps might return "Km" or "Mi", but I've only seen "M".
measurement_t get_measurement(const char* part1, const char* part2)
{
   measurement_t result;
   memset(&result, 0, sizeof(result));

   if (*part1 != 0)
      sscanf(part1, "%d.%d", &result.whole, &result.decimal) ;

   if (*part2 != 0)
      result.units = part2[0];
   else
      result.units = '?';

   
   return result;
}


void mica2_gps_print_gga(const gps_gga_t* gga)
{
   printf("utc:  %02d:%02d:%02d\n",
	  gga->utc.hours,
	  gga->utc.minutes,
	  gga->utc.seconds);
   
   printf("lat:  %d deg. %d.%d min. %c\n",
	  gga->latitude.degrees,
	  gga->latitude.minutes.whole,
	  gga->latitude.minutes.decimal,
	  gga->latitude.direction);
   
   printf("long: %d deg. %d.%d min. %c\n",
	  gga->longitude.degrees,
	  gga->longitude.minutes.whole,
	  gga->longitude.minutes.decimal,
	  gga->longitude.direction);
   
   printf("fix:  %C\n", gga->fix_quality);
   printf("sats: %C\n", gga->satellite_count);
   
   printf("horz: %d.%d\n",
	  gga->horiz_dilution.whole,
	  gga->horiz_dilution.decimal);
   
   printf("alt:  %d.%d %c\n",
	  gga->altitude.whole,
	  gga->altitude.decimal,
	  gga->altitude.units);
   
   printf("geo:  %d.%d %c\n",
	  gga->geoid_height.whole,
	  gga->geoid_height.decimal,
	  gga->geoid_height.units);
   
   printf("sec:  %d\n", gga->since_last_update);
   printf("dgps: %d\n\n", gga->dgps_id);
   
}

uint8_t dev_ioctl_DEV_MICA2_GPS(int8_t request, ...)
{
   va_list ap;
   
   va_start (ap, request);
   
   switch (request)
   {
   case MICA2_GPS_READ_STRING:
   case MICA2_GPS_READ_GGA:
      read_type = request;
      break;
      /* other packet types can be implemented in the same way.
       * they just need a structure (ex: gps_gga_t),
       * a MICA2_GPS_READ_TYPE enum value (ex MICA2_GPS_READ_GGA),
       * a block in the dev_read_DEV_MICA_GPS() switch statement,
       * and a function to parse the string into the structure
       */
   default:
      return DEV_BAD_IOCTL;
   }

   va_end(ap);

   return DEV_OK;
}


uint8_t dev_mode_DEV_MICA2_GPS(uint8_t md)
{
   if (md == DEV_MODE_OFF)
   {
      gps_mode = md;
      gps_disable_bits(GPS_TX_RX);
      gps_off();
      return DEV_OK;
   }
   else if (md == DEV_MODE_ON)
   {
      gps_mode = md;
      gps_disable_bits(GPS_TX_RX);
      gps_on();
      return DEV_OK;
   }
   else
      return DEV_UNSUPPORTED;
   
}



uint16_t dev_write_DEV_MICA2_GPS(const void* buffer, uint16_t len)
{
   
   //while ( !(UCSR1A & (1 << RXC1)));
   gps_disable_bits(GPS_TX_ADDR);
   gps_enable_bits(GPS_RX_ADDR);
   //  com_mode(IFACE_SERIAL2, IF_LISTEN);
   com_ioctl(IFACE_SERIAL2, UART_IOCTL_RAW_MODE0);
   // com_ioctl(IFACE_SERIAL2, UART_IOCTL_BAUD_RATE, B4800);
   
   comBuf b;
   memcpy(b.data, buffer, len);
   b.size = len;

   
   com_send(IFACE_SERIAL2, &b);

   UART1_WAIT_FOR_TXC();
   
   gps_disable_bits(GPS_RX_ADDR);
   gps_enable_bits(GPS_TX_ADDR);

   return DEV_OK;
   
}

uint16_t mica2_gps_readline(char* buff, uint16_t len)
{
   uint16_t i = 0;
   char c;
   // while we have room left in the buffer plus a null terminator
   while(i < len - 1)
   {
      // wait for data on UART 1
      while ( !(UCSR1A & (1 << RXC1)));
      
      c = UDR1;
      
      // packets are terminated with \r\n; skip the \r, when we catch
      // a \n, null terminate the string and we're done.
      if (c == '\r')
	 continue;
      else if (c == '\n')
      {
	 buff[i] = 0;
	 break;
      }
//      else if (( c&0x00FF ) == 0x00B3) { buff[i] = c; break; }
      else // otherwise store the data in the buffer.
      {
	 buff[i++] = c;
      }
   }
   

   // if we've run out of room, add a null terminator and return.
   if (i == len - 1)
      buff[i] = 0;

   return i;
}

uint16_t mica2_gps_readsirf(uint8_t* buffer, uint16_t len)
{
  uint8_t status = 0;
  uint8_t tmp;
  uint16_t psize = 0;
 
  // wait for the header 0xA0 0xA2
  while(1)
  {
     while ( !(UCSR1A & (1 << RXC1)));

     tmp = UDR1;
     //printf("read %x\n", (uint16_t)(tmp));

     if (tmp == 0xA0)
      status = 1;
     else 
       if (tmp == 0xA2 && status == 1 )
         break;
       else
         status = 0;
  }

  //printf("got header\n");
  buffer[0] = 0xA0;
  buffer[1] = 0xA2;

  // now read the length: two bytes, MSByte first
  while ( !(UCSR1A & (1 << RXC1)));

  // read hi byte
  psize = UDR1;
  //buffer[2] = psize;  
  psize <<= 8;
  buffer[2] = psize;
  while ( !(UCSR1A & (1 << RXC1)));

  // read lo byte
  psize |= UDR1;
  buffer[3] = psize;
  //buffer[3] = UDR1;
  //printf("read payload length as %d\n", psize);

  //memcpy(buffer+2, &psize, 2);
  // if the payload size + header + footer is > len
  if (psize + 8 > len)
  {  
     printf("uh oh, buffer's not big enough for size %d\n", psize);
     return 0;
  }


  // copy length to buffer

  int i = 4;

  // read payload + checksum + end seq
  for(i; i < psize + 4; i++)
  {
     while ( !(UCSR1A & (1 << RXC1)));
     buffer[i] = UDR1;
  }

  // return length
  return i;
 
}



uint16_t dev_read_DEV_MICA2_GPS(void* buffer, uint16_t len)
{
   uint16_t ret_val = 1;
   // if the GPS is turned off, turn it on.
   if (gps_mode == DEV_MODE_OFF)
   {
      gps_on();
      gps_enable_bits(GPS_TX_RX);
   }
   gps_enable_bits(GPS_TX_ADDR);
   
   // determine what type of variable we should store in buffer.
   switch(read_type)
   {
      
   case MICA2_GPS_READ_GGA:
      // we should store a gps_gga_t in buffer.
      // if buffer is not the size of a gps_gga_t, we have a problem.
      if (len != sizeof(gps_gga_t))
      {
	 ret_val = 0;
	 break;
      }

      // loop until we read a GGA packet
      do
      {
	 mica2_gps_readline(gps_buffer, GPS_BUFFER_SIZE);
      }while(!mica2_gps_is_gga(gps_buffer));
      // parse the packet into buffer.
      mica2_gps_parse_gga(gps_buffer, (gps_gga_t*)buffer);
      
      break;
      
   case MICA2_GPS_READ_STRING:
      // we should store a simple string in buffer.
      mica2_gps_readline(buffer, len);
      break;
      
   }
   // if the GPS was turned off, turn it back off.
   if (gps_mode == DEV_MODE_OFF)
   {
      gps_disable_bits(GPS_TX_RX);
      gps_off();
   }
   
   return 1;
}


void gps_disable_bits(uint8_t mask)
{
   uint8_t i;
   // set the address from which to read/write
   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR, SWITCH_2);
   // read the current value
   dev_read(DEV_AVR_I2C, &i, sizeof(i));
   // clear the RX/TX bits
   i &= ~(mask);
   // write the value back
   dev_write(DEV_AVR_I2C, &i, sizeof(i));

   // verify that the bits have been cleared.
   do
   {
      i = 0;
      dev_read(DEV_AVR_I2C, &i, sizeof(i));
   }while(i & (mask));
   
   // disable rx
   UCSR1B &= ~(1 << RXEN1);
}

void gps_enable_bits(uint8_t mask)
{
   // disable uart1 interrupt
   UCSR1B &= ~((1 << RXCIE1)  | (1 << RXEN1));
   // enable rx
   UCSR1B |= (1 << RXEN1);
   
   uint8_t i;
   // set the address from which to read/write
   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR, SWITCH_2);
   // read the current value
   dev_read(DEV_AVR_I2C, &i, sizeof(i));
   // set the RX/TX bits
   i |= (mask);
   // write the value back
   dev_write(DEV_AVR_I2C, &i, sizeof(i)); //overwrite value

   // verify that the bits have been written correctly.
   do
   {
      i = 0;
      dev_read(DEV_AVR_I2C, &i, sizeof(i));
   }while((i & (mask)) != (mask));
}


#define GPS_ENABLE_MASK (GPS_ENABLE_ADDR)

void gps_on(void)
{
   uint8_t i;
   // set speed
   dev_ioctl(DEV_AVR_I2C, I2C_SET_BRR, 50);
   // set the address from which to read/write
   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR, SWITCH_1);
   // read the current value
   dev_read(DEV_AVR_I2C, &i, sizeof(i));
   // set the appropriate bits.
   i |= GPS_ENABLE_MASK;
   // write the value back
   dev_write(DEV_AVR_I2C, &i, sizeof(i));

   // verify that the bits have been written correctly.
   do
   {
      i = 0;
      dev_read(DEV_AVR_I2C, &i, sizeof(i));
   }while((i & (GPS_ENABLE_MASK)) != GPS_ENABLE_MASK);
}


void gps_off(void)
{
   uint8_t i;

   // set the address from which to read/write
   dev_ioctl(DEV_AVR_I2C, I2C_DEST_ADDR, SWITCH_1);
   // read the current value
   dev_read(DEV_AVR_I2C, &i, sizeof(i));
   // mask out the appropriate bits
   i &= (uint8_t)(~GPS_ENABLE_MASK);
   // write the value back.
   dev_write(DEV_AVR_I2C, &i, sizeof(i));

   // verify that the bits have been cleared.
   do 
   {
      i = 0;
      dev_read(DEV_AVR_I2C, &i, sizeof(i));
   }while((i & (GPS_ENABLE_MASK)));
}


void mica2_gps_init(void)
{
   mos_mutex_init(&mica2_gps_mutex);
   gps_mode = DEV_MODE_OFF;
   read_type = MICA2_GPS_READ_STRING;
}


#endif // defined(MICA2_GPS) || !defined(SCONS)
#endif // PLATFORM_MICA_ANY
