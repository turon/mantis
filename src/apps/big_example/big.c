#include "mos.h"
#include "com.h"
#include "dev.h"
#include "printf.h"
#include "cc2420.h"
#include "led.h"

// this function calculates a simple 8-bit
// exclusive or checksum of the provided data 
uint8_t simple_crc(uint8_t* data, uint8_t bytes);
// this function determines what sensors should be read
// from, reads their values into the provided packet,
// and returns the number of bytes.  It includes a
// CRC.
uint8_t read_sensors(comBuf* b);
// this function unpacks the packet on the rx end.
void parse_packet(comBuf* p);
// this thread sends our data every 5 seconds.
void tx_thread(void);
// this thread constantly polls for incoming data.
void rx_thread(void);


comBuf send_buf;

void tx_thread(void)
{
   while(1)
   {

      send_buf.size = read_sensors(&send_buf);
      
      com_send(IFACE_RADIO, &send_buf);
      printf("[TX] sent %d bytes\n", send_buf.size);
      mos_led_toggle(0);
      mos_thread_sleep(5000);
   }
}


void rx_thread(void)
{
 
   
   while(1)
   {
      comBuf* p = com_recv(IFACE_RADIO);
      printf("[RX] received %d bytes\n", p->size);

      parse_packet(p);
      
      com_free_buf(p);
      
   }
}

void start(void)
{

   com_mode(IFACE_RADIO, IF_LISTEN);
   com_ioctl(IFACE_RADIO, CC2420_LOW_POWER_MODE);
   
   mos_thread_new(tx_thread, 128, PRIORITY_NORMAL);
   mos_thread_new(rx_thread, 128, PRIORITY_NORMAL);
}

uint8_t simple_crc(uint8_t* data, uint8_t bytes)
{
   uint8_t crc = 0;
   
   while(bytes--)
      crc ^= *(data++);

   return crc;
}


// builds a packet in the following format:
// [uint8_t] device code
// [void*  ] device reading
// repeat.
uint8_t read_sensors(comBuf* out)
{
   uint16_t scratch;
   uint8_t index = 0;

   // the first item in the packet will be
   // a 4-byte hardware ID.
   uint32_t hid;

   dev_read(DEV_HARDWARE_ID, &hid, sizeof(hid));

   out->data[index++] = DEV_HARDWARE_ID;
   buf_insert32(out->data, index, hid);

   index += sizeof(hid);
   
   
#ifdef PLATFORM_TELOSB
   // read the temperature value
   dev_read(DEV_MSP_TEMPERATURE, &scratch, sizeof(scratch));
  
   out->data[index++] = DEV_MSP_TEMPERATURE;
   buf_insert16(out->data, index, scratch);
   
   index += sizeof(scratch);
   
   dev_read(DEV_MSP_HUMIDITY, &scratch, sizeof(scratch));

   // and the humidity value
   out->data[index++] = DEV_MSP_HUMIDITY;
   buf_insert16(out->data, index, scratch);
   
   index += sizeof(scratch);
   
#endif

#ifdef ARCH_AVR
   // TODO: Read from MICAZ/MICA2 sensors/GPS, etc.
   

#endif
   
   out->data[index] = simple_crc(out->data, index);
   
   return ++index;
   
}

// these macros are scaled math following the formulas in the data sheet for the temp/humidity sensors
#define CALC_MSP_HUM(n) ( -4 + ((n)/25) + ((n)/2000) + ((n)*(n)/-500000) + ((n)*(n)/-1250000) )
#define CALC_MSP_TEMP(n) ( ((n)/100) + ((n)/125) - 40 )

void parse_packet(comBuf* p)
{
   int index = 0;

   uint8_t crc = simple_crc(p->data, p->size - 1);

   if (crc != p->data[p->size - 1])
   {   
      printf("CRC Mismatch: Calculated %02x, Read %02x\n", crc, p->data[p->size - 1]);
      return;
   }

   // walk through the entire packet, up until just before the final byte,
   // which is the CRC.
   while(index < p->size - 1)
   {
      uint8_t dev_code = p->data[index];
      uint16_t scratch;

      // skip past the device code to the device data
      index++;
      
      switch(dev_code)
      {
      case DEV_HARDWARE_ID:
	 printf("Hardware Id: 0x%02h%02h%02h%02h\n",
		p->data[index + 0], p->data[index + 1],
		p->data[index + 2], p->data[index + 3]);

	 index += 4;
	 break;
      case DEV_MSP_TEMPERATURE:
	 scratch = buf_extract16(p->data, index);
	 printf("MSP Temperature: %d F\n", CALC_MSP_TEMP(scratch));
	 index += 2;
	 break;
      case DEV_MSP_HUMIDITY:
	 scratch = buf_extract16(p->data, index);
	 printf("MSP Humidity: %d %%\n", CALC_MSP_HUM(scratch));
	 index += 2;
	 break;
      case DEV_MICA2_GPS:
	 break;
      default:
	 printf("Unknown Device Code '%d'\n", dev_code);
	 return;
      }
      
   }

   printf("\n");
   
   
}

