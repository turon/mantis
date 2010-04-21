#include "msched.h" // start()
#include "printf.h" // printf
#include "dev.h"    // device functions
#include "clock.h"  // mos_mdelay
#include "msp-humidity.h" // ioctl arguments

void start(void)
{
   uint16_t value;
   dev_open(DEV_MSP_HUMIDITY);
   
   dev_mode(DEV_MSP_HUMIDITY, DEV_MODE_ON);
   // turn off OTP reload for faster response time and lower power consumption
   dev_ioctl(DEV_MSP_HUMIDITY, SHT11_RELOAD_OFF);


   int i;
   for(i = 0; i < 5; ++i)
   {
      dev_read(DEV_MSP_TEMPERATURE, (void*)&value, sizeof(uint16_t));
      printf("temp = %x\n", value);
      dev_read(DEV_MSP_HUMIDITY, (void*)&value, sizeof(uint16_t));
      printf("hum  = %x\n", value);

      mos_mdelay(500);
   }

   dev_mode(DEV_MSP_HUMIDITY, DEV_MODE_OFF);
   dev_close(DEV_MSP_HUMIDITY);
}
