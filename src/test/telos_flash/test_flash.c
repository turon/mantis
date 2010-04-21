#include "dev.h"
#include "telos-flash.h"
#include "msched.h"
#include "printf.h"

//#define TEST_WRITE
#define TEST_READ

uint16_t buffer;

static uint16_t new_id = 0xABBA;
void start(void)
{
   
   dev_open(DEV_TELOS_FLASH);
   dev_mode(DEV_TELOS_FLASH, DEV_MODE_ON);
   dev_ioctl(DEV_TELOS_FLASH, DEV_SEEK, (uint32_t)0);
   
#ifdef TEST_WRITE
   printf("Testing Write...");
   dev_ioctl(DEV_TELOS_FLASH, TELOS_FLASH_SECT_ERASE, (uint32_t)0);
   dev_write(DEV_TELOS_FLASH, (uint8_t*)&new_id, sizeof(new_id));
   printf("Done.\n");
#endif

#ifdef TEST_READ
   printf("Testing Read...");
   dev_ioctl(DEV_TELOS_FLASH, DEV_SEEK, (uint32_t)0);
   dev_read(DEV_TELOS_FLASH, &buffer, sizeof(buffer));
   printf("Read '%x'\n", buffer);
#endif

   dev_mode(DEV_TELOS_FLASH, DEV_MODE_OFF);
   dev_close(DEV_TELOS_FLASH);
}
