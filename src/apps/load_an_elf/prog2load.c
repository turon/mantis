#include <inttypes.h>

#include "mos.h"
#include "msched.h"
#include "led.h"
#include "clock.h"

int shutdown = 0;

void blink_a (void)
{
  uint32_t sleep_time_a = 100;
   
  while (!shutdown) {
    mos_led_toggle (0);
    mos_thread_sleep (sleep_time_a);
  }
  printf("blink_a finished\n");
  mos_thread_exit();
}

void load (void)
{
  printf("load() called\n");
  mos_thread_new (blink_a, 128, PRIORITY_NORMAL);
}

void unload (void)
{
  printf("unload() called\n");
  shutdown = 1;
}
