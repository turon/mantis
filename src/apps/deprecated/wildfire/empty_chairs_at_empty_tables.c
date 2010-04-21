
#include <inttypes.h>

#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "com.h"

static comBuf sendBuf;
static uint8_t state;

#define INIT 0
#define GO 1
#define SLEEPNODE 2
#define READYNODE 3

#define SLEEPTIME 600

uint16_t wind_ticks;
uint16_t wind_speed;
mos_alarm_t wind_timer;

uint32_t wind_timer_ms = 1000;

static mos_mutex_t state_mutex;

void wind_timer_callback();
void send();
void sleepthread();
void thread1();
void thread2();


void start(void)
{
   com_mode(IFACE_RADIO, IF_LISTEN);

   //wind_timer.func = wind_timer_callback;
   //wind_timer.kbus = wind_timer_ms;
   // setting 'reset_to' means that the alarm
   // will be added back into the list with
   // the 'reset_to' time after each time it expires.
   //wind_timer.reset_to = wind_timer_ms;
   //mos_alarm (&wind_timer);

   mos_thread_new (sleepthread, 128, PRIORITY_NORMAL);  
   mos_thread_new (thread1, 128, PRIORITY_NORMAL);
  
   state = GO;

}

void thread1()
{
    printf ("thread1 running\n");
    
    while(1)
    {
	if (state == SLEEPNODE)
	{
	    printf("sleeping thread 1\n");
	    mos_thread_sleep (SLEEPTIME);
	    printf("waking thread 1\n");
	}
    }
}


void sleepthread()
{
    uint32_t myvar;

    printf ("sleepthread running\n");
    myvar = 390;
    
    printf ("starting while\n");  //stops here??????!?!?!????
    while (1)
    {

	printf ("setting sleep time of %d\n", myvar);
	mos_thread_sleep(myvar);
	printf ("done sleeping\n");
	// DOESN'T WAKE UP HERE!!!!

	printf ("locking mutex 2\n");
	mos_mutex_lock(&state_mutex);
	state = SLEEPNODE;
	mos_led_on(0);
	mos_led_off(1);
	mos_thread_sleep(SLEEPTIME);
	mos_led_on(1);
	mos_led_off(0);
	state = READYNODE;
	printf ("locking mutex 2\n");
	mos_mutex_unlock(&state_mutex);

    }
}

void wind_timer_callback()
{
    mos_led_toggle(2);
    wind_speed = wind_ticks;
    wind_ticks = 0;
 
}
