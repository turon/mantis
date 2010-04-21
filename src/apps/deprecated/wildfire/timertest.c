#include <inttypes.h>

#include "mos.h"
#include "msched.h"  // MANTIS scheduler (gives us start)
#include "led.h"     // LED control
#include "clock.h"
#include "printf.h"
#include "sem.h"

mos_sem_t sem;
mos_alarm_t alarm;

void blink_me(void* unused)
{
	/*
    alarm.reset_to = 1024;
    mos_sem_post(&sem);
    */
}


void timer_thread(void)
{
	/*
    uint8_t i = 0;
    mos_sem_init(&sem, 0);
    
    alarm.kbus = 1024;
    alarm.reset_to = 0;
    alarm.func = blink_me;
    alarm.data = &alarm;
    
    mos_alarm(&alarm);
    
    while(1)
    {
	mos_sem_wait(&sem);
	mos_led_toggle(2);
	mos_show_and_clear_ext_timer();
    }
    */
}


void start(void)
{
    //mos_thread_new(timer_thread, 128, PRIORITY_NORMAL);

    return;
}
