#include <inttypes.h>
#include "mos.h"
#include "led.h"
#include "dev.h"
#include "com.h"
#include "msched.h"
#include "printf.h"
#include "clock.h"
#include "adc.h"
#include "avr-adc.h"
#include "mica2-light-temp.h"
#include "fire.h"
#include "cc1000.h"
#include "sem.h"

static mos_alarm_t send_timer;
uint32_t send_timer_ms = 15; //old 4000
void send_timer_callback();

void timer()
{
    mos_alarm (&send_timer);
}

void start (void)
{

    //set timer functions
    send_timer.func = send_timer_callback;
    send_timer.msecs = send_timer_ms;
    send_timer.reset_to = 0;

    //start sleep alarm
    mos_alarm (&send_timer);


}