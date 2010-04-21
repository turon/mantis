
#ifndef _CLOCK_H_
#define _CLOCK_H_

#include "timer.h"

// constants/macros/typdefs
typedef struct AbsoluteTimerStructure
{	
        uint16_t mtics; 
	uint8_t hours;
	uint8_t minutes;
	uint8_t seconds;
	uint8_t day;
	uint8_t month;
	uint16_t year;
}ATimer;

// month days constants
static char __attribute__ ((progmem)) MonthDayTable[] = {31,28,31,30,31,30,31,31,30,31,30,31};

void clock_init();
void clock_service();
void clock_set_second(uint8_t sec);
void clock_set_minute(uint8_t min);
void clock_set_hours(uint8_t h);
void clock_set_day(uint8_t d);
void clock_set_month(uint8_t mon);
void clock_set_years(uint16_t y);
void start(void);
ATimer* clock_get_time();

#endif /* _CLOCK_H_ */
