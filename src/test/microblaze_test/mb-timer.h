#ifndef MB_TIMER_H
#define MB_TIMER_H
//timer header
#include "mos.h"

void mb_timer_register_callback(void (*callback_func)(void));
void mb_timer_set_reset(uint32_t reset_value_);
void mb_timer_start();
void mb_timer_stop();
void mb_init_timer();
unsigned int mb_timer_get_value();

#endif
