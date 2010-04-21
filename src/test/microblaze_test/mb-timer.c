//timer interface
//
#include "mb-timer.h"
#include "xtmrctr.h"
#include "led.h"


static XTmrCtr TimerCounterInst;
static void (*timer_callback)(void);
static uint32_t reset_value = 0;
static XTmrCtr* mb_timer_counter_ptr = &TimerCounterInst;
static const Xuint16 mb_timer_device_id = XPAR_OPB_TIMER_0_DEVICE_ID;
static const Xuint16 mb_timer_interrupt_id = XPAR_OPB_INTC_0_OPB_TIMER_0_INTERRUPT_INTR;
static const Xuint8 mb_timer_0 = 0;

void mb_timer_counter_handler(void *CallBackRef, Xuint8 mb_timer_0);

void mb_timer_set_reset(uint32_t reset_value_)
{
    reset_value = reset_value_;
    XTmrCtr_SetResetValue(mb_timer_counter_ptr, mb_timer_0, reset_value);
}

void mb_timer_register_callback(void (*callback_func)(void))
{
    timer_callback = callback_func;
}

unsigned int mb_timer_get_value()
{
    return XTmrCtr_GetValue(mb_timer_counter_ptr, mb_timer_0);
}

void mb_timer_start()
{
    XTmrCtr_SetResetValue(mb_timer_counter_ptr, mb_timer_0, reset_value);
    mb_enable_interrupt(mb_timer_interrupt_id);
    XTmrCtr_Start(mb_timer_counter_ptr, mb_timer_0);
}

void mb_timer_stop()
{
    XTmrCtr_Stop(mb_timer_counter_ptr, mb_timer_0);
    mb_disable_interrupt(mb_timer_interrupt_id);
}

void mb_init_timer()
{
    XTmrCtr_Initialize(mb_timer_counter_ptr, mb_timer_device_id);
    XTmrCtr_SetHandler(mb_timer_counter_ptr, mb_timer_counter_handler, mb_timer_counter_ptr);
    XTmrCtr_SetOptions(mb_timer_counter_ptr, mb_timer_0, XTC_INT_MODE_OPTION | XTC_AUTO_RELOAD_OPTION);
    mb_register_interrupt(mb_timer_interrupt_id, mb_timer_counter_handler, mb_timer_counter_ptr);
//    mb_register_interrupt(mb_timer_interrupt_id, (XInterruptHandler)XTmrCtr_InterruptHandler, mb_timer_counter_ptr);
}

//timer callback
void mb_timer_counter_handler(void *CallBackRef, Xuint8 timer_number)
{
    if(timer_callback != NULL)
        timer_callback();
}

