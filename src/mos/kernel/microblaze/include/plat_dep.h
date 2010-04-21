#ifndef __PLAT_DEP_H__
#define __PLAT_DEP_H__
#include "mos.h"
#include "plat_clock.h"

typedef uint32_t stackval_t;

//32bit address space
typedef stackval_t memtype_t;

typedef uint32_t handle_t;
extern stackval_t __stack;

typedef uint32_t register_type_t;
register register_type_t zero_register asm("r0");
register register_type_t stack_pointer_register asm("r1");
register register_type_t return_address_interrupt asm("r14");
register register_type_t return_address_subroutine asm("r15");
//register register_type_t program_counter asm("pc");
//register register_type_t status_register asm("rmsr");

#define CC2420_FIFOP_INTERRUPT() mos_cc2420_fifop_interrupt_handler()

void mos_cc2420_fifop_interrupt_handler();

#define _end __stack

#define ALARM_TIMER_EXPIRED 1
#define ALARM_INT_HEADER void ostimer_fired


// place holders ////////////////////
#define SLEEP_INT_HEADER void placeholder__
#define SLEEP_TIMER_EXPIRED TRUE

#define DISABLE_ALARM_TIMER() 


#define ENABLE_ALARM_TIMER() do{ \
    ; \
}while(0)

#define MIN_STACK_SIZE 1024

#define DISABLE_INTS()
#define ENABLE_IDLE()
#define IDLE()
#define SET_SLEEP_TIMER_VALUE(value) 
#define SET_ALARM_TIMER_VALUE(value) 
#define SET_SLEEP_OCR_VALUE(value)
#define ENABLE_SLEEP_TIMER()
#define DISABLE_SLEEP_TIMER()
#define PRE_KERNEL_SLEEP()
#define SLEEP()
#define POST_KERNEL_SLEEP()
#define ENABLE_SLEEP()
#define GET_SP(sp) do{ \
    register register_type_t current_stack_pointer asm("r1"); \
    sp = current_stack_pointer; \
}while(0)

#define GET_MSR()  mfmsr();
#define SET_MSR(handle) mtmsr(handle);
/*
#define MB_RESET() do{   \
    program_counter = 0; \
    mtmsr(0);            \

} while(0)
*/
//#define SET_MSR(val) { asm volatile ( "mts\trmsr,%0\n" :: "d"(val)); }

///////////////////////////////////

#define ARCH_PROGMEM 
#define WAIT_FOR_ASYNC_UPDATE()

#define MEMORY_BASE 0x00
#define MEMORY_SIZE 0xffff

#define SR_IF_MASK 0x0

void mos_enable_ints(handle_t int_handle);

/** @brief Initialize the hardware timer
 * This timer is used for the kernel time slice.
 */
void kernel_timer_init(void);

/** @brief initialize the timer used for sleeping
 *
 */
void sleep_timer_init(void);

/** @brief enables interrupts
 *
 */

/** @brief disables interrupts
 *
 */
handle_t mos_disable_ints(void);

#define ENABLE_INTS() do { \
    microblaze_enable_interrupts(); \
}while(0)
  
/** @brief disables interrupts
 *
 */


#define PIN_SET(regbit)  ()
#define PIN_CLR(regbit)  ()
#define PIN_READ(regbit) ()

#define PIN_OUT(regbit) do{          \
}while(0)


#define PIN_INP(regbit) do{             \
}while(0)

#endif
