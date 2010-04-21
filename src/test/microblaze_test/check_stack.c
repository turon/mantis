/*
#include "context_switch.h"
void check_stack()
{
mos_thread_t *_current_thread = mos_thread_current();
//Detect stack violations
if(_current_thread->sp && _current_thread->thread_id)
{
stackval_t thread_stack_begin = (stackval_t)_current_thread->stack;
stackval_t thread_stack_size  = (stackval_t)_current_thread->stackSize;
stackval_t thread_stack_end   = thread_stack_begin + thread_stack_size;
stackval_t tid = _current_thread->thread_id;
stackval_t tsp = (stackval_t)_current_thread->sp;

if(tsp < thread_stack_begin || tsp > thread_stack_end)
{
printf("%sThread Stack Violation!!%s\r\n", COLOR_RED, COLOR_NONE);
printf("Thread [%d] Begin: %s%d%s End: %s%d%s SP: %s%d%s\r\n",
tid,
COLOR_YELLOW, thread_stack_begin, COLOR_NONE,
COLOR_YELLOW, thread_stack_end, COLOR_NONE,
COLOR_RED, tsp, COLOR_NONE);
while(1);
}
}
}
*/

