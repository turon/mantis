#ifndef _XILINX_HELP_H_ 
#define _XILINX_HELP_H_

#include "xbasic_types.h"
#include "xintc.h"
#include "xintc_l.h"
#include "mb_interface.h"

void init_ints();

void mb_register_interrupt(Xuint8 device_id_, 
        XInterruptHandler Handler_,
        void *user_data_);
void mb_register_and_enable_interrupt(Xuint8 device_id_, 
         XInterruptHandler handler_,
        void *user_data_);
void mb_enable_interrupt(Xuint8 device_id_);
void mb_disable_interrupt(Xuint8 device_id_);

#endif
