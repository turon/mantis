
#include "microblaze_exceptions_i.h"
#include "xilinx_help.h"
#include "printf.h"

#define INTC_DEVICE_ID XPAR_OPB_INTC_0_DEVICE_ID

static XIntc InterruptController; /* Instance of the Interrupt Controller */

static void init_exceptions();
XExceptionHandler XilinxExceptionHandler(void *user_data);

void init_ints()
{
    XIntc_Initialize(&InterruptController,INTC_DEVICE_ID);
    XIntc_Start(&InterruptController, XIN_REAL_MODE);
    init_exceptions();
}

void init_exceptions()
{
    microblaze_register_exception_handler(XEXC_ID_UNALIGNED_ACCESS, XilinxExceptionHandler, XEXC_ID_UNALIGNED_ACCESS);
    microblaze_register_exception_handler(XEXC_ID_ILLEGAL_OPCODE, XilinxExceptionHandler, XEXC_ID_ILLEGAL_OPCODE);
    microblaze_register_exception_handler(XEXC_ID_IOPB_EXCEPTION, XilinxExceptionHandler, XEXC_ID_IOPB_EXCEPTION);
    microblaze_register_exception_handler(XEXC_ID_DOPB_EXCEPTION, XilinxExceptionHandler, XEXC_ID_DOPB_EXCEPTION);
    microblaze_register_exception_handler(XEXC_ID_DIV_BY_ZERO, XilinxExceptionHandler, XEXC_ID_DIV_BY_ZERO);
    microblaze_register_exception_handler(XEXC_ID_FPU, XilinxExceptionHandler, XEXC_ID_FPU);
}

void mb_register_and_enable_interrupt(Xuint8 interrupt_id_, XInterruptHandler handler_, void *user_data_)
{
    mb_register_interrupt(interrupt_id_, handler_, user_data_);
    mb_enable_interrupt(interrupt_id_);
}

void mb_register_interrupt(Xuint8 interrupt_id_, XInterruptHandler handler_, void *user_data_)
{
    XIntc_Connect(&InterruptController, interrupt_id_, handler_, user_data_);
}

void mb_enable_interrupt(Xuint8 interrupt_id_)
{
    XIntc_Enable(&InterruptController, interrupt_id_);
}

void mb_disable_interrupt(Xuint8 device_id_)
{
    XIntc_Disable(&InterruptController, device_id_);
}

XExceptionHandler XilinxExceptionHandler(void *user_data)
{
    uint8_t i = 0;
    while(i++ < 15)
    {
        mos_led_display(5);
        mos_mdelay(500);
        mos_led_display(0xA);
        mos_mdelay(500);
    }
    return NULL;
}

XInterruptHandler DeviceDriverHandler(void *CallbackRef)
{
    printf("generic\r\n");
    return NULL;
}


