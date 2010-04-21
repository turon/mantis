BUILT_APPS = generator.elf radio_to_uart.elf receiver.elf gen_and_recv.elf
generator_elf_SOURCES = generator.c generator_app.c
radio_to_uart_elf_SOURCES = radio_to_uart.c
receiver_elf_SOURCES = receiver.c receiver_app.c
gen_and_recv_elf_SOURCES = gen_and_recv.c receiver.c generator.c 
if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

