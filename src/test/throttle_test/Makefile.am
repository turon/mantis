BUILT_APPS = throttle_send.elf throttle_recv.elf
throttle_send_elf_SOURCES = generator.c
throttle_recv_elf_SOURCES = throttle.c

if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

