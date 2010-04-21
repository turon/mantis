BUILT_APPS = sense_forward_send.elf sense_forward_recv.elf
sense_forward_send_elf_SOURCES = sense_and_forward.c
sense_forward_recv_elf_SOURCES = receiver.c
if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

