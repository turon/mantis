BUILT_APPS = send.elf sink.elf
send_elf_SOURCES = send.c
sink_elf_SOURCES = sink.c
if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

