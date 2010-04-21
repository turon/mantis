BUILT_APPS = receiver.elf generator.elf
generator_elf_SOURCES = generator2.c
receiver_elf_SOURCES = receiver2.c

if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif


NET_LIB = $(NET_LIB_COM)

include ../Makefile.am.inc

