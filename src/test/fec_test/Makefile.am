BUILT_APPS = fec_test.elf
fec_test_elf_SOURCES = fec_test.c

if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

