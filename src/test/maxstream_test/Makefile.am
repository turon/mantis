BUILT_APPS = maxstream_test.elf
maxstream_test_elf_SOURCES = maxstream_test.c

if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

