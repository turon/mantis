BUILT_APPS = test_hum.elf 
test_hum_elf_SOURCES = test_hum.c
if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

