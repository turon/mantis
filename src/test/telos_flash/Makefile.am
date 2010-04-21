BUILT_APPS = test_flash.elf 
test_hum_elf_SOURCES = test_flash.c
if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

