BUILT_APPS = accel_test.elf
mosix_test_elf_SOURCES = accel_test.c

if PLATFORM_MICA2
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

