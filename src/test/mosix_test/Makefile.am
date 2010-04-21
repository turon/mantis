
if PLATFORM_MICA2
BUILT_APPS = mosix_test.elf
mosix_test_elf_SOURCES = mosix_test.c
endif

if PLATFORM_MICAZ
BUILT_APPS = mosix_test.elf
mosix_test_elf_SOURCES = mosix_test.c
endif

if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

