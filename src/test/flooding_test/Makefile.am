BUILT_APPS = flood.elf flood_recv.elf
flood_elf_SOURCES = flooding_test.c
flood_recv_elf_SOURCES = flooding_test_recv.c
if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

