BUILT_APPS = weatherboard.elf wb_accel.elf wb_light.elf
wb_light_elf_SOURCES = wb_light.c
wb_accel_elf_SOURCES = wb_accel.c
weatherboard_elf_SOURCES = weatherboard_test.c
if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

