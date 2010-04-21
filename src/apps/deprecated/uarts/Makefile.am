BUILT_APPS = uartsender.elf uart2led.elf
uartsender_elf_SOURCES = uartsender.c
uart2led_elf_SOURCES = uart2led.c

if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

