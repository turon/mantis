
#for the sensor node...
if ARCH_AVR
BUILT_APPS = light_send.elf temp_send.elf receiver.elf
temp_send_elf_SOURCES = thermal_send.c
light_send_elf_SOURCES = light_send.c
receiver_elf_SOURCES = receiver.c
else
BUILT_APPS = 
endif

include ../../Makefile.am.inc
