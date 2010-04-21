if ARCH_AVR

BUILT_APPS = tinyblink.elf blink0.elf blink1.elf blink2.elf deluge_setup.elf radio_test.elf

tinyblink_elf_SOURCES = tinyblink.c
blink0_elf_SOURCES = blink0.c
blink1_elf_SOURCES = blink1.c
blink2_elf_SOURCES = blink2.c
deluge_setup_elf_SOURCES = deluge_setup.c
radio_test_elf_SOURCES = radio_test.c

endif

if ARCH_LINUX

BUILT_APPS = aqshell

aqshell_SOURCES = aqshell.c

endif

include ../../Makefile.am.inc
