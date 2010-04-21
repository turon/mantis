
if ARCH_MICRO
BUILT_APPS = commander.elf 
commander_elf_SOURCES = commander.c
endif

if ARCH_AVR
BUILT_APPS = commander.elf rf_commander.elf
commander_elf_SOURCES = commander.c mica_functions.c
rf_commander_elf_SOURCES = rf_commander.c
endif
include ../../Makefile.am.inc

