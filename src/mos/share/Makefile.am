noinst_LIBRARIES = lib_share.a

AM_CPPFLAGS =
lib_share_a_SOURCES = command_list.c

if ARCH_MICRO
lib_share_a_SOURCES += printf.c command_daemon.c hex_output.c testbed_event.c
endif

if ARCH_AVR
lib_share_a_SOURCES += reprogram_commands.c
endif

include ../../Makefile.lib.inc

