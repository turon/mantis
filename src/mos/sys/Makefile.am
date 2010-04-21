noinst_LIBRARIES = lib_sys.a
lib_sys_a_SOURCES = led.c main.c

if ARCH_AVR
lib_sys_a_SOURCES += avr_init.c
endif

if ARCH_MSP430
lib_sys_a_SOURCES += msp430_init.c
endif

if PLATFORM_MICROBLAZE
lib_sys_a_SOURCES += mb_led.c microblaze_init.c
endif

#if ARCH_PXA27X
#lib_sys_a_SOURCES += pxa27x_init.c
#endif

include ../../Makefile.lib.inc


