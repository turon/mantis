
noinst_LIBRARIES = lib_com.a
lib_com_a_SOURCES = com.c loopback.c

if ARCH_LINUX
lib_com_a_SOURCES += serial.c terminal.c udp.c xmos_radio.c
endif

if ARCH_AVR
lib_com_a_SOURCES += uart.c
endif

if ARCH_MSP430
lib_com_a_SOURCES += uart.c
endif

if PLATFORM_MICROBLAZE
lib_com_a_SOURCES += mb_uart.c cc2420.c mb-cc2420-gpio.c
endif


if PLATFORM_MICA2
lib_com_a_SOURCES += cc1000.c cc1000_raw.c cc1000_csma.c cc1000_csma_ack.c \
                     cc1000_tdma.c cc1000_bmac.c
endif

if PLATFORM_MICAZ
lib_com_a_SOURCES += cc2420.c cc2420-gpio.c
endif

if PLATFORM_TELOSB
lib_com_a_SOURCES += cc2420.c cc2420-gpio.c
endif

if PLATFORM_AVRDEV
lib_com_a_SOURCES += maxstream.c
endif

if PLATFORM_MICA2DOT
lib_com_a_SOURCES += cc1000.c cc1000_raw.c cc1000_csma.c cc1000_csma_ack.c \
                     cc1000_tdma.c cc1000_bmac.c
endif

include ../../Makefile.lib.inc
