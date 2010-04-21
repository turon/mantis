
AM_CPPFLAGS =

if ARCH_MICRO
AM_CPPFLAGS += \
  -I$(top_srcdir)/src/mos/sys/include \
  -I$(top_srcdir)/src/mos/dev/include \
  -I$(top_srcdir)/src/mos/kernel/micro/include \
  -I$(top_srcdir)/src/mos/kernel/include \
  -I$(top_srcdir)/src/mos/share/include \
  -I$(top_srcdir)/src/mos/com/include \
  -I$(top_srcdir)/src/mos/net/include \
  -I$(top_srcdir)/src/lib/include
endif

if ARCH_AVR
AM_CPPFLAGS += -I$(top_srcdir)/src/mos/kernel/avr/include
AM_CPPFLAGS += -I$(top_srcdir)/src/mos/kernel/avr/boot/include
endif

if ARCH_MSP430
AM_CPPFLAGS += -I$(top_srcdir)/src/mos/kernel/msp430/include \
           -I$(top_srcdir)/src/mos/kernel/msp430/boot
endif

if PLATFORM_MICROBLAZE
AM_CPPFLAGS += -I$(top_srcdir)/src/mos/kernel/microblaze/include
AM_CPPFLAGS += -I$(top_srcdir)/../nodeblaze/microblaze_0/include
endif


if ARCH_LINUX
AM_CPPFLAGS += \
  -I$(top_srcdir)/src/mos/sys/include          \
  -I$(top_srcdir)/src/mos/kernel/linux/include \
  -I$(top_srcdir)/src/mos/kernel/include       \
  -I$(top_srcdir)/src/mos/share/include        \
  -I$(top_srcdir)/src/mos/com/include          \
  -I$(top_srcdir)/src/mos/dev/include          \
  -I$(top_srcdir)/src/lib/include
endif

