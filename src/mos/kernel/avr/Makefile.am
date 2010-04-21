noinst_LIBRARIES = lib_kernel.a
lib_kernel_a_SOURCES = realtime.c plat_dep.c

#Gonna need a platform here
AM_CPPFLAGS = \
  -I$(top_srcdir)/src/mos/sys/include \
  -I$(top_srcdir)/src/mos/share/include \
  -I$(top_srcdir)/src/tools/shell \
  -I$(top_srcdir)/src/mos/share/include \
  -I$(top_srcdir)/src/mos/kernel/include \
  -I$(top_srcdir)/src/mos/kernel/avr/include \
  -I$(top_srcdir)/src/mos/kernel/micro/include \
  -I$(top_srcdir)/src/mos/kernel/avr/boot/include \
  -I$(top_srcdir)/src/lib/include \
  -I$(top_srcdir)/src/mos/com/include \
  -I$(top_srcdir)/src/mos/dev/include 

SUBDIRS = boot
