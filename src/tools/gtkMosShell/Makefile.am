## Process this file with automake to produce Makefile.in

bin_PROGRAMS = gmosshell

gmosshell_SOURCES = \
	gtkmosshell.c \
	support.c \
	callbacks.c 


gmosshell_CFLAGS = -ggdb -pg -Wall `pkg-config gtk+-2.0 gthread-2.0 libxml-2.0 \
		                  libglade-2.0 libgnomecanvas-2.0 --cflags` \
	-I$(top_srcdir)/src/mos/share/include \
	-I$(top_srcdir)/src/mos/kernel/linux/include \
	-I$(top_srcdir)/src/tools/gateway/include \
	-I$(top_srcdir)/src/mos/com/include \
	-I$(top_srcdir)/src/mos/net/include \
	-I$(top_srcdir)/src/mos/sys/include \
	-I$(top_srcdir)/src/tools \
	-I$(top_srcdir)/src/tools/shell \
	-I$(top_srcdir)/src/tools/gtkMosShell/include 

gmosshell_LDFLAGS = -ggdb -pg `pkg-config gtk+-2.0 gthread-2.0 libglade-2.0 \
                  libxml-2.0 libgnomecanvas-2.0 --libs` -lpopt


LDADD = \
  $(top_builddir)/src/tools/shell/lib_shell.a \
  $(top_builddir)/src/mos/sys/lib_sys.a \
  $(top_builddir)/src/mos/com/lib_com.a \
  $(top_builddir)/src/mos/dev/lib_dev.a \
  $(top_builddir)/src/mos/kernel/linux/lib_kernel.a \
  $(top_builddir)/src/mos/share/lib_share.a \
  $(top_builddir)/src/lib/lib.a \
  -lpthread 

all:
	cp -fr $(top_srcdir)/src/tools/gtkMosShell/glade .

install:
	/bin/sh ../../../../../mkinstalldirs /usr/local/bin
	/usr/bin/install -c gmosshell /usr/local/bin/gmosshell
	mkdir -p /usr/local/share/gmosshell
	cp -fr $(top_srcdir)/src/tools/gtkMosShell/glade /usr/local/share/gmosshell
	cp -fr $(top_srcdir)/src/tools/shell/boot.srec /usr/local/share/mos

#endif

#include ../../Makefile.am.inc

