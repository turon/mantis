
#if PLATFORM_LINUX
noinst_PROGRAMS = logview
logview_SOURCES = logview.c
logview_CFLAGS = -ggdb -Wall `pkg-config gtk+-2.0 libxml-2.0 gnet-2.0 \
		             libglade-2.0 libgnomecanvas-2.0 --cflags` \
	-I$(top_srcdir)/src/tools/logview/include \
	-I$(top_srcdir)/src/mos/sys/include \
	-I$(top_srcdir)/src/tools
logview_LDFLAGS = -ggdb `pkg-config gtk+-2.0 gnet-2.0 libglade-2.0 \
                  libxml-2.0 libgnomecanvas-2.0 --libs` -lpopt -lgtkdatabox

all:
	cp -fr $(top_srcdir)/src/tools/logview/glade .
#endif

#include ../../Makefile.am.inc

