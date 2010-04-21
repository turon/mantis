
#if PLATFORM_LINUX

SUBDIRS = libbionet

noinst_PROGRAMS = cortex
cortex_SOURCES = cortex.c \
		 filter_wizard.c \
		 filter_model.c \
		 menu_handlers.c \
		 commanding_view.c \
		 hablist_view.c \
		 nodelist_view.c \
		 generic_view.c \
		 filterlist_view.c \
                 gui_gtk.c \
                 gui_topo.c \
		 bionet-interface.c \
		 model_gtk.c \
                 gui_xml.c
cortex_CFLAGS = -ggdb -Wall -O0 `pkg-config gtk+-2.0 gthread-2.0 libxml-2.0 \
        libgnomecanvas-2.0 gmodule-2.0 libglade-2.0 --cflags glib-2.0` \
	`xml2-config --cflags` \
	-I$(top_srcdir)/src/mos/net/include \
	-I$(top_srcdir)/src/tools/cortex/include \
	-I$(top_srcdir)/src/mos/sys/include \
	-I$(top_srcdir)/src/tools/cortex/libbionet \
	-I$(top_srcdir)/src/tools/cortex/util
	-I$(top_srcdir)/src/tools
cortex_LDFLAGS = -ggdb -O0 `pkg-config gtk+-2.0 glib-2.0 gthread-2.0 libglade-2.0 \
                 gmodule-2.0 libxml-2.0 libgnomecanvas-2.0 --libs` \
		 -L$(top_builddir)/src/tools/cortex/libbionet 

LDADD = -lbionet


all:
#	cp $(top_srcdir)/src/tools/cortex/filter_list.xml .
	cp -fr $(top_srcdir)/src/tools/cortex/glade .
#endif

#include ../../Makefile.am.inc

