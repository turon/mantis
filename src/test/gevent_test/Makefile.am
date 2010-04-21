#if ARCH_LINUX
bin_PROGRAMS = client server
client_SOURCES = client.c
server_SOURCES = server.c

INCLUDES = -g `pkg-config glib-2.0 gnet-2.0 --cflags` \
	-I$(top_srcdir)/src/lib/include

LIBS = -g `pkg-config glib-2.0 gnet-2.0 --libs` \
	$(top_builddir)/src/lib/gevent/lib_gevent.a

#endif
