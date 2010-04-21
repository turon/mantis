bin_PROGRAMS = mos_gateway client_test client_random client_xml_log
mos_gateway_SOURCES = gateway.c
client_test_SOURCES = client-test.c
client_random_SOURCES = client-random.c
client_xml_log_SOURCES = client-xml-log.c

AM_CFLAGS = -Wall -ggdb 

INCLUDES = `pkg-config gnet-2.0 libxml-2.0 --cflags` \
	-I$(top_srcdir)/src/mos/share/include \
	-I$(top_srcdir)/src/mos/kernel/linux/include \
	-I$(top_srcdir)/src/tools/gateway/include \
	-I$(top_srcdir)/src/mos/com/include \
	-I$(top_srcdir)/src/mos/net/include \
	-I$(top_srcdir)/src/mos/sys/include \
	-I$(top_srcdir)/src/tools \
	-I$(top_srcdir)/src/tools/cortex/include

LIBS = `pkg-config gnet-2.0 libxml-2.0 --libs`

# $(top_builddir)/src/mos/sys/lib_sys.a 

LDADD = $(top_builddir)/src/mos/com/lib_com.a \
	$(top_builddir)/src/mos/kernel/linux/lib_kernel.a
