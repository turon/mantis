noinst_LIBRARIES = lib_shell.a
lib_shell_a_SOURCES = shell.c flash.c load.c run.c
bin_PROGRAMS = mos_shell
mos_shell_SOURCES = mos_shell.c
man1_MANS = mos_shell.1

INCLUDES = \
  -I$(top_srcdir)/src/lib/include \
  -I$(top_srcdir)/src/mos/sys/include \
  -I$(top_srcdir)/src/tools/shell \
  -I$(top_srcdir)/src/mos/com/include \
  -I$(top_srcdir)/src/mos/kernel/linux/include \
  -I$(top_srcdir)/src/mos/share/include \
  -I/sw/include

LDADD = \
  $(top_builddir)/src/tools/shell/lib_shell.a \
  $(top_builddir)/src/mos/sys/lib_sys.a \
  $(top_builddir)/src/mos/com/lib_com.a \
  $(top_builddir)/src/mos/dev/lib_dev.a \
  $(top_builddir)/src/mos/kernel/linux/lib_kernel.a \
  $(top_builddir)/src/mos/share/lib_share.a \
  $(top_builddir)/src/lib/lib.a \
  -lpthread 

install:
	/bin/sh ../../../../../mkinstalldirs /usr/local/bin
	install -c mos_shell /usr/local/bin/mos_shell
	mkdir -p /usr/local/share/mos
	cp -fr $(top_srcdir)/src/tools/gtkMosShell/glade /usr/local/share/gmosshell
	cp -fr $(top_srcdir)/src/tools/shell/boot.srec /usr/local/share/mos
