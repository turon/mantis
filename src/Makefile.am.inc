DEFAULT_LIBS_MICRO = -lgcc $(top_builddir)/src/mos/sys/lib_sys.a \
                     $(top_builddir)/src/mos/com/lib_com.a \
                     $(top_builddir)/src/mos/dev/lib_dev.a \
                     $(top_builddir)/src/mos/net/lib_net.a \
                     $(top_builddir)/src/mos/com/lib_com.a \
                     $(top_builddir)/src/mos/dev/lib_dev.a \
                     $(top_builddir)/src/lib/lib.a \
                     $(top_builddir)/src/mos/share/lib_share.a \
                     $(top_builddir)/src/mos/com/lib_com.a \
                     $(top_builddir)/src/mos/dev/lib_dev.a \
                     $(top_builddir)/src/lib/lib.a



DEFAULT_LIBS_LINUX = $(top_builddir)/src/mos/sys/lib_sys.a \
		     $(top_builddir)/src/mos/dev/lib_dev.a \
	             $(top_builddir)/src/mos/com/lib_com.a \
	             $(top_builddir)/src/mos/kernel/linux/lib_kernel.a \
		     $(top_builddir)/src/mos/share/lib_share.a \
	             $(top_builddir)/src/lib/lib.a $(EXTRA_LIBS)

LDADD = $(DEFAULT_LIBS)

if ARCH_MICRO

all: $(BUILT_APPS) $(BUILT_APPS:.elf=.srec)

clean:
	rm -rf *.srec *.elf *.o

if ARCH_AVR
%.srec: %.elf
	avr-objcopy --output-target=srec $^ $@
endif

if ARCH_MSP430
%.srec: %.elf
	msp430-objcopy --output-target=srec $^ $@
endif

if PLATFORM_MICROBLAZE
%.srec: %.elf
	mb-objcopy --output-target=srec $^ $@
endif

DEFAULT_LIBS = $(DEFAULT_LIBS_MICRO)

INCLUDES = \
  -I$(top_srcdir)/src/mos/kernel/micro/include \
  -I$(top_srcdir)/src/mos/dev/include \
  -I$(top_srcdir)/src/mos/sys/include \
  -I$(top_srcdir)/src/mos/kernel/include \
  -I$(top_srcdir)/src/mos/net/include \
  -I$(top_srcdir)/src/mos/com/include \
  -I$(top_srcdir)/src/lib/include \
  -I$(top_srcdir)/src/mos/share/include

if ARCH_AVR
DEFAULT_LIBS +=  $(top_builddir)/src/mos/kernel/avr/lib_kernel.a   \
		 $(top_builddir)/src/mos/kernel/micro/lib_kernel_generic.a

INCLUDES += -I$(top_srcdir)/src/mos/kernel/avr/include \
            -I$(top_srcdir)/src/mos/kernel/avr/boot/include
endif

if ARCH_MSP430
DEFAULT_LIBS += $(top_builddir)/src/mos/kernel/micro/lib_kernel_generic.a \
		$(top_builddir)/src/mos/kernel/msp430/lib_kernel.a

INCLUDES += -I$(top_srcdir)/src/mos/kernel/msp430/include
endif

if PLATFORM_MICROBLAZE
app_dir = $(PWD)
nodeblaze_dir = $(top_srcdir)/../nodeblaze
platlib_dir = /home/charles/src/lib_rev_1_1

DEFAULT_LIBS += $(top_builddir)/src/mos/kernel/micro/lib_kernel_generic.a
DEFAULT_LIBS += $(top_builddir)/src/mos/kernel/microblaze/lib_kernel.a \
                $(nodeblaze_dir)/microblaze_0/lib/libc.a \
                $(nodeblaze_dir)/microblaze_0/lib/libxil.a
                 
AM_LDFLAGS = -Wl,-T -Wl,$(nodeblaze_dir)/NodeBlaze_linker_script.ld 
INCLUDES += -I$(top_srcdir)/src/mos/kernel/microblaze/include
INCLUDES += -I$(nodeblaze_dir)/microblaze_0/include

kernel:
	make -C $(top_builddir)/src/mos

    #    make -C $(nodeblaze_dir) -f system.make bits

download.cmd: $(BUILT_APPS)
	echo "setMode -bscan" >> $@
	echo "setCable -p auto" >> $@
	echo "identify" >> $@
	echo "assignfile -p 3 -file download.bit" >> $@
	echo "program -p 3" >> $@
	echo "quit" >> $@
	echo "" >> $@

THIS_ELF = $(firstword $(BUILT_APPS))

build: kernel $(BUILT_APPS)

debug: kernel $(BUILT_APPS)
	mb-gdb --command=./gdb_cmd $(THIS_ELF)

load: kernel $(BUILT_APPS) download.cmd
	bitinit $(nodeblaze_dir)/system.mhs -lp $(platlib_dir) -pe microblaze_0 $(THIS_ELF) -bt $(nodeblaze_dir)/implementation/system.bit -bm $(nodeblaze_dir)/implementation/system_bd.bmm -o download.bit
	data2mem -bm $(nodeblaze_dir)/implementation/system_bd -bt $(nodeblaze_dir)/implementation/system.bit  -bd $(THIS_ELF) tag microblaze_0  -o b download.bit
	impact -batch download.cmd

endif

else #ARCH LINUX

DEFAULT_LIBS = $(DEFAULT_LIBS_LINUX)

INCLUDES = \
  -I$(top_srcdir)/src/mos/kernel/linux/include \
  -I$(top_srcdir)/src/mos/dev/include \
  -I$(top_srcdir)/src/mos/sys/include \
  -I$(top_srcdir)/src/mos/kernel/include \
  -I$(top_srcdir)/src/lib/include \
  -I$(top_srcdir)/src/mos/com/include \
  -I$(top_srcdir)/src/mos/share/include \
  -I$(top_srcdir)/src/mos/net/include \
  -I$(top_srcdir)/src/apps/bionet/cortex/include
EXTRA_LIBS = -lpthread

endif

noinst_PROGRAMS = $(BUILT_APPS)
