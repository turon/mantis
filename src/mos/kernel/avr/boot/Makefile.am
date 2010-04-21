noinst_PROGRAMS = boot.elf
boot_elf_SOURCES = flash.S boot.c serial.c boot.lcf

boot.elf$(EXEEXT): copyflash.S serial.c boot.c boot.lcf \
		   $(top_srcdir)/src/lib/crc.c  \
		   $(top_srcdir)/src/mos/sys/led.c

	$(CC) $(CFLAGS) $(AM_CPPFLAGS) -o boot.elf$(EXEEXT) $^

.PHONY: load

copyflash.S: $(top_srcdir)/src/mos/kernel/avr/boot/flash.S
	cp $^ $@

load:
	avarice -p -j /dev/ttyS0 -f boot.elf

AM_CPPFLAGS = \
  -I$(top_srcdir)/src/mos/sys/include \
  -I$(top_srcdir)/src/mos/kernel/avr/boot/include \
  -I$(top_srcdir)/src/mos/kernel/micro/include \
  -I$(top_srcdir)/src/mos/kernel/avr/include \
  -I$(top_srcdir)/src/lib/include \
  -I$(top_srcdir)/src/mos/dev/include \
  -I$(top_srcdir)/src/mos/com/include \
  -I$(top_builddir)
