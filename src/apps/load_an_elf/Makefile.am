DEFS+=-DUSING_ELF

if ARCH_LINUX # Linux rules

BUILT_APPS = load_static.elf load_net.elf send_static.elf
load_static_elf_SOURCES = symbols.c prog2load_buf.c load_static.c loader/elfloader.c loader/elfloader-x86.c loader/symtab.c elfstore/elfstore-ram.c
load_net_elf_SOURCES = symbols.c remake_com.c load_net.c loader/elfloader.c loader/elfloader-x86.c loader/symtab.c elfstore/elfstore-ram.c
send_static_elf_SOURCES = symbols.c prog2load_buf.c send_static.c loader/elfloader.c loader/elfloader-x86.c loader/symtab.c elfstore/elfstore-ram.c

.PHONY: symbols.c prog2load_buf.c prog2load_buf.h remake_com.c
symbols.c:
	@echo Generating symbols....
	generate_symbols $(CORE)
prog2load_buf.c: prog2load_buf.h prog2load.o
	@echo Generating static ELF source code...
	strip --strip-unneeded prog2load.o
	binary2buffer prog2load.o > prog2load_buf.c
prog2load_buf.h: prog2load.o
	strip --strip-unneeded prog2load.o
	binary2buffer_h prog2load.o > prog2load_buf.h

endif # End of Linux rules


if ARCH_MSP430 # Telos rules

BUILT_APPS = load_static.elf load_net.elf
load_static_elf_SOURCES = symbols.c prog2load_buf.c load_static.c loader/elfloader.c loader/elfloader-msp430.c loader/symtab.c elfstore/elfstore-ram.c dev/flash.c
load_net_elf_SOURCES = symbols.c remake_com.c load_net.c loader/elfloader.c loader/elfloader-msp430.c loader/symtab.c elfstore/elfstore-ram.c dev/flash.c
send_static_elf_SOURCES = symbols.c prog2load_buf.c send_static.c loader/elfloader.c loader/elfloader-msp430.c loader/symtab.c elfstore/elfstore-ram.c dev/flash.c

.PHONY: symbols.c prog2load_buf.c prog2load_buf.h remake_com.c
symbols.c:
	@echo Generating symbols....
	generate_symbols $(CORE)
prog2load_buf.c: prog2load_buf.h prog2load.o
	@echo Generating static ELF source code...
	msp430-strip --strip-unneeded prog2load.o
	binary2buffer prog2load.o > prog2load_buf.c
prog2load_buf.h: prog2load.o
	msp430-strip --strip-unneeded prog2load.o
	binary2buffer_h prog2load.o > prog2load_buf.h

endif # End of Telos rules

## Remaking com system with USING_ELF defined
remake_com.c:
	(cd ../../../src/mos/com && make clean && make DEFS=-DUSING_ELF)
	touch remake_com.c

include ../../Makefile.am.inc
