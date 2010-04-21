BUILT_APPS = dvdrp.elf dvdrprecv.elf dvdrpbounce.elf align.elf
dvdrp_elf_SOURCES = dvdrp_test.c
dvdrprecv_elf_SOURCES = dvdrp_test_recv.c
dvdrpbounce_elf_SOURCES = dvdrp_midd.c
align_elf_SOURCES = align.c
if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

