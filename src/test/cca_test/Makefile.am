BUILT_APPS = generator.elf cca_test.elf cca_test_bmac.elf
cca_test_elf_SOURCES = cca_test.c
cca_test_bmac_elf_SOURCES = cca_test_bmac.c
generator_elf_SOURCES = generator.c

if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

