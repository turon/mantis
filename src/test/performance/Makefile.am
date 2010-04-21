BUILT_APPS = thread.elf nothread.elf dev.elf nodev.elf com.elf nocom.elf \
	     thread-spam.elf
com_elf_SOURCES = com.c
nocom_elf_SOURCES = nocom.c
dev_elf_SOURCES = dev.c
nodev_elf_SOURCES = nodev.c
thread_elf_SOURCES = kernel-thread.c
thread_spam_elf_SOURCES = thread-spam.c
nothread_elf_SOURCES = kernel-nothread.c
nothread_elf_LDADD =
if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

