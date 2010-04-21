BUILT_APPS = empty.elf set_id.elf fire_base.elf fire_network.elf timertest.elf test_send.elf test_recv.elf sleeptest.elf
wildfire_send_elf_SOURCES = wildfire_send.c
wildfire_recv_elf_SOURCES = wildfire_recv.c
empty_elf_SOURCES = empty_chairs_at_empty_tables.c
set_id_elf_SOURCES = set_id.c
#windtunnel_send_elf_SOURCES = windtunnel_send.c
#windtunnel_recv_elf_SOURCES = windtunnel_recv.c
fire_network_elf_SOURCES = fire_network.c
fire_base_elf_SOURCES = fire_base.c
timertest_elf_SOURCES = timertest.c
test_send_elf_SOURCES = test_send.c
test_recv_elf_SOURCES = test_recv.c
sleeptest_elf_SOURCES = sleeptest.c

if ARCH_AVR
NET_LIB = $(NET_LIB_RADIOMAC_FLOODING)
else
NET_LIB = $(NET_LIB_GSMAC)
endif

include ../../Makefile.am.inc

