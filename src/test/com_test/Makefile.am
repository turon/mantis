if ARCH_LINUX
BUILT_APPS = linux_serial terminal udp_recv udp_send remote_control
linux_serial_SOURCES = linux_serial.c
terminal_SOURCES = terminal.c
udp_recv_SOURCES = udp_recv.c
udp_send_SOURCES = udp_send.c
remote_control_SOURCES = remote_control.c
else
BUILT_APPS = relay.elf sensor_send.elf
relay_elf_SOURCES = relay.c
sensor_send_elf_SOURCES = sensor_send.c

endif

NET_LIB = $(NET_LIB_COM)

include ../../Makefile.am.inc

