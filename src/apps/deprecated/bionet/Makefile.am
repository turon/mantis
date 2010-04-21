
if ARCH_AVR
BUILT_APPS = bedrest_sense.elf bedrest_relay.elf calibrate.elf bedrest_human_relay.elf radio_flood.elf radio_ack.elf
bedrest_sense_modified_elf_SOURCES = bedrest_sense_modified.c
bedrest_relay_elf_SOURCES = bedrest_relay.c
bedrest_sense_elf_SOURCES = bedrest_sense.c bedrest_config.h bedrest_shared.h
bionet_sense_elf_SOURCES = sense.c
bionet_relay_elf_SOURCES = relay.c
calibrate_elf_SOURCES = calibrate.c
radio_spam_elf_SOURCES = radio_spam.c
net_send_elf_SOURCES = net_send.c
bedrest_human_relay_elf_SOURCES = bedrest_human_relay.c
radio_flood_elf_SOURCES = radio_flood.c
radio_ack_elf_sources = radio_ack.c

else
#BUILT_APPS = bionet_server.elf logger
BUILT_APPS = bionet_server.elf radio_spam.elf
bionet_server_elf_SOURCES = gateway.c
radio_spam_elf_SOURCES = radio_spam.c
#logger_SOURCES = logger.c
#logger_CFLAGS = `pkg-config gnet-2.0 --cflags` 
#logger_LDFLAGS = `pkg-config gnet-2.0 --libs`

endif
include ../../Makefile.am.inc

