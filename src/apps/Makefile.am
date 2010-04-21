
SUBDIRS = blink_led commander

if ARCH_MICRO
SUBDIRS += blink_led alarm_example big_example commander \
	   led_example mst_proto_example \
	   printf_example sense_and_forward sleep_example
endif


if PLATFORM_MICA_ANY
SUBDIRS += gps_example
endif

if PLATFORM_TELOSB
SUBDIRS += flash_receiver testbed_demo
endif


.PHONY: $(APPS)

$(APPS):
	@echo "Making all in $@"
	@cd $@ && make
