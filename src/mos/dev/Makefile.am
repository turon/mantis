
noinst_LIBRARIES = lib_dev.a
lib_dev_a_SOURCES = dev.c loopback.c

if ARCH_LINUX
lib_dev_a_SOURCES += xmos-flash.c
endif

if ARCH_MICRO
lib_dev_a_SOURCES += hardware-id.c
endif

if PLATFORM_MICA_ANY
lib_dev_a_SOURCES += \
	            atmel-flash.c \
	            simple_fs.c \
		    mica2-light-temp.c \
	            mica2-mic.c \
	            mica2-sounder.c \
	            mica2-accel.c \
	            mica2-magnet.c \
	            avr-rssi.c \
	            mica2-battery.c \
	            mica2-ultrasound.c \
		mica2-gps.c
endif

if PLATFORM_AVRDEV
lib_dev_a_SOURCES += \
	simple_fs.c \
	loopback.c
endif

if ARCH_AVR
lib_dev_a_SOURCES += adc.c spi.c
lib_dev_a_SOURCES += avr-adc.c \
                     avr-eeprom.c \
                     avr-i2c.c
endif

if ARCH_MSP430
lib_dev_a_SOURCES += adc.c spi.c
lib_dev_a_SOURCES += msp-adc.c \
		     msp-humidity.c \
		     telos-flash.c \
		     msp-flash.c \
		     msp430-i2c.c \
		     st_LIS3L.c
endif

if PLATFORM_MICROBLAZE
lib_dev_a_SOURCES += mb-adc.c \
					 mb-spi.c \
					 mb-adc-parallel.c \
					 mb-dipsw.c
endif

include ../../Makefile.lib.inc
