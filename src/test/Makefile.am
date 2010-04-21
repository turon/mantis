
SUBDIRS = semaphore_test

if ARCH_MICRO
SUBDIRS += com_test mosix_test comselect_test \
           uart_test
endif


if ARCH_MSP430
SUBDIRS += radio_test cc2420_test telos_hum telos_flash
endif

if ARCH_AVR
SUBDIRS += eeprom_test mosix_test fft_test radio_test sensor_test \
           rssi_test ext_flash_test id_test realtime_test cca_test cts_test \
	       clock_test sleep_test fec_test performance
           #dvdrp_test throttle_test tdma_test
endif

if PLATFORM_TELOSB
SUBDIRS += asymmetry
endif

if PLATFORM_MICAZ
SUBDIRS += asymmetry
endif

if PLATFORM_MICA_ANY
SUBDIRS += adc_test accel_test weatherboard_test
endif

if ARCH_LINUX
SUBDIRS += crc_test fec_test arith_test comselect_test
endif

if PLATFORM_MICROBLAZE
SUBDIRS += microblaze_test fft_test
endif
