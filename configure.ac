AC_INIT([mos],[0.9])
AM_INIT_AUTOMAKE(mos, 0.9)

AC_PREREQ([2.50])

if ! test "$enable_build" = "yes"
then
  AC_ERROR([Please use the configure script from the appropriate build directory], 1)
fi

AM_INIT_AUTOMAKE

AC_PROG_CC
AC_PROG_CXX
AC_PROG_RANLIB
AM_PROG_AS


linux=no

if test "$cross_compiling" = "no"
then
linux=yes
fi

if test "$enable_arm" = "yes"
then
linux=yes
fi

if test "$linux" = "yes"
then
  AC_HEADER_STDC
  #For mox
  AC_CHECK_HEADER([stdint.h])
  #For shell
  AC_CHECK_HEADERS([fcntl.h inttypes.h stdlib.h string.h termios.h unistd.h])
  #For agent
  AC_CHECK_HEADERS([arpa/inet.h inttypes.h stdlib.h sys/socket.h])
  AC_C_CONST
  AC_CHECK_FUNCS([bzero strdup socket])
  AC_CHECK_LIB(pthread,pthread_create,
    [ AC_DEFINE(PTHREAD_NAME,[pthread],[Name of the pthread library to link against])
      AC_DEFINE(HAVE_LIBPTHREAD, 1, [Define if libpthread is available])
      [PTHREAD_LINK="-lpthread"]
      [PTHREAD_DEFS="-DUSE_PTHREADS"]
    ],
    [AC_ERROR([You need to install the pthread development package], 1)])
  #defined above
#
#  AM_PATH_GLIB_2_0(2.0.0,,AC_MSG_ERROR([
#  *** GLib 2.0 is required to build MOS; please make sure you have the GLib
#  *** development headers installed. The latest version of GLib is
#  *** always available at http://www.gtk.org/.]))
#  AM_PATH_GTK_2_0(2.0.0,,AC_MSG_ERROR([
#  *** GTK+ 2.0 is required to build MOS; please make sure you have the GTK+
#  *** development headers installed. The latest version of GTK+ is
#  *** always available at http://www.gtk.org/.]))
#  AM_PATH_GNET_2_0(2.0.0,,AC_MSG_ERROR([
#  *** Gnet 2.0 is required to build MOS; please make sure you have the Gnet
#  *** development headers installed. The latest version of Gnet is
#  *** always available at http://www.gnetlibrary.org/.]))
#  AC_CHECK_LIB(gtkdatabox, gtk_databox_new, , [AC_MSG_ERROR([
#  *** Gtkdatabox is required to build MOS; please make sure you have the 
#  *** Gtkdatabox development headers installed. The latest version of
#  *** Gtkdatabox is always available at http://www.eudoxos.de/gtk/gtkdatabox/.])])

  AM_CONDITIONAL(ARCH_LINUX, true)
  AM_CONDITIONAL(ARCH_AVR, false)
  AM_CONDITIONAL(ARCH_MICRO, false)
  AM_CONDITIONAL(ARCH_MSP430, false)
  AM_CONDITIONAL(PLATFORM_NYMPH, false)
  AM_CONDITIONAL(PLATFORM_MICA2, false)
  AM_CONDITIONAL(PLATFORM_MICAZ, false)
  AM_CONDITIONAL(PLATFORM_MICA_ANY, false)
  AM_CONDITIONAL(PLATFORM_MICA2DOT, false)
  AM_CONDITIONAL(PLATFORM_TELOSB, false)
  AM_CONDITIONAL(PLATFORM_MICROBLAZE, false)
  AM_CONDITIONAL(FLOODING_ROUTING, false)
  AC_DEFINE(PLATFORM_LINUX, 1, [Defined if the platform is Linux])
else
  AM_CONDITIONAL(ARCH_MICRO, true)
  AC_DEFINE(ARCH_MICRO, 1, [Defined if the architecture is a microcontroller])
  AM_CONDITIONAL(ARCH_LINUX, false)
  AM_CONDITIONAL(ARCH_AVR, false)
  AM_CONDITIONAL(ARCH_MSP430, false)  
 
fi

AM_CONDITIONAL(PLATFORM_MICA2, test "$enable_mica2" = yes)
if test "$enable_mica2" = "yes"
then
  AC_DEFINE(PLATFORM_MICA2, 1, [Defined if the platform is mica2])
  AC_DEFINE(PLATFORM_MICA_ANY, 1, [Defined if the platform is any mica])
  AC_DEFINE(FLOODING_ROUTING, 1, [Defined if we are using flooding routing])
  AC_DEFINE(CLOCK_SPEED_7_37, 1, [Defined if the clock is 7.37 MHz])
  AC_DEFINE(ARCH_AVR, 1, [Defined if the architecture is AVR])
  AM_CONDITIONAL(ARCH_AVR, true)
  AM_CONDITIONAL(FLOODING_ROUTING, true)
  AM_CONDITIONAL(PLATFORM_NYMPH, false)
  AM_CONDITIONAL(PLATFORM_MICA2, true)
  AM_CONDITIONAL(PLATFORM_MICA_ANY, true)
  AM_CONDITIONAL(PLATFORM_MICAZ, false)
  AM_CONDITIONAL(PLATFORM_TELOSB, false)
  AM_CONDITIONAL(PLATFORM_MICA2DOT, false)
  AM_CONDITIONAL(PLATFORM_MICROBLAZE, false)
fi

AM_CONDITIONAL(PLATFORM_MICAZ, test "$enable_micaz" = yes)
if test "$enable_micaz" = "yes"
then
  AC_DEFINE(PLATFORM_MICAZ, 1, [Defined if the platform is micaz])
  AC_DEFINE(PLATFORM_MICA_ANY, 1, [Defined if the platform is any mica])
  AC_DEFINE(FLOODING_ROUTING, 1, [Defined if we are using flooding routing])
  AC_DEFINE(CLOCK_SPEED_7_37, 1, [Defined if the clock is 7.37 MHz])
  AC_DEFINE(ARCH_AVR, 1, [Defined if the architecture is AVR])
  AC_DEFINE(PLATFORM_SUPPORTS_CC2420, 1, [Defined if the platform supports the CC2420 Radio])
  AM_CONDITIONAL(ARCH_AVR, true)
  AM_CONDITIONAL(FLOODING_ROUTING, true)
  AM_CONDITIONAL(PLATFORM_NYMPH, false)
  AM_CONDITIONAL(PLATFORM_MICA2, false)
  AM_CONDITIONAL(PLATFORM_MICA_ANY, true)
  AM_CONDITIONAL(PLATFORM_MICAZ, true)
  AM_CONDITIONAL(PLATFORM_TELOSB, false)
  AM_CONDITIONAL(PLATFORM_AVRDEV, false)
  AM_CONDITIONAL(PLATFORM_MICA2DOT, false)
  AM_CONDITIONAL(PLATFORM_MICROBLAZE, false)
fi

AM_CONDITIONAL(PLATFORM_AVRDEV, test "$enable_avrdev" = yes)
if test "$enable_avrdev" = "yes"
then
  AC_DEFINE(PLATFORM_AVRDEV, 1, [Defined if the platform is micaz])
  AC_DEFINE(FLOODING_ROUTING, 1, [Defined if we are using flooding routing])
  AC_DEFINE(CLOCK_SPEED_3_68, 1, [Defined if the clock is 3.68 MHz])
  AC_DEFINE(ARCH_AVR, 1, [Defined if the architecture is AVR])
  AM_CONDITIONAL(ARCH_AVR, true)
  AM_CONDITIONAL(FLOODING_ROUTING, true)
  AM_CONDITIONAL(PLATFORM_NYMPH, false)
  AM_CONDITIONAL(PLATFORM_MICA2, false)
  AM_CONDITIONAL(PLATFORM_MICA_ANY, false)
  AM_CONDITIONAL(PLATFORM_MICAZ, false)
  AM_CONDITIONAL(PLATFORM_TELOSB, false)
  AM_CONDITIONAL(PLATFORM_AVRDEV, true)
  AM_CONDITIONAL(PLATFORM_MICA2DOT, false)
  AM_CONDITIONAL(PLATFORM_MICROBLAZE, false)
fi

AM_CONDITIONAL(PLATFORM_MICA2DOT, test "$enable_mica2dot" = yes)
if test "$enable_mica2dot" = "yes"
then
  AC_DEFINE(PLATFORM_MICA2DOT, 1, [Defined if the platform is micaz])
  AC_DEFINE(PLATFORM_MICA_ANY, 1, [Defined if the platform is any mica])
  AC_DEFINE(FLOODING_ROUTING, 1, [Defined if we are using flooding routing])
  AC_DEFINE(CLOCK_SPEED_4_0, 1, [Defined if the clock is 4.0 MHz])
  AC_DEFINE(ARCH_AVR, 1, [Defined if the architecture is AVR])
  AM_CONDITIONAL(ARCH_AVR, true)
  AM_CONDITIONAL(FLOODING_ROUTING, true)
  AM_CONDITIONAL(PLATFORM_NYMPH, false)
  AM_CONDITIONAL(PLATFORM_MICA2, false)
  AM_CONDITIONAL(PLATFORM_MICA_ANY, true)
  AM_CONDITIONAL(PLATFORM_MICAZ, false)
  AM_CONDITIONAL(PLATFORM_TELOSB, false)
  AM_CONDITIONAL(PLATFORM_AVRDEV, false)
  AM_CONDITIONAL(PLATFORM_MICA2DOT, true)
  AM_CONDITIONAL(PLATFORM_MICROBLAZE, false)
fi

AM_CONDITIONAL(PLATFORM_TELOSB, test "$enable_telosb" = yes)
if test "$enable_telosb" = "yes"
then
  AC_DEFINE(PLATFORM_TELOSB, 1, [Defined if the platform is micaz])
  AC_DEFINE(FLOODING_ROUTING, 1, [Defined if we are using flooding routing])
  AC_DEFINE(CLOCK_SPEED_3_68, 1, [Defined if the clock is 4.0 MHz])
  AC_DEFINE(ARCH_MSP430, 1, [Defined if the architecture is MSP430])
  AC_DEFINE(PLATFORM_SUPPORTS_CC2420, 1, [Defined if the platform supports the CC2420 Radio])
  AM_CONDITIONAL(ARCH_MSP430, true)
  AM_CONDITIONAL(FLOODING_ROUTING, true)
  AM_CONDITIONAL(PLATFORM_NYMPH, false)
  AM_CONDITIONAL(PLATFORM_MICA2, false)
  AM_CONDITIONAL(PLATFORM_MICA_ANY, false)
  AM_CONDITIONAL(PLATFORM_MICAZ, false)
  AM_CONDITIONAL(PLATFORM_TELOSB, true)
  AM_CONDITIONAL(PLATFORM_AVRDEV, false)
  AM_CONDITIONAL(PLATFORM_MICA2DOT, false)
  AM_CONDITIONAL(PLATFORM_MICROBLAZE, false)
fi

AM_CONDITIONAL(PLATFORM_MICROBLAZE, test "$enable_microblaze" = yes)
if test "$enable_microblaze" = "yes"
then
  AC_DEFINE(PLATFORM_MICROBLAZE, 1, [Defined if the platform is microblaze])
  AC_DEFINE(FLOODING_ROUTING, 1, [Defined if we are using flooding routing])
  AC_DEFINE(CLOCK_SPEED_8_0, 1, [Defined if the clock is 4.0 MHz])
  AC_DEFINE(PLATFORM_SUPPORTS_CC2420, 1, [Defined if the platform supports the CC2420 Radio])
  AM_CONDITIONAL(ARCH_MICRO, true)
  AM_CONDITIONAL(FLOODING_ROUTING, true)
  AM_CONDITIONAL(PLATFORM_NYMPH, false)
  AM_CONDITIONAL(PLATFORM_MICA2, false)
  AM_CONDITIONAL(PLATFORM_MICA_ANY, false)
  AM_CONDITIONAL(PLATFORM_MICAZ, false)
  AM_CONDITIONAL(PLATFORM_TELOSB, false)
  AM_CONDITIONAL(PLATFORM_AVRDEV, false)
  AM_CONDITIONAL(PLATFORM_MICA2DOT, false)
  AM_CONDITIONAL(PLATFORM_MICROBLAZE, true)
fi

AC_MSG_CHECKING([if we should default to USB-serial])
ac_usbserial=no
AC_ARG_ENABLE(usbserial,
              AC_HELP_STRING([--enable-usbserial], [default to using your USB-serial converter]),
              [ ac_usbserial=${enableval}])
AC_MSG_RESULT($ac_usbserial)
if test "$ac_usbserial" = yes; then
    AC_DEFINE(USB_SERIAL, 1, [default to using the USB-serial converter])
fi

AM_CONFIG_HEADER([config.h])

AC_CONFIG_FILES([
	  Makefile
	  src/Makefile 
	  src/lib/Makefile
	  src/mos/Makefile
	  src/mos/kernel/Makefile
	  src/mos/kernel/avr/Makefile
	  src/mos/kernel/avr/boot/Makefile
	  src/mos/kernel/msp430/Makefile
	  src/mos/kernel/micro/Makefile
      	  src/mos/kernel/microblaze/Makefile
	  src/mos/kernel/linux/Makefile
	  src/mos/sys/Makefile
	  src/mos/share/Makefile
	  src/mos/net/Makefile
	  src/mos/com/Makefile
	  src/mos/dev/Makefile
	  src/apps/Makefile	
	  src/apps/alarm_example/Makefile
	  src/apps/big_example/Makefile
	  src/apps/blink_led/Makefile
	  src/apps/commander/Makefile
	  src/apps/led_example/Makefile
	  src/apps/mst_proto_example/Makefile
	  src/apps/printf_example/Makefile
	  src/apps/sense_and_forward/Makefile
	  src/apps/sleep_example/Makefile
	  src/apps/testbed_demo/Makefile
	  src/apps/flash_receiver/Makefile
	  src/apps/gps_example/Makefile
	  src/test/Makefile
	  src/test/adc_test/Makefile
	  src/test/arith_test/Makefile
	  src/test/fec_test/Makefile
	  src/test/cca_test/Makefile
	  src/test/cc2420_test/Makefile
	  src/test/com_test/Makefile
	  src/test/comselect_test/Makefile
      	  src/test/microblaze_test/Makefile
	  src/test/crc_test/Makefile
	  src/test/eeprom_test/Makefile
	  src/test/ext_flash_test/Makefile
	  src/test/flooding_test/Makefile
	  src/test/fft_test/Makefile
	  src/test/gps_test/Makefile
	  src/test/id_test/Makefile
	  src/test/mosix_test/Makefile
	  src/test/radio_test/Makefile
	  src/test/realtime_test/Makefile
	  src/test/rssi_test/Makefile
	  src/test/sensor_test/Makefile
	  src/test/semaphore_test/Makefile
	  src/test/sleep_test/Makefile
	  src/test/tdma_test/Makefile
	  src/test/throttle_test/Makefile
	  src/test/clock_test/Makefile
	  src/test/dvdrp_test/Makefile
	  src/test/cts_test/Makefile
	  src/test/weatherboard_test/Makefile
	  src/test/performance/Makefile
	  src/test/accel_test/Makefile
	  src/test/uart_test/Makefile
	  src/test/uart_test_raw/Makefile
	  src/test/telos_hum/Makefile
	  src/test/telos_flash/Makefile
      	  src/test/asymmetry/Makefile
	  src/tools/Makefile
	  src/tools/shell/Makefile
	  src/tools/gtkMosShell/Makefile
	  src/tools/gateway/Makefile
	  src/tools/cortex/libbionet/Makefile
	  src/tools/cortex/Makefile
	  src/tools/logview/Makefile
	  ])

AC_OUTPUT
