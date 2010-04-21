
#ifndef BEDREST_CONFIG_H
#define BEDREST_CONFIG_H

#define BEDREST_PACKET        11
#define BEDREST_RAW_PACKET    12
#define BEDREST_CONFIG_PACKET 13

#define SMOOTHING_NUM_READINGS 6
#define SMOOTHING_DISCARDS 2
#define SAMPLE_DELAY_MS 100 //100 for 10 Hz
#define SAMPLE_N 10

//#define RAW_DATA_MODE //comment out this line to disable

#define CONFIG_PACKET_INTERVAL 50 //this is a multiple of the base frequency, now
#define SOFTWARE_VERSION 0x0
#define THERMISTER_OHMS 9300

#define SHOW_LEDS //comment out to stop the leds



/*____ _____ ____   _
 *|  _ \_   _|  _ \ / \
 *| | | || | | |_) / _ \
 *| |_| || | |  __/ ___ \
 *|____/ |_| |_| /_/   \_\ */



#define DTPA_ENABLED //comment out to disable dtpa

#define MAX_POWER           255
#define INIT_POWER           50
#define MIN_POWER             0
#define HALF_POWER           50
#define POWER_STEP_NOMINAL    5
#define POWER_STEP_INCREASE  10
#define POWER_STEP_DECREASE   5
#define POWER_STEP            1
#define TIMEOUT            10 //this isn't miliseconds, it is ticks.
#define TIME_STEPS           2

#define NACK_THRESHHOLD       3
#define ACK_THRESHHOLD        3
#define MAX_NACKS             5
#define FLOOR_ACKS           30
#define FLOOR_NACKS           2



#endif
