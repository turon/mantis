#include"mos.h"

#ifndef BEDREST_SHARED_H
#define BEDREST_SHARED_H

/** @brief Bedrest Packet Structure. */
typedef struct 
{
    uint16_t battery;
    uint16_t avg_accelx;
    uint16_t std_accelx;
    uint16_t avg_accely;
    uint16_t std_accely;
    uint16_t temp;
    uint8_t txpower;
    uint8_t seqNum;
} bedrest_t;


typedef struct
{
    uint16_t sample_delay;
    uint16_t Xa;
    uint16_t Xb;
    uint16_t Ya;
    uint16_t Yb;
    uint16_t resistor;
    uint16_t svn_id;
    uint8_t smooth_num;
    uint8_t config_interval;
    uint8_t raw_data;
    uint8_t smooth_disc;
    uint8_t UNUSED;
    uint8_t sample_n;
    
}config_t;

//DTPA constants here.

#define TX_POWER_MAXIMUM      0
#define TX_POWER_FIND_FLOOR   1
#define TX_POWER_FLOOR        2
#define TX_POWER_PROBE        3

#endif
