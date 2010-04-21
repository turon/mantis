//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003,2004,2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

/**************************************************************************/
/* For simple testing of the microblaze port                              */
/**************************************************************************/

//main include
#include "mos.h"

//system includes
#include "led.h"
#include "dev.h"
#include "sem.h"
//#include "com.h"
#include "printf.h"

//includes for this app
#include "fix_fft.h"
#include "show_amp.h"

#include "command_daemon.h"

#include "clock.h"

#include "mb-timer.h"

#define USE_FFT

mos_sem_t sem_a;
mos_sem_t sem_b;

static uint8_t mic_continue;

static volatile bool array_full = false;
static volatile unsigned int fill_index = 0;
static volatile unsigned int* samples     = (int*) 0x00032000;
static volatile unsigned int* samples_out = (int*) 0x00036000;


//int16_t in[128];           /* samples array */
void fill_array()
{
    fill_index = 0;
    array_full = false;
    mb_timer_start();

    volatile unsigned int timer_value;
    while(!array_full)
        timer_value = mb_timer_get_value();

    mb_timer_stop();
    unsigned int i;
}


void show(uint8_t val)
{
    uint8_t led_disp = 0;
    if(val > 20) led_disp |= (1 << 0);
    if(val > 40) led_disp |= (1 << 1);
    if(val > 60) led_disp |= (1 << 2);
    if(val > 80) led_disp |= (1 << 3);
    mos_led_display(led_disp);
}

void read_and_show(void)
{
    uint16_t val;
    dev_read(DEV_ADC, &val, 2);
    {

        int16_t uval = val - 417; 
        if(uval < 0) uval *= -1;
        show(uval);
    }
}

void mic_test(void)
{
    while(mic_continue)
    {
        //        printf("%sMic!!%s\r\n", COLOR_BLUE, COLOR_NONE);
        read_and_show();
    }
}

void show_thread_err()
{
    printf("error creating thread\r\n");
}

void start_mic_test()
{
    mic_continue = true;
    if(mos_thread_new (mic_test, 1024, PRIORITY_NORMAL) != THREAD_OK)
        show_thread_err();
}
void stop_mic_test() { mic_continue = false; }


void timer_fill_array()
{
    if(!array_full)
    {
        int16_t val;
        dev_read(DEV_ADC, &val, 2);
        val -= 417;
        samples[fill_index]  = val;
        //samples[fill_index] &= 0x3FF;
        //        samples[fill_index] -= 417;
        if(++fill_index == 64)
            array_full = true;
    }
}

#if defined USE_FFT
#define NUM_BUFFS 1
#define BUFF_SIZE 64
void do_fft()
{

    comBuf send_pkt;
    uint8_t cnt;
    unsigned int i;

    mb_timer_register_callback(timer_fill_array);
    mb_timer_set_reset(0xFF800000);
    //mb_timer_set_reset(0xFA800000);
    //mb_timer_set_reset(0xFd400000);
    mb_timer_stop();
    //    ENABLE_INTS();

    for(i=0;i<64;i++)
    {
        samples[i] = 0;
        send_pkt.data[i] = 0;
    }

    send_pkt.size = 64;

    int16_t *vals = send_pkt.data;
    unsigned int *data = samples_out;
    for(i=0;i<32;i++)
    {
        vals[i] = 0x00;
    }

    while(1)
    {


        //        mos_clear_ext_timer();
        fill_array();
        //        uint16_t val;
        //        dev_read(DEV_ADC, &val, 2);
        //        samples[0]  = val;

                for(i=0;i<64;i++)
                {
                    samples_out[i] = 0xFF;
                }

        //        mos_show_ext_timer();

        // Initialize the FFT opb
        //setup fft_opb with (start/end/sample count)
        XIo_Out32(XPAR_FFT_OPB_0_BASEADDR + 0x0, (int)samples);
        XIo_Out32(XPAR_FFT_OPB_0_BASEADDR + 0x4, (int)samples_out);
        XIo_Out32(XPAR_FFT_OPB_0_BASEADDR + 0x104, NUM_BUFFS * BUFF_SIZE);

        //let it run (go)
        XIo_Out32(XPAR_FFT_OPB_0_BASEADDR + 0x108, 0xDEADBEEF);

        //       mos_mdelay(1);
        mos_udelay(50);


        //check that it finished
        //while (XIo_In32(XPAR_FFT_OPB_0_BASEADDR) == 0){}
        //       for(i=0; i < 64; i++)
        //       {
        //           data[i] &= 0x3FF;
        //           if(data[i] > 512)
        //               data[i] = 1024 - data[i];
        //       }

        //       data = samples;
        //read back samples from memory

        for(i=0;i<32;i++)
        {
            //make them the same sine for averaging
            //            if((data[i * 2] < 0 && data[i * 2 + 1] > 0) ||
            //               (data[i * 2] > 0 && data[i * 2 + 1] < 0))
            //                data[i * 2] *= -1;
            //            vals[i] = (data[i * 2] + data[i * 2 + 1]) / 2;
            vals[i] = samples_out[i];
            //            vals[i] = 0x7303;
        }
        //        mos_mdelay(200);

//        vals[0] = samples_out[1];
        com_send(IFACE_SERIAL, &send_pkt);
        //mos_mdelay(1000);
    }
}
#endif
void start(void)
{
    mb_init_timer();
    //    printf("start\r\n");
    do_fft();
    //    mos_sem_init(&sem_a, 1);
    //    mos_sem_init(&sem_b, 0);
    /*
       if(mos_thread_new (mos_command_daemon, 1024, PRIORITY_NORMAL) != THREAD_OK)
       show_thread_err();

       mos_register_function("mic", start_mic_test);
       mos_register_function("blink1", start_blink1);
       mos_register_function("blink2", start_blink2);
       mos_register_function("generator", start_generator);
       mos_register_function("receiver", start_receiver);
#if defined USE_FFT
mos_register_function("fft", do_fft);
#endif
mos_register_function("fft_test", do_fft_test);

    //and for turning those threads off
    mos_register_function("mic_off", stop_mic_test);
    mos_register_function("blink1_off", stop_blink1);
    mos_register_function("blink2_off", stop_blink2);
    mos_register_function("generator_off", stop_generator);
    mos_register_function("receiver_off", stop_receiver);
    */

    //    start_generator();
    //   start_receiver();
}


