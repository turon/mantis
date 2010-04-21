//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/types.h>
#include <readline/readline.h>
#include <readline/history.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <getopt.h>
#include "serial.h"
#include "shell_serial.h"
#include "flash.h"
#include "load.h"
#include "mcc.h"
#include "mcs.h"
#include "arg.h"
#include "mutex.h"
/* following added for command server */
#include "com.h"
#include "msched.h"
#include "command_list.h"

#include "shell.h"

extern int go;
extern int program;
extern char *srec;
extern int clear_cb;

void shell_main();
void quit(void);
void check_args (int argc, char *argv[]); //parse command line args
void usage (char *name);
void mos_terminal();
void reprogram(void);
void verify(void);
void reload();

static void set_mos();
static void serial_off();
static void serial_on();
static bool search_for_node = TRUE;
static void load_srec_file(char *filename);

static struct option long_options[]= {
    {"sdev", 1, NULL, 0},
    {"sbaud", 1, NULL, 0},
    {0, 0, NULL, 0}
};


#define LOCAL_COMMAND_COUNT_MAX 30
command_list_t local_commands[LOCAL_COMMAND_COUNT_MAX];

int logging = 0;
FILE* logfile = NULL;

void start (void)
{
    mos_thread_new(shell_main, 128, PRIORITY_NORMAL);
}

void reload()
{
    load_srec_file(srec);
}

void shell_main()
{
    int argc = mos_arg_argc();
    char **argv = mos_arg_argv();
    comBuf send_pkt;

    fprintf(stderr, "Using sdev %s, sbuad ", DEFAULT_SERIAL_DEV); 
    switch(DEFAULT_BAUDRATE) {
        case B115200:
            fprintf(stderr, "B115200");
            break;
        case B57600:
            fprintf(stderr, "B57600");
            break;
        case B38400:
            fprintf(stderr, "B38400");
            break;
        default:
            fprintf(stderr, "unknown!");
            break;
    }
    fprintf(stderr, "\n");

    com_mode(IFACE_SERIAL, IF_LISTEN);

    local_commands[0].input=NULL;
    register_function(local_commands,"quit\n",quit,
            LOCAL_COMMAND_COUNT_MAX);
    register_function(local_commands,"reload\n",reload,
            LOCAL_COMMAND_COUNT_MAX);
    register_function(local_commands,"reprogram\n",reprogram,
            LOCAL_COMMAND_COUNT_MAX);
    register_function(local_commands,"verify\n",verify,
            LOCAL_COMMAND_COUNT_MAX);
    register_function(local_commands,"set_mos\n",set_mos,
            LOCAL_COMMAND_COUNT_MAX);
    register_function(local_commands,"serial off\n",serial_off,
            LOCAL_COMMAND_COUNT_MAX);
    register_function(local_commands,"serial on\n",serial_on,
            LOCAL_COMMAND_COUNT_MAX);
    fprintf(stderr,"Welcome to the MANTIS shell v0.9!\n");

    check_args (argc, argv);

    serial_flush();

    if (clear_cb) {
        printf ("Sending cb 9\n");
        shell_find_loader (NULL);
        send_pkt.size = 1;
        send_pkt.data[0] = CLEAR_CB;
        com_send (IFACE_SERIAL, &send_pkt);

        check_recv (CLEAR_CB_PONG);
    }

    // Check to see if we are in non-interactive mode
    if(program)
        load_srec_file(srec);
    else {
        if(search_for_node) {
            shell_find_loader(NULL);
            shell_start_execution(NULL);
        }
    }

    mos_terminal();

    exit(0);
}

#include <sys/time.h>

void mos_terminal ()
{
    comBuf *recvd;
    IF_SET set;

    //internal timer to test/debug
    struct timeval oldval;
    struct timeval newval;
    gettimeofday(&oldval, NULL);

    fprintf(stderr,"mos_terminal loaded...\n");

    com_mode(IFACE_TERMINAL, IF_LISTEN);      //add stdin to set
    com_mode(IFACE_SERIAL, IF_LISTEN); //tell the uart we want to listen
    com_mode(IFACE_UDP, IF_LISTEN);
    while(1) {
        IF_ZERO(&set);                     //reset the set of interfaces
        IF_SET(IFACE_TERMINAL, &set);      //add stdin to set
        IF_SET(IFACE_SERIAL, &set);        //add serial to set
        IF_SET(IFACE_UDP, &set);           //become a server
        com_select(&set,COM_BLOCK);            //wait for a device to respond

        if(IF_ISSET(IFACE_SERIAL,&set)) {      //something on serial line
            recvd = com_recv(IFACE_SERIAL);    //retrieve data from serial
            if(recvd->data[0] == 255 &&
                    recvd->data[1] == 255 &&
                    (recvd->size == 3 || // so we can use MOS printf
                     (recvd->size == 4 && recvd->data[3] == 0)))
            {
                switch(recvd->data[2])
                {
                    case 0: //reset the timer
                        // I'm adding case 3 so that MOS apps can use printf to send the
                        // timer reset command.  0 still works, because MOS printf puts 
                        // a 0 on the end of every string, but printf("\xFF\xFF\3")
                        // makes more sense than printf("\xFF\xFF\0").
                    case 3:
                        gettimeofday(&oldval, NULL);
                        printf("Resetting timer at %d.%06d\n", (int)oldval.tv_sec,
                                (int)oldval.tv_usec);
                        if (logging) {
                            fprintf(logfile, "Resetting timer at %d.%06d\n",
                                    (int)oldval.tv_sec, (int)oldval.tv_usec);
                            fflush(logfile);
                        }
                        break;
                    case 1: //print the timer
                        gettimeofday(&newval, NULL);
                        newval.tv_sec -= oldval.tv_sec;
                        newval.tv_usec -= oldval.tv_usec;
                        if(newval.tv_usec < 0) {
                            newval.tv_sec--;
                            newval.tv_usec += 1000000;
                        }
                        printf("%d.%06d seconds passed\n", (int)newval.tv_sec,
                                (int)newval.tv_usec);
                        if (logging) {
                            fprintf(logfile, "%d.%06d seconds passed\n",
                                    (int)newval.tv_sec, (int)newval.tv_usec);
                            fflush(logfile);
                        }
                        break;
                    case 2: //print and reset the timer
                        gettimeofday(&newval, NULL);
                        newval.tv_sec -= oldval.tv_sec;
                        newval.tv_usec -= oldval.tv_usec;
                        gettimeofday(&oldval, NULL);
                        if(newval.tv_usec < 0) {
                            newval.tv_sec--;
                            newval.tv_usec += 1000000;
                        }
                        printf("%d.%06d seconds passed (reset)\n",
                                (int)newval.tv_sec, (int)newval.tv_usec);
                        if (logging) {
                            fprintf(logfile, "%d.%06d seconds passed (reset)\n",
                                    (int)newval.tv_sec, (int)newval.tv_usec);
                            fflush(logfile);
                        }
                        break;
                    case 4: // turn on logging
                        if (logfile) logging = 1;
                        break;
                    case 5: // turn off logging
                        logging = 0;
                        break;
                    case 6: // print absolute time
                        gettimeofday(&newval, NULL);
                        printf("Time: %10d.%06d\n", (int)newval.tv_sec,
                                (int)newval.tv_usec);
                        if (logging) {
                            fprintf(logfile, "Time: %10d.%06d\n", (int)newval.tv_sec,
                                    (int)newval.tv_usec);
                            fflush(logfile);
                        }
                        break;
                    default:
                        com_send(IFACE_TERMINAL, recvd);   //send data to terminal
                        if (logging) {
                            fprintf(logfile, "%s", recvd->data);
                            fflush(logfile);
                        }
                }
            } else {
                com_send(IFACE_TERMINAL, recvd);   //send data to terminal
                if (logging) {
                    fprintf(logfile, "%s", recvd->data);
                    fflush(logfile);
                }
            }
            //TODO: forward traffic over the network
            com_free_buf(recvd);             
            fflush (stdout);                 //show the string incase no \n


        } else if(IF_ISSET(IFACE_TERMINAL,&set)){//user typed something
            recvd = com_recv(IFACE_TERMINAL);   //get the data 
            if(!command_parse(local_commands,recvd->data,
                        LOCAL_COMMAND_COUNT_MAX)){//local command?
                shell_send_to_node(recvd->data); //strip the newline
            }
            com_free_buf(recvd);
        } else if(IF_ISSET(IFACE_UDP, &set)){//something from the network
            recvd = com_recv(IFACE_UDP);
            fprintf(stderr,"\n nework data:");
            com_send(IFACE_TERMINAL,recvd);
            shell_send_to_node(recvd->data); //strip the newline
            com_free_buf(recvd);
        } else {
            usleep(10000);
        }
    }
}

void quit(void)
{
    exit(0);
}

void reprogram(void)
{
    if(srec != NULL) {
        fprintf(stderr,"reprogramming [%s] onto sensor node\n",srec);
        shell_send_to_node("DR");
        if(check_recv(ACK) != -1) {
            fprintf(stderr,"dynamic reprogramming available.\n");
            shell_load_image(srec, NULL);
        } else
            fprintf(stderr,"dr not available.\n");
    }
    else
        fprintf(stderr,"no srec provided.\n");

    return;
}

void verify(void)
{
    if(srec != NULL) {
        fprintf(stderr,"verify [%s] was reprogrammed on sensor node\n",srec);
        shell_send_to_node("VDR");
        if(check_recv(ACK) != -1) {
            fprintf(stderr,"verification available.\n");
            shell_load_image(srec, NULL);
        } else
            fprintf(stderr,"verification not available.\n");
    }
    else
        fprintf(stderr,"no srec provided.\n");

    return;
}

void usage (char *name)
{
    fprintf (stderr,"%s: Usage:\n", name);
    fprintf (stderr,"  --sdev <device>  Name of serial port to open\n");
    fprintf (stderr,"  -n              Drop directly into shell, do not"
            "look for\n");
    fprintf (stderr,"                  hardware on the serial port.\n");
    fprintf (stderr,"  -p <srec_file>  Name of srecord to download"
            "onto board.\n");
    fprintf (stderr,"  -D              Dump the current settings to stdout.\n");
    fprintf (stderr,"\n");
}

void check_args (int argc, char *argv[])
{

    int c;

    int option_index = 0;
    /* flag for erasing boot control block */
    while ((c = getopt_long (argc, argv, "DBnd:p:r:l:", long_options,
                    &option_index)) != -1) {
        switch (c) {
            // No default connection
            case 'n':
                search_for_node=FALSE;
                break;

                // Specify the serial device to use
            case 'd': //TODO ... use ioctl to change serial device
                //ret = serial_open(optarg);
                break;
            case 'r':
                srec = malloc(strlen(optarg) + 1);
                strcpy(srec,optarg);
                fprintf (stderr,"optarg is '%s'\n", srec);
                break;
                // Non-interactive programming mode 
            case 'p':
                program = 1;
                srec = malloc(strlen(optarg) + 1);
                strcpy(srec,optarg);
                break;
            case 'B':
                printf ("Loading a new boot control block\n");
                clear_cb = 1;
                break;
            case 'l':
                printf("Logging to file %s\n", optarg);
                logfile = fopen(optarg, "a");
                break;
            case '?':
                if(isprint(optopt))
                    fprintf(stderr, "Unknown option `-%c'.\n", optopt);
                else
                    fprintf(stderr,"Unknown option character `\\x%x'.\n",optopt);

                usage(argv[0]);
                exit(1);

            default:
                //fprintf(stderr,"Default...\n");
                break;
        }      
    }
}

void load_srec_file(char *filename)
{
    int ret_val;
    shell_find_loader(NULL);

    while((ret_val=shell_load_image(srec, NULL)) != SUCCESS) {
        switch(ret_val) {
            case SREC_READ_ERROR:
                fprintf(stderr,"Error reading SREC file!\n");
                return;
                break;
            case LOAD_COMMAND_FAILURE:
                fprintf(stderr,"Node didn't accept \"LOAD\" command\n");
                break;
            case PAGE_PACKET_FAILURE:
                fprintf(stderr,"Page packet failed. \n");
                break;
            default:
                fprintf(stderr,"Unknown return code\n");
                break;
        }
        shell_find_loader(NULL);
    }
    ret_val = shell_start_execution(NULL);
}

static void set_mos()
{
    shell_load_bootloader();
}

static void serial_off()
{
    fprintf(stderr,"turning off serial\n");
    com_mode(IFACE_SERIAL,IF_OFF);
}

static void serial_on()
{
    fprintf(stderr,"turning on serial\n");
    com_mode(IFACE_SERIAL, IF_LISTEN);
}
