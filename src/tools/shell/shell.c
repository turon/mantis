//  This file is part of MANTIS OS, Operating System
//  See http://mantis.cs.colorado.edu/
//
//  Copyright (C) 2003-2005 University of Colorado, Boulder
//
//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the mos license (see file LICENSE)

#define _GNU_SOURCE

#include <stdio.h>

#include <stdbool.h>

#include <stdlib.h>

#include <inttypes.h>
#include <ctype.h>
#include <string.h>
#include "shell_serial.h"
#include "serial.h"
#include "flash.h"
#include "load.h"
#include "mcc.h"
#include "mcs.h"
#include "run.h"
#include "shell.h"
/* following added for command server */
#include "com.h"

int go = 0;
int program = 0;
int clear_cb =0;
char *srec = NULL;

static char *stripwhite (char *line)
{
    char *p = line;
    while (isspace (*p)) {
        p++;
    }
    return p;
}

void shell_send_to_node(char * string){
    comBuf packet;
    char *cursor=string;
    packet.size=0;
    /* copy the string into a packet */
    while (*cursor && packet.size < COM_DATA_SIZE - 1){
        if(*cursor == '\n') //make sure we don't sent newline
            *cursor++;
        else if(*cursor == ' '){ //break each packet up by spaces
            packet.data[packet.size++]='\0'; //null terminate
            com_send(IFACE_SERIAL, &packet);
            packet.size=0;
            *cursor++;
        }
        else
            packet.data[packet.size++] = *cursor++; //add char, inc size
    }
    packet.data[packet.size++]='\0'; //null terminate string
    com_send(IFACE_SERIAL, &packet); // send it over the uart
}      

void shell_send_byte(uint8_t byte)
{
    comBuf send_packet;
    send_packet.size=1;
    send_packet.data[0]=byte;
    com_send(IFACE_SERIAL, &send_packet);
}

int shell_find_node (find_node_callback find_callback_func)
{
    comBuf *recv_pkt, send_pkt;
    uint8_t retry_max = 50;
    uint8_t retries = 0;
    bool  error_flag = false;
    com_mode (IFACE_SERIAL, IF_LISTEN);

    //send_pkt.data[0] = SHELL_PRESENT;
    //send_pkt.data[1] = '\0';
    //send_pkt.size=2;
    //com_send(IFACE_SERIAL, &send_pkt);
    /*
       if (clear_cb) {
       printf ("Sending cb 0\n");
       send_pkt.data[0] = CLEAR_CB;
       send_pkt.data[1] = '\0';
       send_pkt.size = 2;
       com_send(IFACE_SERIAL, &send_pkt);

       check_recv (CLEAR_CB_PONG);
       } else {
       send_pkt.data[0] = SHELL_PRESENT;
       send_pkt.data[1] = '\0';
       send_pkt.size=2;
       com_send(IFACE_SERIAL, &send_pkt);
       }
       */
    while(retries++ < retry_max) {
        if(find_callback_func != NULL)
            find_callback_func(retries);
        else {
            if(retries > 1)
                fprintf(stderr,"Retry [%d]...", retries);
        }

        if (clear_cb) {
            printf ("Sending cb 1\n");
            send_pkt.data[0] = CLEAR_CB;
            send_pkt.data[1] = '\0';
            send_pkt.size = 2;
            com_send (IFACE_SERIAL, &send_pkt);
        } else {
            send_pkt.data[0] = SHELL_PRESENT;
            send_pkt.data[1] = '\0';
            send_pkt.size=2;
            com_send (IFACE_SERIAL, &send_pkt);	 
        }

        recv_pkt = com_recv(IFACE_SERIAL);
        //usleep (10000);
        switch (recv_pkt->data[0]) {
            case CLEAR_CB_PONG:
                printf ("Cleared boot control block\n");
                clear_cb = 0;
                //com_free_buf (recv_pkt);
                //return(BOOTLOADER_MODE);
                break;
            case LOADER_PRESENT:
                send_pkt.data[0] = SHELL_PRESENT;
                send_pkt.data[1] = '\0';
                send_pkt.size = 2;
                com_send(IFACE_SERIAL, &send_pkt); //send a ping
                break;
            case LOADER_PRESENT_PONG:
                com_free_buf(recv_pkt);
                return(BOOTLOADER_MODE);
                break;
            case APP_PRESENT:
                com_free_buf(recv_pkt);
                recv_pkt = com_recv(IFACE_SERIAL);
                com_free_buf(recv_pkt); //throw away the command prompt
                return(APP_MODE);
                break;
            default:
                if(!error_flag){
                    fprintf(stderr, "Unexpected serial input from node: %d.\n",
                            recv_pkt->data[0]);
                    error_flag = true;
                }
                break;
        }
        com_free_buf (recv_pkt);

        //usleep(10000);
    }
    return NODE_NOT_FOUND;
}

int shell_find_loader(find_node_callback find_callback_func)
{
    int ret_val;
    fprintf(stderr,"Searching for a sensor node...");
    fflush (stderr);
    while((ret_val = shell_find_node (find_callback_func)) != BOOTLOADER_MODE){
        if(ret_val == APP_MODE){
            fprintf(stderr,"Application Mode, Sending reboot...\n");
            shell_send_to_node("MR"); //restart the app...
        }
        if(ret_val == NODE_NOT_FOUND) {
            fprintf(stderr,"Node not found!\n");
            return ret_val;
        }
        fprintf(stderr,"Searching for a sensor node...");
        ret_val = shell_find_node(find_callback_func);
    }
    fprintf (stderr,"bootloader mode.\n");
    return true;
}

int8_t shell_load_image(char *arg, update_callback callback_func)
{
    char *name, *s;
    char *image;
    int val;
    int8_t ret;

    // Only use file completion for this line, and then revert back
    if (arg == NULL) {
        //set_file_completion();
        fprintf(stderr,"Enter name of S-Record file: ");
        ret = scanf("%s", name);
        //set_command_completion();
    } else {
        name = arg;
    }

    s = stripwhite(name); // Strip the whitespace so fopen works

    // Read the srecord into memory
    if (strstr(s, ".elf") != NULL)
    {
        image = (char *)malloc(CODE_MAX_ELF_SIZE);
        printf("'%s' is an elf binary\n", s);

        val = elf_read(s, image);

    }
    else
    {
        image = (char *)malloc(CODE_MAX_SIZE);
        val = srec_read(s, image);
    }


    if(val == -1) {
        fprintf(stderr,"Did not load file...\n");
        free (image);
        return SREC_READ_ERROR;
    }

    ret = write_image(image, val, callback_func);

    free (image);
    return ret;
}

/* send a command that restarts the node */
void shell_send_restart()
{
    shell_send_to_node("MR"); //restart the app
}

/* Send command to start program execution. */
int8_t shell_start_execution(char *arg)
{
    comBuf startex_packet;
    startex_packet.size = 1;
    startex_packet.data[0] = START;
    uint8_t ret_val;

    com_send(IFACE_SERIAL, &startex_packet);
    while(1) {
        ret_val = check_recv(START);
        if(ret_val == NODE_RESTARTED) { //node restarted so...
            fprintf(stderr,"oops... node restarted, sending again.\n");
            usleep (10000);
            com_send(IFACE_SERIAL, &startex_packet); //try again
        }
        else 
            return SUCCESS;
    }
}

void shell_load_bootloader()
{
    char *write_fuse = "uisp -dprog=mib510 -dserial=/dev/ttyS0 -dpart=ATmega128 --wr_fuse_h=0x10 -v=0";
    char *erase_mem = "uisp -dprog=mib510 -dserial=/dev/ttyS0 -dpart=ATmega128 --erase -v=0";
    char *load_srec = "uisp -dprog=mib510 -dserial=/dev/ttyS0 -dpart=ATmega128 --upload if=/usr/local/share/mos/boot.srec -v=0";
    char *verify = "uisp -dprog=mib510 -dserial=/dev/ttyS0 -dpart=ATmega128 --verify if=/usr/local/share/mos/boot.srec -v=0";


    serial_block();

    if(file_exists("/usr/local/share/mos/boot.srec") == false){
        fprintf(stderr, "ERROR: /usr/local/share/mos/boot.srec not found.\n");
        return;
    } else {
        fprintf(stderr, "boot file found!\n");
    }

    fprintf(stderr, "\nWriting the fuse high byte...\n");
    fprintf(stderr, "%s\n",write_fuse);
    if(!run_command(write_fuse)){
        printf("Writing fuse high byte failed.\n");
        return;
    }

    fprintf(stderr, "\nErasing program memory...\n");
    fprintf(stderr, "%s\n",erase_mem);
    if(!run_command(erase_mem)){
        printf("Erasing memory failed.\n");
        return;
    }

    fprintf(stderr, "\nLoading the bootloader program...\n");
    fprintf(stderr, "%s\n",load_srec);
    if(!run_command(load_srec)){
        printf("loading failed.\n");
        return;
    }

    fprintf(stderr, "\nVerifying the new image...\n");
    fprintf(stderr, "%s\n",verify);
    if(!run_command(verify)){
        printf("verify failed.\n");
        return;
    }

    fprintf(stderr, "Done.\n");
    serial_unblock();

}

