#include "simple_fs.h"
#include "mutex.h"
#include "com.h"
#include "printf.h"
#include "string.h"

#include "log.h"

#define LOG_HEADER_LENGTH 4

log_file open_log;

//void __debug_print_hdr(log_file *f) {
//    printf("%C %C %d\n", f->record_size, f->footer_char, f->length);
//}

//void __debug_read_print_hdr() {
//    log_file tmp_hdr;
//    
//    mos_file_read((uint8_t*)&tmp_hdr, open_log.logfile, 0, LOG_HEADER_LENGTH);
//    __debug_print_hdr(&tmp_hdr);
//}

uint16_t get_log_length() {
    uint16_t bytes = LOG_HEADER_LENGTH;
    uint8_t  i, ctl;

    while(bytes < open_log.length) {
        mos_file_read(open_log.ubuf, open_log.logfile, bytes, 
                      open_log.record_size);

        ctl = 1;
        for(i = 0; i < open_log.record_size; ++i) {
            if(open_log.ubuf[i] != open_log.footer_char) {
                ctl = 0;
                break;
            }
        }

        if(ctl) {
            break;
        }

        bytes += open_log.record_size;
    }

    return bytes; 
}

uint8_t append_log(uint8_t *buf) {
    //need an extra buffer for the footer
    if(open_log.offset + (open_log.record_size << 1) >= open_log.length)
        return 0;

    mos_mutex_lock(&open_log.file_lock);
    
    //prepare footer
    memset(open_log.ubuf, open_log.footer_char, open_log.record_size);
    
    //write footer
    mos_file_write(open_log.ubuf, open_log.logfile, 
                   open_log.offset + open_log.record_size, 
                   open_log.record_size);
    //if we crash now the file is still correctly delimated
    mos_file_write(buf, open_log.logfile, open_log.offset, 
                   open_log.record_size);
    //if we crash now we get junk, but the file is not currupted
    mos_file_flush(open_log.logfile);
    
    open_log.offset += open_log.record_size;
    
    mos_mutex_unlock(&open_log.file_lock);

    return open_log.record_size;
}

uint8_t *read_log_index(uint16_t i, uint8_t *buf) {
    uint8_t  *tbuf;
    uint16_t  read_req;
    
    // are we trying to read too far?
    read_req = (i + 1) * open_log.record_size + LOG_HEADER_LENGTH;
    if(read_req > open_log.length || read_req > open_log.offset) {
        return NULL;
    }

    tbuf = (buf ? buf : open_log.ubuf);
    
    mos_file_read(tbuf, open_log.logfile, read_req - open_log.record_size,
                  open_log.record_size);

    return tbuf;
}

void reopen_log(char *name, uint16_t file_size, uint8_t rec_size, char foot, 
                uint8_t *buf) 
{
    printf("Init FS.\n");
    simple_fs_init();

    open_log.ubuf = buf;
    mos_mutex_init(&open_log.file_lock);
    mos_mutex_lock(&open_log.file_lock);

    printf("opening file\n");
    open_log.logfile = mos_file_open(name);
    if(open_log.logfile == NULL) {
        printf("Creating a new log.\n");
        open_log.logfile = mos_file_create(name, file_size);
        if(open_log.logfile == NULL) {
            //leave log locked - it could not be initialized
            printf("Could not initialize logfile, quiting.\n");
            return;
        }

        //else we write out a fresh file header
        open_log.record_size = rec_size;
        open_log.footer_char = foot;
        open_log.length = file_size;

        mos_file_write((uint8_t*)&open_log, open_log.logfile, 0, 
                       LOG_HEADER_LENGTH);
        mos_file_flush(open_log.logfile); //otherwise the header will not 
                                          //be written out

        memset(open_log.ubuf, foot, rec_size);
        mos_file_write(open_log.ubuf, open_log.logfile, LOG_HEADER_LENGTH,
                       rec_size);

        open_log.offset = LOG_HEADER_LENGTH;
    } else {
        mos_file_read((uint8_t*)&open_log, open_log.logfile,
                      0, LOG_HEADER_LENGTH);
        open_log.offset = get_log_length();
    }

    mos_mutex_unlock(&open_log.file_lock);

    printf("current log size: %d (%d)\n", open_log.offset, open_log.length);
}

typedef void (*reboot_func)(void);
static void reboot(void)
{
    // disable the scheduler.
#ifdef ARCH_AVR
    TCCR0 = (0 << CS02) | (0<< CS01) | (0 << CS00);  // sys clk /1024 (1 sec)
    SPCR &= ~(1 << SPE);
    cli();
#else
#warning "need to implement reboot for other arches"
#endif
    reboot_func rfunc = (reboot_func)0x1E000;
    rfunc();
}

void clear_log() {
    simple_fs_format();
    reboot();
}
