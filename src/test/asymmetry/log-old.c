#include "simple_fs.h"
#include "mutex.h"
#include "com.h"
#include "printf.h"
#include "string.h"

#include "log.h"

mos_file    *logfile = NULL;
mos_mutex_t  file_lock;
uint16_t     offset;

uint16_t get_log_pos() {
    uint8_t buf[2];

    mos_mutex_lock(&file_lock);
    mos_file_read(buf, logfile, 0, 2);
    mos_mutex_unlock(&file_lock);

    return buf_extract_WORD(buf, 0); 
}

void set_log_pos(uint16_t pos) {
    uint8_t buf[2];

    mos_mutex_lock(&file_lock);
    buf_insert_WORD(buf, 0, pos);
    mos_file_write(buf, logfile, 0, 2);
    mos_mutex_unlock(&file_lock);
}

uint8_t append_log(uint8_t *buf, uint8_t size) {
    uint8_t int_buf[2];
    
    if(offset + size >= logfile->length)
        return 0;
                
    mos_mutex_lock(&file_lock);
    
    mos_file_write(buf, logfile, offset, size);
    offset += size;
    buf_insert_WORD(int_buf, 0, offset);
    mos_file_write(int_buf, logfile, 0, 2);

    mos_mutex_unlock(&file_lock);

    return size;
}

void init_log() {
    char buf[6];
    
    printf("Init FS.\n");
    simple_fs_init();

    mos_mutex_init(&file_lock);

    printf("opening file\n");
    logfile = mos_file_open("log");
    if(logfile == NULL) {
        printf("Creating a new log.\n");
        logfile = mos_file_create("log", 50240);
        if(logfile == NULL) {
            printf("Could not initialize logfile, quiting.\n");
            //and keep the file locked - readers and wrterrs will block
            return;
        }

        //else we write out a fresh file header
        offset = 2;
        buf_insert_WORD(buf, 0, offset);
        mos_file_write(buf, logfile, 0, 2);
    } else {
        offset = get_log_pos();
    }
    
    //insert reset marker
    memset(buf, 0, sizeof(buf));
    mos_file_write(buf, logfile, offset, sizeof(buf));
    offset += sizeof(buf);
    set_log_pos(offset);

    printf("starting offset: %d\n", offset);
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
