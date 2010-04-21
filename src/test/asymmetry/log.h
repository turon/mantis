#ifndef LOG_FILE_MANAGE
#define LOG_FILE_MANAGE

#include "simple_fs.h"
#include "mutex.h"

// return the current log head, the next free location. Does not 
// automaticaly lock the file from modification, so lock the mutex 
// first if you need protection.
uint16_t get_log_length();

// append to the end of the log, checking if the log is full.  returns 0 if the
// log can not be grown further.  the buffer passed may not be the same buffer
// passed to open_log.  this operation locks the file until completion.
uint8_t append_log(uint8_t *buf);

// read log entry at index i.  returns NULL if the index is invalid.  if buf is
// NULL, the function uses the buffer passed in reopen_log is used.  this is
// not safe if there is a seperate thread writing at the same time.  note that
// read_log does not lock the file, during the read, so incompletely written data
// will not be returned.  locking outside the function is best if you pass buf
// as NULL.
uint8_t *read_log_index(uint16_t i, uint8_t *buf);

// open the current log or initialize a new one if it's not present.  This
// operation locks the file until completion.  if it fails the file remains
// locked.
void reopen_log(char *name, uint16_t file_size, uint8_t rec_size, char foot, 
                uint8_t *buf);

// clear the log (delete the filesystem) and reset the node.  the next call to
// init_log() will create a default log.
void clear_log();

typedef struct _log_header_t {
    //header var
    uint8_t  record_size;
    uint8_t  footer_char;
    uint16_t length;

    //instance var
    uint8_t     *ubuf;
    uint16_t     offset;
    mos_file    *logfile;
    mos_mutex_t  file_lock;
} __attribute__((packed)) log_file;

extern log_file open_log;

#endif
