#include <inttypes.h>

#include "mos.h"
#include "msched.h"
#include "clock.h"
#include "loader/elfloader.h"
#include "prog2load_buf.h"
#include "symbols.h"

/* Loads static ELF data located in the prog2load buffer
   (prog2load_buf.c). The ELF is loaded and unloaded periodically. */
void start(void) {
  int elfloader_err;
  uint32_t sleep_time = 4000;
  mos_thread_sleep (sleep_time);
  
  printf("load_static started\n\n");
  
  /* Load and unload forever */
  while (1) {
    if (elfloader_loaded_unload != NULL) {
      printf("shutting down already started application...\n");
      elfloader_loaded_unload();
    }
    
    printf("sleeping...\n");
    mos_thread_sleep (sleep_time);
    
    /* Prepare ELF loader by setting memory to handle */
    printf("writing ELF data to elfstore %C\n", sizeof(prog2load));
    elfstore_write(prog2load, 0, sizeof(prog2load));
    
    /* Load ELF */
    printf("loading ELF now...");
    elfloader_err = elfloader_load();
    if (elfloader_err != ELFLOADER_OK) {
      printf("failed\n");
      printf("ELF loader returned error %C, elfloader_unknown = %s\n", elfloader_err, elfloader_unknown);
      printf("symbols size is %C\n", sizeof(symbols));
    } else {
      /* Start loaded application */
      printf("success\n");
      printf("sleeping...\n");
      mos_thread_sleep (sleep_time);
      
      printf("starting loaded application now...\n");
      elfloader_loaded_load();
    }
    
    printf("waiting before reloading...\n\n\n");
    mos_thread_sleep (sleep_time*10);
  }
}
