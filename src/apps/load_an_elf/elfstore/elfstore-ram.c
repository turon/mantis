#include "elfstore/elfstore.h"
#include <string.h>

#define ELFSTORE_SIZE 0x800

static char elf_data[ELFSTORE_SIZE];

/*---------------------------------------------------------------------------*/
int elfstore_read(char *buf, unsigned short offset, unsigned int len) {
  memcpy(buf, &elf_data[offset], len);
  return len;
}
/*---------------------------------------------------------------------------*/
int elfstore_write(char *buf, unsigned short offset, unsigned int len) {
  memcpy(&elf_data[offset], buf, len);
  return len;
}
/*---------------------------------------------------------------------------*/
int elfstore_size() {
  return ELFSTORE_SIZE;
}
