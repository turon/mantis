#include <stdio.h>
#include <fcntl.h>

static char buffer[100000];

int main(int argc, char **argv) {
  int fd;
  int i;
  int size;
  
  if (argc < 2) {
    fprintf(stderr, "need file argument, aborting\n");
    exit(1);
  }
  
  fd = open(argv[1], O_RDONLY);
  if(fd <= 0) {
    fprintf(stderr, "open error %d\n", fd);
    exit(1);
  }
  
  size = read(fd, buffer, sizeof(buffer));
  fprintf(stderr, "read size %d\n", size);
  close(fd);
  
  printf("const char prog2load[%i] = {\n", size);
  for (i=0; i < size; i++) {
    printf("%i", (int) buffer[i]);
    if (i != size-1)
      printf(",\n");
  } 
  printf("};\n\n");

  return 0;
}
