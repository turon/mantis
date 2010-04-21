#include "dvdrp.h"

int start(void) {
  printf("%C\n", __alignof__(dvdrp_mesg_pkt));
  printf("%C\n", __alignof__(dvdrp_advert_pkt));
  printf("%C\n", __alignof__(uint32_t));
  printf("%C\n", __alignof__(uint16_t));
  printf("%C\n", __alignof__(uint8_t));

  return 0;
}
