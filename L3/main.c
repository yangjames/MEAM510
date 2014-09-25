#include "m_general.h"
#include <stdint.h>

int main(void) {
  DDRE |= 1<<6;
  PORTE |= (1<<6);
  int i;
  while(1) {
    for(i=0;i<30000;++i)
      asm("nop");
    PORTE ^=(1<<6);
  }
  return 0;
}
