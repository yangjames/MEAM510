#include <m_general.h>

int main(void) {
  DDRE |= 1<<6;
  PORTE &= !(1<<6);
  for(;;) {
  }
  return 0;
}
