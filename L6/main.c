#include "m_general.h"
#include "m_bus.h"
#include "m_bus.c"
#include "m_imu.h"
#include "m_imu.c"
#include "m_usb.h"
#include "m_usb.c"

int main(void) {
  DDRE |= (1 << 6);
  PORTE |= (1 << 6);

  int data[9];
  m_bus_init();
  m_usb_init();
  while(!m_usb_isconnected());
  if(!m_imu_init(0, 0)) {
    PORTE &= ~(1 << 6);
    while(1);
  }
  while(1) {
    if (m_imu_raw(data)) {
      m_usb_tx_int(data[0]);
      m_usb_tx_string("\n\r");
    }
  }
  return 0;
}
