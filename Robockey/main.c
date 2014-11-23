/* fiene code */
#include "m_includes.h"

/* custom code */
#include "config.h"
#include "motor.h"

/* predeclarations */
void init();

char radio_buf[PACKET_LENGTH];

/* main function */
int main(void) {
  /* initialize */
  init();

  /* main loop */
  while(1) {
    m_usb_tx_string("hello world\n\r");
  }

  return 0;
}

void init() {
  /* clear global interrupts */
  cli();

  /* set clock to full 16MHz */
  CLKPR = 1 << CLKPCE;
  CLKPR = 0x00;

  /* initialize USB port for debugging purposes */
  m_usb_init();

  /* initialize i2c port */
  m_bus_init();

  /* initialize radio port */
  while (!m_rf_open(CHANNEL, ADDRESS, PACKET_LENGTH));

  /* initialize motor drivers */
  init_motor_drivers();

  /* set global interrupts */
  sei();
}

/* read in radio packet */
ISR(INT2_vect) { m_rf_read(radio_buf, PACKET_LENGTH); }
