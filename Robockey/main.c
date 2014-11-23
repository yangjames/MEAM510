#include "m_includes.h"
#include "config.h"
#include "motor.h"
#include "solenoid.h"
#include "m_usb.h"

/* predeclarations */
void init();
void parse_radio_data();

/* global variables */
uint8_t radio_buf[PACKET_LENGTH];
volatile char radio_flag;
volatile uint64_t tim3_ovf;

/* main function */
int main(void) {
  /* system initialize */
  init();

  float dt = 0.0;

  /* main loop */
  while(1) {
    /* check if we got a new packet */
    if (radio_flag) {
      radio_flag = 0;
      parse_radio_data();
    }

    /* get delta t */
    dt = (float)(TCNT3 + tim3_ovf*65536)/16000000;
    tim3_ovf = 0;
    TCNT3 = 0;
    
    m_usb_tx_string("delta t: ");
    m_usb_tx_int((int)(dt*1000000));
    m_usb_tx_string("us\n\r");
  }

  return 0;
}

/* initialization code */
void init() {
  cli();

  /* set clock to full 16MHz */
  CLKPR = 1 << CLKPCE;
  CLKPR = 0x00;

  /* initialize fiene code */
  m_usb_init();
  m_bus_init();
  while (!m_rf_open(CHANNEL, ADDRESS, PACKET_LENGTH));

  /* initialize drivers */
  init_motor_drivers();
  init_solenoid();

  /* intialize system timer */
  TCCR3A = 0x00; // normal compare output mode operation
  TCNT3 = 0x00; // intial timer value = 0
  TIMSK3 = 0x01; // enable overflow interrupt
  TCCR3B = 0x01; // enable timer

  sei();
}

void parse_radio_data() {
  switch(radio_buf[0]) {
  case 0xA0: break;
  case 0xA1: m_red(ON); break;
  case 0xA2: m_green(ON); break;
  case 0xA3: break;
  case 0xA4: break;
  case 0xA5: break;
  case 0xA6: break;
  case 0xA7: break;
  case 0xA8: break;
  default: break;
  }
}

/* system timer overflow */
ISR(TIMER3_OVF_vect) {tim3_ovf++;}

/* read in radio packet */
ISR(INT2_vect) {m_rf_read(radio_buf, PACKET_LENGTH); radio_flag = 1;}
