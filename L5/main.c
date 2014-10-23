#include "m_general.h"
#include "m_usb.c"

typedef enum {
  SEARCH,
  STOP,
  SHOOT
};

void init();

int main(void) {
  init();

  PORTE &= ~(1 << 6);
  ADCSRA |= (1 << ADSC);

  while(1) {
  }

  return 0;
}

void init() {
  cli();
  m_usb_init();
  while(!m_usb_isconnected());

  /* initialize kicker pin */
  DDRB |= (1 << 6);
  PORTB |= (1 << 6);
  
  DDRE |= (1 << 6);
  PORTE |= (1 << 6);

  /* configure ADC */
  ADMUX |= (1 << REFS0) | (1 << MUX0); // set reference voltage to VCC 5V
  ADCSRA |= (1 << ADEN) | (1 << ADIE); // enable ADC
  
  sei();
}

ISR(ADC_vect) {
  m_usb_tx_int(ADCW);
  m_usb_tx_string("\n\r");
  ADCSRA |= (1 << ADSC);
}
