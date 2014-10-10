#include "m_general.h"
#include <stdint.h>

ISR(TIMER1_COMPA_vect) {
  PORTB &= ~(1 << 4);
  TCNT1 = 0;
}

ISR(TIMER1_COMPB_vect) {
  DDRE |= (1 << 6);
  PORTB |= (1 << 4);
}

int main(void) {
  cli();

  /* configure output pin */
  DDRB |= 1 << 4;
  PORTB &= ~(1 << 4);

  DDRE |= 1 << 6;
  PORTE &= ~(1 << 6);

  /* set clock to full 16MHz */
  CLKPR = 1 << CLKPCE;
  CLKPR = 0x00;

  /* configure timer */
  TCCR1A = 0x00; // regular timer 1 output compare operation
  TCNT1 = 0x00; // set initial timer value to 0
  OCR1A = 48484; // set output compare A default value
  OCR1B = 24242; // set output compare B default value
  TIMSK1 = 0x06; // enable A and B output compares
  TCCR1B = 0x01; // enable timer

  sei();
  while(1);

  return 0;
}
