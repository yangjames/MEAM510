#include m_general.h
#include m_usb.c

int main(void) {
  //init();
  
  return 0;
}

void init() {
  /* initialize kicker pin */
  DDRB |= (1 << 6);
  PORTB |= (1 << 6);
  
  /* configure ADC */
  ADMUX |= (1 << REFS0); // set reference voltage to VCC 5V
  ADCSRA |= (1 << ADEN); // enable ADC

}
