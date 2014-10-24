#include "m_general.h"
#include "m_usb.c"

/* FSM states */
typedef enum {
  SEARCH,
  STOP,
  SHOOT
};

/* pre-declarations */
void init();

/* main */
int main(void) {
  /* initialize microcontroller */
  init();

  /* turn on status LED */
  PORTE &= ~(1 << 6);

  /* start ADC conversions */
  ADCSRA |= (1 << ADSC);

  /* start motor */
  TCCR1B |= (1 << CS10);

  /* main loop */
  while(1) {
    /* m_usb_tx_string("C6: "); */
    /* m_usb_tx_int((PINC & (1 << 6)) > 0); */
    /* m_usb_tx_string(" C7: "); */
    /* m_usb_tx_int((PINC & (1 << 7)) > 0); */
    /* m_usb_tx_string("\n\r"); */
    if (PINC & (1 << 6) == 0)
      PORTB |= (1 << 0);
    else
      PORTB &= ~(1 << 0);
    if (PINC & (1 << 7) == 0)
      PORTD |= (1 << 5);
    else
      PORTD &= ~(1 << 5);

    if (ADCW > 512) {
      m_green(OFF);
    }
    else {
      m_green(ON);
    }
  }

  return 0;
}

void init() {
  cli();

  /* set clock to full 16MHz */
  CLKPR = 1 << CLKPCE;
  CLKPR = 0x00;

  /* initialize debugging tool */
  m_usb_init();
  //while(!m_usb_isconnected());

  /* initialize kicker pin */
  // TODO

  /* intialize motor pins and PWM timer */
  DDRB |= (1 << 0) | (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6); // pwm, direction pins
  PORTB |= (1 << 4) | (1 << 1); // stay in one direction, start charging

  DDRD |= (1 << 5);
  //PORTD |= (1 << 5);
  

  TCCR1A |= (1 << WGM10) | (1 << WGM11); // waveform 15
  TCCR1B |= (1 << WGM12) | (1 << WGM13);

  TCCR1A |= (1 << COM1B1);
  OCR1A = 1200; // set to 14kHz at 50% duty cycle
  OCR1B = 600;

  /* initialize status LED */
  DDRE |= (1 << 6);
  PORTE |= (1 << 6);

  /* configure ADC */
  
  ADMUX |= (1 << REFS0) | (1 << MUX0); // set reference voltage to external AREF
  ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADPS2); // enable ADC
  
  sei();
}

ISR(ADC_vect) {
  //m_usb_tx_int(ADCW);
  //m_usb_tx_string("\n\r");
  ADCSRA |= (1 << ADSC);
}
