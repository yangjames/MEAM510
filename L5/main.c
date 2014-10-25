#include "m_general.h"
#include "m_usb.c"

/* pre-declarations */
void init();

volatile int max_adc;

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
    //m_usb_tx_int(ADCW);
    //m_usb_tx_string("\n\r");

    /* turn on yellow LED at 50% charge */
    if ((PINC & (1 << 7)) == 0)
      PORTB |= (1 << 0);
    else
      PORTB &= ~(1 << 0);

    /* turn on green LED at 100% charge */
    if ((PINC & (1 << 6)) == 0)
      PORTD |= (1 << 5);
    else
      PORTD &= ~(1 << 5);
  }

  return 0;
}

void init() {
  cli();

  /* set clock to full 16MHz */
  CLKPR = 1 << CLKPCE;
  CLKPR = 0x00;

  m_usb_init();
  //while(!m_usb_isconnected());

  /* intialize output pins */
  DDRB |= (1 << 4) | (1 << 5) | (1 << 6); // pwm, direction pins
  DDRB |= (1 << 1); // charge / kick pin

  DDRB |= (1 << 0); // charge inidicator LEDs
  DDRD |= (1 << 5);

  PORTB |= (1 << 4); // set motor direction
  PORTB |= (1 << 1); //start charging

  /* initialize motor PWM timer1 */
  TCCR1A |= (1 << WGM10) | (1 << WGM11); // waveform 15
  TCCR1B |= (1 << WGM12) | (1 << WGM13);

  TCCR1A |= (1 << COM1B1);
  OCR1A = 1200; // set to 14kHz at 50% duty cycle
  OCR1B = 600;

  /* initialize kicker PWM timer 3 */
  

  /* initialize status LED */
  DDRE |= (1 << 6);
  PORTE |= (1 << 6);

  /* configure ADC */
  ADMUX |= (1 << REFS0) | (1 << MUX0); // set reference voltage to 5V
  ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADPS2); // enable ADC, enable interrupt, prescale by 16
  
  sei();
}


ISR(ADC_vect) {
  if (ADCW > max_adc)
    max_adc = ADCW;
  if (ADCW > 700)
    PORTB &= ~(1 << 4);
  ADCSRA |= (1 << ADSC);
}
