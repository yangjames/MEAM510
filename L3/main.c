#include "m_general.h"
#include <stdint.h>

/* section of code you want to run */
#define L3_2_3_1

ISR(TIMER1_COMPA_vect) {
  PORTB ^= 1 << 6;
  TCNT1 = 0;
}

int main(void) {

  /* 2.1.1 */
#ifdef L3_2_1_1

  /* set output pin 6 on port E and turn off */
  DDRE |= 1 << 6;
  PORTE |= 1 << 6;

  /* main loop */
  int i;
  uint64_t cycles = 30000;

  while(1) {

    /* no-op for defined number of cycles to delay blinking */
    for(i=0;i<cycles;++i)
      asm("nop");

    /* toggle 6 pin in register E */
    PORTE ^=(1<<6);
  }
#endif
  
  /* 2.1.2 */
#ifdef L3_2_1_2

  /* set direction of pin 6 on port B and turn off */
  DDRB |= 1 << 6;
  PORTB |= 1 << 6;

  /* main loop */
  int i;
  uint64_t cycles = 249;
  while(1) {
    for (i = 0; i < cycles; i++)
      asm("nop");
  
    /* toggle 6 pin in register B */
    PORTB ^= 1 << 6;
  }
#endif

#ifdef L3_2_1_3

  /* set direction of pin 6 on port B and turn off */
  DDRB |= 1 << 6;
  PORTB |= 1 << 6;

  /* set clock pre-scaler */
  CLKPR = (1 << CLKPCE);
  CLKPR = 4;

  /* main loop */
  int i;
  uint64_t cycles = 249;
  while(1) {
    for (i = 0; i < cycles; i++)
      asm("nop");
  
    /* toggle 6 pin in register B */
    PORTB ^= 1 << 6;
  }

#endif

#ifdef L3_2_2_1
  /* clear interrupts */
  cli();

  /* set direction of pin 6 on port B and turn off */
  DDRB |= 1 << 6;
  PORTB |= 1 << 6;

  CLKPR = (1 << CLKPCE);
  CLKPR = 0;

  /* configure timer 1 */
  TCCR1A = 0x00; // normal compare output mode operation
  TCCR1B = 0x01; // no prescale
  TCNT1 = 0x00; // set initial timer value to 0
  OCR1A = 16000; // set output compare value for 500Hz
  TIMSK1 = 0x02; // enable output compare

  /* enable interrupts */
  sei();

  /* main loop */
  while(1);

#endif

#ifdef L3_2_2_2
  /* clear interrupts */
  cli();

  /* set direction of pin 6 on port B and turn off */
  DDRB |= 1 << 6;
  PORTB |= 1 << 6;

  CLKPR = (1 << CLKPCE);
  CLKPR = 4;

  /* configure timer 1 */
  TCCR1A = 0x00; // normal compare output mode operation
  TCCR1B = 0x03; // no prescale
  TCNT1 = 0x00; // set initial timer value to 0
  OCR1A = 15625; // set output compare value for 0.5Hz
  TIMSK1 = 0x02; // enable output compare

  /* enable interrupts */
  sei();

  /* main loop */
  while(1);

#endif

#ifdef L3_2_3_1
  /* clear interrupts */
  cli();

  /* set direction of pin 6 on port B and turn off */
  DDRB |= 1 << 6;
  PORTB |= 1 << 6;

  /* configure ADC */
  ADMUX |= (1 << REFS0); // set reference voltage to VCC 5V
  ADCSRA |= (1 << ADEN); // enable ADC

  /* enable interrupts */
  sei();

  while(1) {
    /* start conversion */
    ADCSRA |= (1 << ADSC);

    /* wait until conversion is finished */
    while (ADCSRA & (1 << ADSC));

    /* check ADC value for turning on/off the LED */
    if (ADCW < 512)
      PORTB |= (1 << 6);
    else
      PORTB &= ~(1 << 6);
  }

#endif

  return 0;
}
