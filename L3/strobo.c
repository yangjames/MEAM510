#include "m_general.h"
#include <stdint.h>

volatile uint64_t tim1_ovf;

ISR(TIMER1_OVF_vect) {
  tim1_ovf++;
}

int main(void) {
  /* set initial overflow value to 0 */
  tim0_ovf = 0;

  /* clear global interrupts */
  cli();

  /* set clock to full 16MHz */
  CLKPR = 1 << CLKPCE;
  CLKPR = 0x00;

  /* configure ADC */
  ADMUX |= (1 << REFS0); // set reference voltage to VCC 5V
  ADCSRA |= (1 << ADEN); // enable ADC

  /* configure timer 1 */
  TCCR1A = 0x00; // normal compare output mode operation
  TCNT1 = 0x00; // intial timer value = 0

  /* enable global interrupts */
  sei();

  /* enable timer 1 */
  TCCR1B = 0x03; // no timer prescale

  /* declare variables */
  float current_time_stamp = 0;
  float previous_time_stamp = 0;

  float left_channel = 0.0;
  float right_channel = 0.0;

  int high_pass_pwm = 0;
  int low_pass_pwm = 0;

  /* set debugging LED output registers */
  DDRB = 0x03;

  /* main loop */
  while(1) {

    /* read adc values from left and right audio channels*/
    ADMUX &= ~(1 << MUX0);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    left_channel = (float)(ADC)/1024;

    ADMUX |= (1 << MUX0);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    right_channel = (float)(ADC)/1024;

    if (left_channel > 0.5)
      PORTB |= (1 << 0);
    else
      PORTB &= ~(1 << 0);

    if (right_channel > 0.5)
      PORTB |= (1 << 0);
    else
      PORTB &= ~(1 << 0);
  }

  return 0;
}
