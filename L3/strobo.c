#include "m_general.h"
#include <stdint.h>

#define PI 3.14159265359

volatile uint64_t tim1_ovf;

ISR(TIMER1_OVF_vect) {
  tim1_ovf++;
}

int main(void) {
  /* set debugging LED output registers */
  DDRB = 0xFF;

  /* set initial overflow value to 0 */
  tim1_ovf = 0;

  /* clear global interrupts */
  cli();

  /* set clock to full 16MHz */
  CLKPR = 1 << CLKPCE;
  CLKPR = 0x00;

  /* configure ADC */
  ADMUX |= (1 << REFS0); // set reference voltage to VCC 5V
  ADCSRA |= (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0); // enable ADC, divide by 8

  /* configure timer 1 */
  TCCR1A = 0x00; // normal compare output mode operation
  TCNT1 = 0x00; // intial timer value = 0
  TIMSK1 = 0x01; // enable overflow interrupt
  TCCR1B = 0x01; // enable timer

  /* enable global interrupts */
  sei();

  /* declare variables */
  float current_time_stamp = 0;
  float previous_time_stamp = 0;

  float left_channel = 0.0;
  float right_channel = 0.0;
  float avg_channel = 0.0;

  float cutoff_low = 200; // low pass filter cutoff frequency in Hz
  float RC_low = 1/(cutoff_low*2*PI);
  float alpha_low = 0.0;
  float low_pass_out = 0.0;

  float cutoff_high = 2000; // high pass filter cutoff frequency in Hz
  float RC_high = 1/(cutoff_high*2*PI);
  float alpha_high = 0.0;
  float high_pass_out = 0.0;

  /* main loop */
  while(1) {

    /* read adc values from left and right audio channels*/
    ADMUX &= ~(1 << MUX0);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    left_channel = (float)(ADCW)/1024;
    ADMUX |= (1 << MUX0);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    right_channel = (float)(ADCW)/1024;

    /* average left and right channels */
    avg_channel = (left_channel + right_channel)/2;
    
    /* get time stamp */
    current_time_stamp = (float)(TCNT1 + tim1_ovf*65536)/16000000;
    tim1_ovf = 0;
    TCNT1 = 0;

    /* low pass filter */
    low_pass_out = 1/(RC_high/current_time_stamp + 1)*avg_channel;

    /* high pass filter */
    high_pass_out = RC_high/current_time_stamp/(RC_high/current_time_stamp + 1);

    /* calculate pwm */
    
  }

  return 0;
}
