#include "m_general.h"
#include <stdint.h>
#include <stdlib.h>

#define PI 3.14159265359

/* global variables */
volatile uint64_t tim1_ovf;
volatile int test, D_1, D_2, D_3, D_4;
float SF;
volatile char op_mode_flag;

/* main process timer */
ISR(TIMER1_OVF_vect) {
  tim1_ovf++;
}

/* low pass filter pwm timer */
ISR(TIMER0_COMPA_vect) {
  PORTB &= ~(1 << 5);
}

ISR(TIMER0_OVF_vect) {
  PORTB |= (1 << 5);
}

/* high pass filter pwm timer */
ISR(TIMER3_COMPA_vect) {
  PORTB |= (1 << 6);
}

ISR(TIMER3_OVF_vect) {
  PORTB &= ~(1 << 6);
}

void set_clock_prescale() {
  /* set clock to full 16MHz */
  CLKPR = 1 << CLKPCE;
  CLKPR = 0x00;
}

void reset_timers() {
  TCCR1A = 0x00;
  TCCR1B = 0x00;
  TIMSK1 = 0x00;
  TCNT0 = 0x00;
  OCR1A = 0x00;

  TCCR0A = 0x00;
  TCCR0B = 0x00;
  TIMSK0 = 0x00;
  TCNT0 = 0x00;
  OCR0A = 0x00;

  TCCR3A = 0x00;
  TCCR3B = 0x00;
  TIMSK3 = 0x00;
  TCNT3 = 0x00;
  OCR3A = 0x00;
}

void reset_adc() {
  ADMUX = 0x00;
  ADCSRA = 0x00;
  DIDR0 = 0x00;
  ADCSRB = 0x00;
}

void configure_audio_visualizer() {
  cli();

  set_clock_prescale(); 
  reset_timers();
  reset_adc();

  /* configure ADC */
  ADMUX |= (1 << REFS0); // set reference voltage to VCC 5V
  ADCSRA |= (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0); // enable ADC, divide by 8

  /* configure timer 1 */
  TCCR1A = 0x00; // normal compare output mode operation
  TCNT1 = 0x00; // intial timer value = 0
  OCR0A = 0x00; // set output compare A value to 0
  TIMSK1 = 0x01; // enable overflow interrupt
  TCCR1B = 0x01; // enable timer

  /* configure timer 0 */
  TCCR0A = 0x00; // set normal output compare operation
  TIMSK0 = 0x03; // enable output compare A and overflow interrupts
  OCR0A = 0x00; // set output compare A value
  TCNT0 = 0x00; // reset timer
  TCCR0B = 0x04; // enable timer 0

  /* configure timer 3 */
  TCCR3A = 0x00;
  TIMSK3 = 0x03;
  OCR3A = 0x00;
  TCNT3 = 0x00;
  TCCR3B = 0x01;

  sei();
}

void configure_stroboscope() {
  cli();

  reset_timers();
  reset_adc();

  /* configure timer 1 */
  clear(TCCR1B, CS12);
  set(TCCR1B, CS11);
  set(TCCR1B, CS10);

  /* set timer mode - mode 15 - */
  set(TCCR1B, WGM13);
  set(TCCR1B, WGM12);
  set(TCCR1A, WGM11);
  set(TCCR1A, WGM10);

  /* set system clock prescaler [2 MHz] */
  m_clockdivide(3);

  /* channel b compare output */
  set(TCCR1A, COM1B1);
  clear(TCCR1A, COM1B0);

  /* 1. set voltage reference - internal 2.56V */
  set(ADMUX, REFS1);
  set(ADMUX, REFS0);

  /* 2. set ADC prescaler - /16 -> 125kHz */
  set(ADCSRA, ADPS2);
  clear(ADCSRA, ADPS1);
  clear(ADCSRA, ADPS0);

  /* 3. Disable digital inputs */
  set(DIDR0, ADC0D);

  /* single-ended channel selection - ADC4 - F1 */
  clear(ADCSRB, MUX5);
  set(ADMUX, MUX2);
  clear(ADMUX, MUX0);
  clear(ADMUX, MUX1);

  /* 4. set up interrupts and triggering, if desired */
  set(ADCSRA, ADATE); // set to free running mode

  /* 5. enable conversions */
  set(ADCSRA, ADEN);

  /* 7. start the conversion process */
  set(ADCSRA, ADSC);

  sei();
}

/* main loop */
int main(void) {
  /* set debugging LED output registers */
  DDRD = 0x50; // set pins 6 and 7 to output, 2-5 to input

  /* set initial overflow value to 0 */
  tim1_ovf = 0;

  /* set operation default mode to standard stroboscope */
  op_mode_flag = 0;

  /* clear global interrupts */
  cli();

  /* configure for audio visualizer */
  configure_stroboscope();
  //configure_audio_visualizer();

  /* enable global interrupts */
  sei();

  /* declare variables */
  double dt = 0;

  float left_channel = 0.0;
  float right_channel = 0.0;
  float avg_channel = 0.0;
  float avg_channel_previous = 0.0;

  float cutoff_low = 150; // low pass filter cutoff frequency in Hz
  float RC_low = 1/(cutoff_low*2*PI);
  float low_pass_out = 0.0;
  float alpha_low = 0.0;

  float cutoff_high = 500; // high pass filter cutoff frequency in Hz
  float RC_high = 1/(cutoff_high*2*PI);
  float high_pass_out = 0.0;
  float alpha_high = 0.0;

  /* main loop */
  while(1) {
    switch(1) {
    //switch(PIND & (1 << 0)) {
    case 0: {

      /* configure audio visualizer flags if necessary */
      if (!op_mode_flag) configure_audio_visualizer();
      op_mode_flag = 0;
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
      avg_channel = (left_channel + right_channel)/2 - 0.5;
    
      /* get elapsed time since last iteration and reset timer */
      dt = (float)(TCNT1 + tim1_ovf*65536)/16000000;
      tim1_ovf = 0;
      TCNT1 = 0;

      /* low pass filter */
      alpha_low = dt/(RC_low + dt);
      low_pass_out = alpha_low*avg_channel + (1-alpha_low)*low_pass_out;

      /* high pass filter */
      alpha_high = RC_high/(dt + RC_high);
      high_pass_out = alpha_high*high_pass_out + alpha_high*(avg_channel - avg_channel_previous);
      avg_channel_previous = avg_channel;

      /* calculate pwm duty cycle */
      float duty_cycle_low = low_pass_out>=0.05 ? low_pass_out : 0;
      float duty_cycle_high = high_pass_out >= 0.05 ? high_pass_out : 0;
      OCR0A = (int)(duty_cycle_low*256); // timer 0 is only 8 bits, so we scale differently
      OCR3A = (int)(duty_cycle_high*65536);
      break;
    }
    case 1: 
      if (op_mode_flag) configure_stroboscope();
      op_mode_flag = 1;
      break;
    default: break;
    }

  }

  return 0;
}
