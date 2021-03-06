#include "m_general.h"
#include <stdint.h>
#include <stdlib.h>

#define PI 3.14159265359

volatile uint64_t tim1_ovf;
volatile int test, D_1, D_2, D_3, D_4;
float SF;

ISR(TIMER1_OVF_vect) {
  tim1_ovf++;
}

ISR(TIMER0_COMPA_vect) {
  PORTB &= ~(1 << 5);
}

ISR(TIMER0_OVF_vect) {
  PORTB |= (1 << 5);
}

ISR(TIMER3_COMPA_vect) {
  PORTB |= (1 << 6);
}

ISR(TIMER3_OVF_vect) {
  PORTB &= ~(1 << 6);
}

void reset_timers() {
  TIMSK0 = 0x00;
  TIMSK3 = 0x00;
}

void enable_timers() {
  TIMSK0 = 0x03;
  TIMSK3 = 0x03;
}

void audio_timer1() {
  TCCR1A = 0x00; // normal compare output mode operation
  TCNT1 = 0x00; // intial timer value = 0
  TIMSK1 = 0x01; // enable overflow interrupt
  TCCR1B = 0x01; // enable timer
}

void strobo_timer1() {
  // Set Timer 1 Prescaler /64 [31.25 kHz]
  /* clear(TCCR1B,CS12); */
  /* set(TCCR1B,CS11); */
  /* set(TCCR1B,CS10); */
    
  /* // Set Timer Mode - Mode 15 - */
  /* set(TCCR1B,WGM13); */
  /* set(TCCR1B,WGM12); */
  /* set(TCCR1A,WGM11); */
  /* set(TCCR1A,WGM10); */

  /* set(TCCR1A, COM1B1); */
  /* clear(TCCR1A, COM1B0); */
  TIMSK1 = 0x00;
  TCCR1A = 0x23;
  TCNT1 = 0x00;
  TCCR1B = 0x1B;
}

int main(void) {
  /* set debugging LED output registers */
  DDRB = 0xFF;
  DDRD = 0x00;

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

  /* enable global interrupts */
  sei();

  /* declare variables */
  double current_time_stamp = 0;

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

  char operation_mode = 0;
  /* main loop */
  while(1) {
    if(PIND & (1 << 6)) {
      if (!operation_mode) {
	cli();
	/* set clock to full 16MHz */
	CLKPR = 1 << CLKPCE;
	CLKPR = 0x00;
	enable_timers();
	audio_timer1();
	ADMUX &= ~(1 << MUX2);
	operation_mode = 1;
	sei();
      }

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
      avg_channel = (left_channel + right_channel)/2 - 0.5; // 0 < avg_channel < 1
    
      /* get time stamp */
      current_time_stamp = (float)(TCNT1 + tim1_ovf*65536)/16000000;
      tim1_ovf = 0;
      TCNT1 = 0;

      /* low pass filter */
      alpha_low = current_time_stamp/(RC_low+current_time_stamp);
      low_pass_out = alpha_low*avg_channel + (1-alpha_low)*low_pass_out;

      /* high pass filter */
      alpha_high = RC_high/(current_time_stamp + RC_high);
      high_pass_out = alpha_high*high_pass_out + alpha_high*(avg_channel - avg_channel_previous);
      avg_channel_previous = avg_channel;

      /* calculate pwm duty cycle */
      float duty_cycle_low = low_pass_out>=0.05 ? low_pass_out : 0;
      float duty_cycle_high = high_pass_out >= 0.05 ? high_pass_out : 0;
      OCR0A = (int)(duty_cycle_low*256);
      OCR3A = (int)(duty_cycle_high*65536);
    }
    else {
      if (operation_mode) {
	cli();
	m_clockdivide(3);
	ADMUX &= ~(1 << MUX0);
	ADMUX |= (1 << MUX2);
	reset_timers();
	strobo_timer1();
	operation_mode = 0;
	sei();
      }
      ADCSRA |= (1 << ADSC);
      while(ADCSRA & (1 << ADSC));
      test = ADC;
      D_1 = 8*check(PIND,4);
      D_2 = 4*check(PIND,5);
      D_3 = 2*check(PIND,2);
      D_4 = 1*check(PIND,3);

      SF = (1 + D_1 + D_2 + D_3 + D_4)/17.0;

      OCR1A = 50*test;
      OCR1B = (int)(SF*OCR1A);
    }
  }

  return 0;
}
