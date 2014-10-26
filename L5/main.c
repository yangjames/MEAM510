#include "m_general.h"
#include "m_usb.c"

#define PI 3.14159265359

/* pre-declarations */
void init();
void check_charge();

volatile uint64_t tim3_ovf, tim0_ovf;

int main(void) {
  /* initialize microcontroller */
  init();

  /* more variables */
  double dt = 0;
  float cutoff_low = 1.0;
  float RC_low = 1/(cutoff_low*2*PI);
  float alpha_low = 0.0;
  float adc_val = 0.0;
  int max_adc = 100;
  char peaked = 0;

  /* turn on status LED */
  PORTE &= ~(1 << 6);

  /* start ADC conversions */
  ADCSRA |= (1 << ADSC);


  /* start motor */
  TCCR1B |= (1 << CS10);

  /* main loop */
  while(1) {
    check_charge();
    if (!peaked) {
      /* get delta t */
      dt = (float)(TCNT3 + tim3_ovf*65536)/16000000;
      tim3_ovf = 0;
      TCNT3 = 0;

      /* calculate low pass gain */
      alpha_low = dt/(RC_low + dt);

      /* get new filtered adc value and store maximum value */
      adc_val =alpha_low*ADCW + (1-alpha_low)*adc_val;
      max_adc = ((int)adc_val > max_adc) ? (int)adc_val : max_adc;
      
      if ((max_adc - adc_val > 70.0) && (adc_val < 10.0))
	peaked = 1;
    }
    else {
      check_charge();
      /* get delta t */
      dt = (float)(TCNT3 + tim3_ovf*65536)/16000000;
      tim3_ovf = 0;
      TCNT3 = 0;

      /* calculate low pass gain */
      alpha_low = dt/(RC_low + dt);

      /* get new filtered adc value and store maximum value */
      adc_val =alpha_low*ADCW + (1-alpha_low)*adc_val;
      
      if (adc_val >= max_adc) {
	PORTB &= ~(1 << 4); // stop the motor
	while ((PINC & (1 << 7)) == 1) check_charge();
	check_charge();
	TCCR0B = 0x01;
	while(tim0_ovf < 187500);
	TCCR0B = 0x00;
	tim0_ovf = 0;
	PORTB &= ~(1 << 1); // turn on kicker
	TCCR0B = 0x01; // start timer
	while(1); // stay in an infinite loop. you're done :)
      }
    }
  }

  return 0;
}
void check_charge() {
  /* turn on yellow LED at 50% charge */
  if ((PINC & (1 << 6)) == 0)
    PORTD |= (1 << 5);
  else
    PORTD &= ~(1 << 5);
  /* turn on green LED at 100% charge */
  if ((PINC & (1 << 7)) == 0)
    PORTB |= (1 << 0);
  else
    PORTB &= ~(1 << 0);
}
void init() {
  cli();

  /* set clock to full 16MHz */
  CLKPR = 1 << CLKPCE;
  CLKPR = 0x00;

  m_usb_init();
  //  while(!m_usb_isconnected());

  /* intialize output pins */
  DDRB |= (1 << 4) | (1 << 5) | (1 << 6); // pwm, direction pins
  DDRB |= (1 << 1); // charge / kick pin
  
  DDRB |= (1 << 0); // charge inidicator LEDs
  DDRD |= (1 << 5);

  PORTB |= (1 << 4); // set motor direction
  PORTB |= (1 << 1); //start charging

  /* initialize motor PWM */
  TCCR1A |= (1 << WGM10) | (1 << WGM11); // waveform 15
  TCCR1B |= (1 << WGM12) | (1 << WGM13);

  TCCR1A |= (1 << COM1B1);
  OCR1A = 1200; // set to 14kHz at 50% duty cycle
  OCR1B = 500;

  /* configure system timer */
  TCCR3A = 0x00; // normal compare output mode operation
  TCNT3 = 0x00; // intial timer value = 0
  TIMSK3 = 0x01; // enable overflow interrupt
  TCCR3B = 0x01; // enable timer
  
  /* configure solenoid timer */
  TIMSK0 |= (1);

  /* initialize status LED */
  DDRE |= (1 << 6);
  PORTE |= (1 << 6);

  /* configure ADC */
  ADMUX |= (1 << REFS0) | (1 << MUX0); // set reference voltage to 5V
  ADCSRA |= (1 << ADEN) | (1 << ADIE) | (1 << ADPS2); // enable ADC, enable interrupt, prescale by 16
  
  sei();
}

ISR(TIMER0_OVF_vect) {
  if (tim0_ovf == 2750)
    PORTB |= (1 << 1);
  else
    tim0_ovf++;
}

ISR(TIMER3_OVF_vect) {
  tim3_ovf++;
}

ISR(ADC_vect) {
  ADCSRA |= (1 << ADSC);
}
