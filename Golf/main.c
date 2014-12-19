#include "m_includes.h"
#include "config.h"
#include "motor.h"
#include "solenoid.h"
#include "m_usb.h"

/* predeclarations */
void init();
void read_adc(int* adc_val, uint8_t* adc_mux_idx);

/* global variables */
volatile uint64_t tim3_ovf;
volatile char state = GO;
volatile char team = N_INIT;
volatile char go_flag = 0;
volatile char motor_disable = 0;

/* main function */
int main(void) {
  /* controls and filtering variables */
  float dt = 0.0;
  float l_duty = 0.0;
  float r_duty = 0.0;

  /* adc variables */
  uint8_t adc_mux_idx[10] = {0,1,7,32,4,5,6,35,36,33};
  int adc_val[10];

  /* system initialize */
  init();
  enable_motors();
  float ratio = 0.0;
  int i;
  int left_sensor = 6, right_sensor = 3;
  int counter = 0;
  /* main loop */
  while(1) {

    /* get delta t */
    dt = (float)(TCNT3 + tim3_ovf*65536)/16000000;
    tim3_ovf = 0;
    TCNT3 = 0;

    /* read adc values */
    read_adc(adc_val, adc_mux_idx);
    
    ratio = (float)(adc_val[left_sensor])/adc_val[right_sensor];
    /* state machine */
    switch (state) {
    case GO: {
      if (fabs(ratio - 1.0) < 0.05 && counter++ > 500)
	state = STOP;
      else {
	if (adc_val[left_sensor] > adc_val[right_sensor]) {
	  set_direction(MOTOR_L, BACKWARD);
	  set_direction(MOTOR_R, FORWARD);
	  set_duty_cycle(MOTOR_L, 0.3);
	  set_duty_cycle(MOTOR_R, 0.3);
	}
	if (adc_val[left_sensor] < adc_val[right_sensor]) {
	  set_direction(MOTOR_R, BACKWARD);
	  set_direction(MOTOR_L, FORWARD);
	  set_duty_cycle(MOTOR_L, 0.3);
	  set_duty_cycle(MOTOR_R, 0.3);
	}
      }
      break;
    }
    case STOP: {
      motor_disable = 1;
      set_direction(MOTOR_L, BRAKE);
      set_direction(MOTOR_R, BRAKE);
      set_duty_cycle(MOTOR_L, 0.0);
      set_duty_cycle(MOTOR_R, 0.0);
      go_flag = 0;
      motor_disable = 0;
      m_wait(20000);
      PORTD &= ~(1 << 7);
      m_wait(45);
      PORTD |= (1 << 7);
      while(1);
    }
    default:
      break;
    }
  }
  return 0;
}


/* initialization code */
void init() {
  cli();

  /* set clock to full 16MHz */
  CLKPR = 1 << CLKPCE;
  CLKPR = 0x00;

  /* initialize fiene code */
  m_disableJTAG();
  m_usb_init();

  /* initialize drivers */
  init_motor_drivers();
  //init_solenoid();
  DDRD |= (1 << 7);
  PORTD |= (1 << 7);
  m_green(ON);

  /* initialize ADC */
  ADMUX |= (1 << REFS0); // set reference voltage to VCC 5V
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescale by 128
  DIDR0 |= 0xF3; // disable digital input
  DIDR2 |= 0x1B; // disable digital input

  /* intialize system timer */
  TCCR3A = 0x00; // normal compare output mode operation
  TCNT3 = 0x00; // intial timer value = 0
  TIMSK3 = 0x01; // enable overflow interrupt
  TCCR3B = 0x01; // enable timer

  /* breakbeam sensor */
  DDRE &= ~(1 << 6);

  sei();
}

void read_adc(int* adc_val, uint8_t* adc_mux_idx) {
  int i;
  for (i = 0; i < 10; i++) {
    ADCSRA &= ~(1 << ADEN);
    ADMUX = (1 << REFS0) | ((adc_mux_idx[i]) & ((1 << MUX0) | (1 << MUX1) | (1 << MUX2)));
    ADCSRB = ((adc_mux_idx[i]) & (1 << MUX5));
    ADCSRA |= (1 << ADEN);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    adc_val[i] = ADCW;
  }
}

/* system timer overflow */
ISR(TIMER3_OVF_vect) {tim3_ovf++;}
