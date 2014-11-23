#include "config.h"
#include "motor.h"

/* initialize motor drivers */
void init_motor_drivers() {
  /* set direction out for enable and direction pins */
  MOTOR_PORT |= ((1 << L_EN) | (1 << R_EN) 
		 | (1 << L_DIR_1) | (1 << L_DIR_2) 
		 | (1 << R_DIR_1) | (1 << R_DIR_2));

  /* turn all motor pins off at start */
  MOTOR_PORT &= ~((1 << L_EN) | (1 << R_EN) 
		  | (1 << L_DIR_1) | (1 << L_DIR_2) 
		  | (1 << R_DIR_1) | (1 << R_DIR_2));

  /* configure motor enable pwm timer */
  TCCR1A |= (1 << WGM10) | (1 << WGM11); // waveform 15
  TCCR1B |= (1 << WGM12) | (1 << WGM13);
  TCCR1A |= (1 << COM1B1) | (1 << COM1C1); // enable out OCA and OCC
  OCR1A = 1200; // set to 14kHz at 0% duty cycle
  OCR1B = 0;
  OCR1C = 0;
}

/* set motor drivers */
void set_duty_cycle(int motor, float duty_cycle) {
  switch (motor) {
  case MOTOR_L: OCR1B = OCR1A*duty_cycle; break;
  case MOTOR_R: OCR1C = OCR1A*duty_cycle; break;
  default: break;
  }
}

/* disable motors */
void disable_motors() { TCCR1B &= ~(1 << CS10); }

/* enable motors */
void enable_motors() { TCCR1B |= (1 << CS10); }
