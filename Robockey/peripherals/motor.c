#include "motor.h"

/* initialize motor drivers */
void init_motor_drivers() {
  /* set direction out for enable and direction pins */
  MOTOR_DDR_PORT |= ((1 << L_EN) | (1 << R_EN) 
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

/* set motor duty cycle */
void set_duty_cycle(int motor, float duty_cycle) {
  switch (motor) {
  case MOTOR_L: OCR1B = OCR1A*duty_cycle; break;
  case MOTOR_R: OCR1C = OCR1A*duty_cycle; break;
  default: break;
  }
}

/* set motor direction */
void set_direction(int motor, int direction) {
  switch (motor) {
  case MOTOR_L: {
    switch (direction) {
    case FORWARD:
      MOTOR_PORT |= (1 << L_DIR_1);
      MOTOR_PORT &= ~(1 << L_DIR_2);
      break;
    case BACKWARD:
      MOTOR_PORT |= (1 << L_DIR_2);
      MOTOR_PORT &= ~(1 << L_DIR_1);
      break;
    case BRAKE:
      MOTOR_PORT &= ~((1 << L_DIR_1) | (1 << L_DIR_2));
      break;
    default: break;
    }
    break;
  }
  case MOTOR_R: {
   switch (direction) {
    case FORWARD:
      MOTOR_PORT |= (1 << R_DIR_1);
      MOTOR_PORT &= ~(1 << R_DIR_2);
      break;
    case BACKWARD:
      MOTOR_PORT |= (1 << R_DIR_2);
      MOTOR_PORT &= ~(1 << R_DIR_1);
      break;
    case BRAKE:
      MOTOR_PORT &= ~((1 << R_DIR_1) | (1 << R_DIR_2));
      break;
    default: break;
    }
    break;
  }
  default: break;
  }
}

/* disable motors */
void disable_motors() {
  MOTOR_PORT &= ~((1 << L_EN) | (1 << R_EN));
  set_duty_cycle(MOTOR_L, 0.0);
  set_duty_cycle(MOTOR_R, 0.0);
  TCCR1B &= ~(1 << CS10);
  //MOTOR_PORT &= ~((1 << L_EN) | (1 << R_EN));
}

/* enable motors */
void enable_motors() { 
  TCCR1B |= (1 << CS10);
}
