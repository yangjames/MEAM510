#include "m_includes.h"
#include "config.h"
#include "motor.h"
#include "solenoid.h"
#include "m_usb.h"
#include "localization.h"

/* predeclarations */
void init();
void parse_radio_data();

/* global variables */
uint8_t radio_buf[PACKET_LENGTH];
volatile char radio_flag;
volatile uint64_t tim3_ovf;
volatile char state = INITIALIZE;
volatile char team = N_INIT;
volatile char go_flag = 0;
volatile char motor_disable = 0;
float pos_d[2] = {0.0, 0.0};

/* main function */
int main(void) {
  /* system initialize */
  init();

  /* localization variables */
  uint16_t constellation[12];
  uint16_t ordered_points[4][2];
  float center[2], height, orientation, 
    x_const_raw = 0.0, y_const_raw = 0.0,
    x_const = 0.0, x_const_dot = 0.0, x_const_ddot = 0.0,
    y_const = 0.0, y_const_dot = 0.0, y_const_ddot = 0.0,
    yaw_const = 0.0;
  int update = 0;

  /* controls and filtering variables */
  float dt = 0.0;
  int imu[9];
  float x_ddot = 0.0, x_ddot_prev = 0.0, x_dot = 0.0, x_dot_prev = 0.0, x = 0.0, x_prev = 0.0, // x variables
    y_ddot = 0.0, y_ddot_prev = 0.0, y_dot = 0.0, y_dot_prev = 0.0, y = 0.0, y_prev = 0.0, // y variables
    yaw = 0.0, yaw_dot = 0.0, prev_yaw_dot = 0.0, // yaw variables
    pitch = 0.0, pitch_dot = 0.0, prev_pitch_dot = 0.0, // pitch variables
    roll = 0.0, roll_dot = 0.0, prev_roll_dot = 0.0, // roll variables
    alpha_lp, cutoff_low = 3.0, RC_low = 1/(cutoff_low*2*PI), // low-pass filter variables
    alpha_hp_acc, cutoff_high_acc = 0.00001, RC_high_acc = 1/(cutoff_high_acc*2*PI),
    alpha_hp, cutoff_high = 0.001, RC_high = 1/(cutoff_high*2*PI),
    x_ddot_local, y_ddot_local;


  float yaw_d = 0.0;
  float yaw_err = 0.0;
  float Kp_yaw = 20.0;
  float l_duty = 0.0;
  float r_duty = 0.0;

  /* sensor fusion variables */
  float alpha_x = 0.999, alpha_xdot = 0.999, alpha_xddot = 0.999;
  float alpha_y = 0.999, alpha_ydot = 0.999, alpha_yddot = 0.999;

  /* start motors */
  enable_motors();

  /* while(1) { */
  /*   if (radio_flag) { */
  /*     radio_flag = 0; */
  /*     m_rf_read(radio_buf, PACKET_LENGTH);  */
  /*     parse_radio_data(); */
  /*   } */
  /* } */
  /* main loop */
  while(1) {
    /* check if we got a new packet */
    if (radio_flag) {
      radio_flag = 0;
      m_rf_read(radio_buf, PACKET_LENGTH); 
      parse_radio_data();
    }

    /* get delta t */
    dt = (float)(TCNT3 + tim3_ovf*65536)/16000000;
    tim3_ovf = 0;
    TCNT3 = 0;

    /* get constellation */
    alpha_lp = dt/(dt+RC_low);
    m_wii_read(constellation);
    if (match_points(constellation, ordered_points))
      if (localize(ordered_points, center, &orientation, &height))
	update = inverse_kinematics(center, &orientation, &height, &x_const_raw, &y_const_raw, &yaw_const);

    /* received good constellation data */
    if (update) {
      /* filter the x and y coordinate values */
      y_const = y_const_raw*(alpha_lp) + (1-alpha_lp)*y_const;
      x_const = x_const_raw*(alpha_lp) + (1-alpha_lp)*x_const;
      
      y_const_dot = (y_const-y)/dt;
      x_const_dot = (x_const-x)/dt;
 
      y_const_ddot = (y_const_dot - y_dot)/dt;
      x_const_ddot = (x_const_dot - x_dot)/dt;
    }

    /* process IMU data */
    if (m_imu_raw(imu)) {
      /* propagate */
      alpha_hp = RC_high/(dt+RC_high); // high pass gain
      alpha_hp_acc = RC_high_acc/(dt+RC_high_acc); // accelrometer high pass gain

      /* calculate yaw */
      yaw_dot = yaw_dot*alpha_hp + (imu[5]*GYRO_SCALE - prev_yaw_dot)*alpha_hp;
      yaw += dt*yaw_dot; // yaw angle
      if (yaw > PI)
	yaw -= 2*PI;
      if (yaw <= -PI)
	yaw += 2*PI;
      prev_yaw_dot = imu[5]*GYRO_SCALE;
      
      /* calculate pitch */
      pitch_dot = pitch_dot*alpha_hp + (imu[4]*GYRO_SCALE - prev_pitch_dot)*alpha_hp;
      pitch += dt*pitch_dot;
      if (pitch > PI)
	pitch -= 2*PI;
      if (pitch <= -PI)
	pitch += 2*PI;
      prev_pitch_dot = imu[4]*GYRO_SCALE;
      if (fabs(imu[0]*GYRO_SCALE) + fabs(imu[2]*ACC_SCALE) > -2*g && fabs(imu[0]*GYRO_SCALE) + fabs(imu[2]*ACC_SCALE) < 2*g)
	pitch = pitch*0.99 - atan2f(imu[0]*ACC_SCALE,imu[2]*ACC_SCALE)*0.01;
      
      /* calculate roll */
      roll_dot = roll_dot*alpha_hp + (-imu[3]*GYRO_SCALE - prev_roll_dot)*alpha_hp;
      roll += dt*roll_dot;
      if (roll > PI)
	roll -= 2*PI;
      if (roll <= -PI)
	roll += 2*PI;
      prev_roll_dot = -imu[3]*GYRO_SCALE;
      if (fabs(imu[1]*GYRO_SCALE) + fabs(imu[2]*ACC_SCALE) > -2*g && fabs(imu[1]*GYRO_SCALE) + fabs(imu[2]*ACC_SCALE) < 2*g)
	roll = roll*0.99 - atan2f(imu[1]*ACC_SCALE,imu[2]*ACC_SCALE)*0.01;

      /* calculate local position */
      x_ddot = x_ddot*alpha_hp_acc + (100*(-ACC_SCALE*imu[0]*sin(yaw) - ACC_SCALE*imu[1]*cos(yaw)) - x_ddot_prev)*alpha_hp_acc;
      x_dot = x_ddot*dt + x_dot_prev;
      x = 1/2*dt*dt*x_ddot_prev + x_dot_prev*dt +x_prev;

      x_ddot_prev = 100*(-ACC_SCALE*imu[0]*sin(yaw) - ACC_SCALE*imu[1]*cos(yaw));
      x_dot_prev = x_dot;
      x_prev = x;

      y_ddot = y_ddot*alpha_hp_acc + (100*(ACC_SCALE*imu[0]*cos(yaw) - ACC_SCALE*imu[1]*sin(yaw)) - y_ddot_prev)*alpha_hp_acc;
      y_dot = y_ddot*dt + y_dot_prev;
      y = 1/2*dt*dt*y_ddot_prev + y_dot_prev*dt + y_prev;

      y_ddot_prev = 100*(ACC_SCALE*imu[0]*cos(yaw) - ACC_SCALE*imu[1]*sin(yaw));
      y_dot_prev = y_dot;
      y_prev = y;

      /* update */
      if (update) {
	m_red(ON);
	x = x_const;
	y = y_const;
	x_dot = x_const_dot;
	y_dot = y_const_dot;
	x_ddot = x_const_ddot;
	y_ddot = y_const_ddot;
	if (yaw_const*180/PI < 178 && yaw_const*180/PI > -178)
	  yaw = yaw_const*0.01 + yaw*0.99;
	else
	  yaw = yaw_const;

	/* state machine */
	switch (state) {
	case INITIALIZE:
	  if (x > 0 && team == N_INIT) {
	    team = RED;
	    pos_d[0] = -100.0;
	    pos_d[1] = 0.0;
	  }
	  if (x < 0 && team == N_INIT) {
	    team = BLUE;
	    pos_d[0] = 100.0;
	    pos_d[1] = 0.0;
	  }
	  if (team != N_INIT && go_flag) state = GO;
	  break;
	case GO: {
	  /* controls */
	  yaw_d = atan2(-pos_d[0]+x,pos_d[1]-y);
	  yaw_err = yaw_d - yaw;
	  yaw_err = yaw_err > PI ? yaw_err - 2*PI : (yaw_err < -PI ? yaw_err+2*PI : yaw_err);

	  if (yaw_err < -PI/4 && !motor_disable) {
	    set_direction(MOTOR_L, FORWARD);
	    set_direction(MOTOR_R, BACKWARD);
	    set_duty_cycle(MOTOR_L, 1.0);
	    set_duty_cycle(MOTOR_R, 1.0);
	  }
	  else if (yaw_err > PI/4 && !motor_disable) {
	    set_direction(MOTOR_L, BACKWARD);
	    set_direction(MOTOR_R, FORWARD);
	    set_duty_cycle(MOTOR_L, 1.0);
	    set_duty_cycle(MOTOR_R, 1.0);
	  }
	  else if (yaw_err < 0.0 && !motor_disable) {
	    set_direction(MOTOR_L, FORWARD);
	    set_direction(MOTOR_R, FORWARD);
	    set_duty_cycle(MOTOR_L, 1.0);
	    r_duty = fabs(yaw_err)/(2*PI)*Kp_yaw;
	    if (r_duty > 1.0)
	      r_duty = 1.0;
	    set_duty_cycle(MOTOR_R, 1.0-r_duty);
	  }
	  else if (yaw_err > 0.0 && !motor_disable) {
	    set_direction(MOTOR_L, FORWARD);
	    set_direction(MOTOR_L, FORWARD);
	    l_duty = fabs(yaw_err)/(2*PI)*Kp_yaw;
	    if (l_duty > 1.0)
	      l_duty = 1.0;
	    set_duty_cycle(MOTOR_L, 1.0-l_duty);
	    set_duty_cycle(MOTOR_R, 1.0);
	  }
	  if (fabs(pos_d[0] - x) < 5.0 && fabs(pos_d[1] - y) < 5.0) {
	    state = STOP;
	    motor_disable = 1;
	    m_green(OFF);
	    set_direction(MOTOR_L, BRAKE);
	    set_direction(MOTOR_R, BRAKE);
	    set_duty_cycle(MOTOR_L, 0.0);
	    set_duty_cycle(MOTOR_R, 0.0);
	    //disable_motors();
	  }
	  break;
	}
	case STOP: {
	  motor_disable = 1;
	  m_green(OFF);
	  set_direction(MOTOR_L, BRAKE);
	  set_direction(MOTOR_R, BRAKE);
	  set_duty_cycle(MOTOR_L, 0.0);
	  set_duty_cycle(MOTOR_R, 0.0);
	  go_flag = 0;
	  motor_disable = 0;
	  team = N_INIT;
	  state = INITIALIZE;
	  //disable_motors();
	}
	default:
	  break;
	}
      }
      else {
	m_red(OFF);
      }
    }
    /* reset constellation update flag */
    update = 0;
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
  m_usb_init();
  m_bus_init();
  while(!m_wii_open());
  while(!m_imu_init(0,0));
  while(!m_rf_open(CHANNEL, ADDRESS, PACKET_LENGTH));
  m_red(ON);

  /* initialize drivers */
  init_motor_drivers();
  init_solenoid();
  m_green(ON);

  /* intialize system timer */
  TCCR3A = 0x00; // normal compare output mode operation
  TCNT3 = 0x00; // intial timer value = 0
  TIMSK3 = 0x01; // enable overflow interrupt
  TCCR3B = 0x01; // enable timer

  sei();
}

void parse_radio_data() {
  switch(radio_buf[0]) {
  case 0xA0: m_green(TOGGLE) break; // test
  case 0xA1: {
    if (team != N_INIT) {
      m_green(TOGGLE);
      state = GO;
    }
    go_flag = 1;
    break; // play
  }
  case 0xA2: break; // goal R
  case 0xA3: break; // goal B
  case 0xA4: break; // pause
  case 0xA5: break;
  case 0xA6: break; // half time
  case 0xA7: break; // game over
  case 0xA8: break; // enemy positions
  case 0xB0:
    m_green(TOGGLE);
    pos_d[0] = (float)(radio_buf[1]*256 + radio_buf[2])/10 - 115.0;
    pos_d[1] = (float)(radio_buf[3]*256 + radio_buf[4])/10 - 60.0;
    state = GO;
    break;
  default: break;
  }
}

/* system timer overflow */
ISR(TIMER3_OVF_vect) {tim3_ovf++;}

/* read in radio packet */
ISR(INT2_vect) {

  radio_flag = 1;
}
