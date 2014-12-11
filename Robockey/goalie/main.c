#include "m_includes.h"
#include "config.h"
#include "motor.h"
#include "solenoid.h"
#include "m_usb.h"
#include "localization.h"

/* predeclarations */
void init();
void parse_radio_data();
void read_adc(int* adc_val, uint8_t* adc_mux_idx);
void estimate_puck_location(int* adc_val, 
			    float* angle, float* distance, 
			    float* sensor_angles);

/* global variables */
uint8_t radio_buf[PACKET_LENGTH];
volatile char radio_flag;
volatile uint64_t tim3_ovf;
volatile char state = INITIALIZE;
volatile char team = N_INIT;
volatile char go_flag = 0;
volatile char motor_disable = 0;
float pos_d[2] = {0.0, 0.0};
float goal_d[2] = {0.0,0.0};

/* main function */
int main(void) {

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
    alpha_lp, cutoff_low = 3.0, RC_low = 1/(cutoff_low*2*PI), // low-pass filter variables
    alpha_hp_acc, cutoff_high_acc = 0.00001, RC_high_acc = 1/(cutoff_high_acc*2*PI),
    alpha_hp, cutoff_high = 0.001, RC_high = 1/(cutoff_high*2*PI),
    x_ddot_local, y_ddot_local;


  float yaw_d = 0.0;
  float yaw_err = 0.0;
  float Kp_yaw = 20.0;
  float l_duty = 0.0;
  float r_duty = 0.0;

  /* adc variables */
  //uint8_t adc_mux_idx[10] = {0,1,7,32,4,5,6,35,36,33};
  uint8_t adc_mux_idx[2] = {7,33};
  int adc_val[2];

  /* system initialize */
  init();
  enable_motors();

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
    }

    /* update */
    if (update) {
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
    }
    /* m_usb_tx_int((int)x); */
    /* m_usb_tx_string(" "); */
    /* m_usb_tx_int((int)y); */
    /* m_usb_tx_string(" "); */
    /* m_usb_tx_int((int)(180/PI*yaw)); */
    /* m_usb_tx_string("\n"); */

    /* read adc values */
    read_adc(adc_val, adc_mux_idx);
    
    /* state machine */
    switch (state) {
    case INITIALIZE:
      if (!(check(PINC,7)) && team == N_INIT) {
	team = BLUE;
	if (x>0){
	  pos_d[0] = -120.0;
	  pos_d[1] = 0.0;
	}
	else if (x<0){
	  pos_d[0] = 120;
	  pos_d[1] = 0.0;
	}
	set(STATUS_PORT,BLUE_LED);
      }
      
      if ((check(PINC,7)) && team == N_INIT) {
	team = RED;
	if (x>0){
	  pos_d[0] = 120.0; //come back and #define these somewhere.
	  pos_d[1] = 0.0;
	}
	else if (x<0){
	  pos_d[0] = -120.0;
	  pos_d[1] = 0.0;
	}
	set(STATUS_PORT,RED_LED);
      }
      break;
    case GO_TO_GOAL: {
      /* controls */
      yaw_d = atan2(-pos_d[0]+x,pos_d[1]-y);
      yaw_err = yaw_d - yaw;
      yaw_err = yaw_err > PI ? yaw_err - 2*PI : (yaw_err < -PI ? yaw_err+2*PI : yaw_err);

      if (yaw_err < -PI/4 && !motor_disable) {
	set_direction(MOTOR_F, RIGHT);
	set_direction(MOTOR_B, LEFT);
	set_duty_cycle(MOTOR_F, 1.0);
	set_duty_cycle(MOTOR_B, 1.0);
      }
      else if (yaw_err > PI/4 && !motor_disable) {
	set_direction(MOTOR_F, LEFT);
	set_direction(MOTOR_B, RIGHT);
	set_duty_cycle(MOTOR_F, 1.0);
	set_duty_cycle(MOTOR_B, 1.0);
      }
      else if (yaw_err < 0.0 && !motor_disable) {
	set_direction(MOTOR_F, RIGHT);
	set_direction(MOTOR_B, RIGHT);
	set_duty_cycle(MOTOR_F, 1.0);
	r_duty = fabs(yaw_err)/(2*PI)*Kp_yaw;
	if (r_duty > 1.0)
	  r_duty = 1.0;
	set_duty_cycle(MOTOR_B, 1.0-r_duty);
      }
      else if (yaw_err > 0.0 && !motor_disable) {
	set_direction(MOTOR_F, RIGHT);
	set_direction(MOTOR_B, RIGHT);
	l_duty = fabs(yaw_err)/(2*PI)*Kp_yaw;
	if (l_duty > 1.0)
	  l_duty = 1.0;
	set_duty_cycle(MOTOR_F, 1.0-l_duty);
	set_duty_cycle(MOTOR_B, 1.0);
      }
      if (fabs(pos_d[0] - x) < 5.0 && fabs(pos_d[1] - y) < 5.0) {
	state = DEFEND;
	motor_disable = 1;
	m_green(OFF);
	set_direction(MOTOR_F, BRAKE);
	set_direction(MOTOR_B, BRAKE);
	set_duty_cycle(MOTOR_F, 0.0);
	set_duty_cycle(MOTOR_B, 0.0);
	//disable_motors();
      }
      break;
    }
    case DEFEND: {
      if (pos_d[0] > 0)
	yaw_d = PI/2;
      else
	yaw_d = -PI/2;
      yaw_err = yaw_d - yaw;
      yaw_err = yaw_err > PI ? yaw_err - 2*PI : (yaw_err < -PI ? yaw_err+2*PI : yaw_err);
      if (yaw_err < -PI/4 && !motor_disable) {
	set_direction(MOTOR_F, RIGHT);
	set_direction(MOTOR_B, LEFT);
	set_duty_cycle(MOTOR_F, 1.0);
	set_duty_cycle(MOTOR_B, 1.0);
      }
      else if (yaw_err > PI/4 && !motor_disable) {
	set_direction(MOTOR_F, LEFT);
	set_direction(MOTOR_B, RIGHT);
	set_duty_cycle(MOTOR_F, 1.0);
	set_duty_cycle(MOTOR_B, 1.0);
      }
      else if (yaw_err < 0.0 && !motor_disable) {
	set_direction(MOTOR_F, RIGHT);
	set_direction(MOTOR_B, RIGHT);
	set_duty_cycle(MOTOR_F, 1.0);
	r_duty = fabs(yaw_err)/(2*PI)*Kp_yaw;
	if (r_duty > 1.0)
	  r_duty = 1.0;
	set_duty_cycle(MOTOR_B, 1.0-r_duty);
      }
      else if (yaw_err > 0.0 && !motor_disable) {
	set_direction(MOTOR_F, RIGHT);
	set_direction(MOTOR_B, RIGHT);
	l_duty = fabs(yaw_err)/(2*PI)*Kp_yaw;
	if (l_duty > 1.0)
	  l_duty = 1.0;
	set_duty_cycle(MOTOR_F, 1.0-l_duty);
	set_duty_cycle(MOTOR_B, 1.0);
      }
      if (fabs(pos_d[0] - x) < 5.0 && fabs(pos_d[1] - y) < 5.0) {
	//state = DEFEND;
	motor_disable = 1;
	m_green(OFF);
	set_direction(MOTOR_F, BRAKE);
	set_direction(MOTOR_B, BRAKE);
	set_duty_cycle(MOTOR_F, 0.0);
	set_duty_cycle(MOTOR_B, 0.0);
      }
      break;
    }
    case STOP: {
      motor_disable = 1;
      m_green(OFF);
      set_direction(MOTOR_F, BRAKE);
      set_direction(MOTOR_B, BRAKE);
      set_duty_cycle(MOTOR_F, 0.0);
      set_duty_cycle(MOTOR_B, 0.0);
      go_flag = 0;
      motor_disable = 0;
      team = N_INIT;
      state = INITIALIZE;
      //disable_motors();
    }
    default:
      break;
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
  m_disableJTAG();
  m_usb_init();
  m_bus_init();
  while(!m_wii_open());
  while(!m_imu_init(0,0));
  while(!m_rf_open(CHANNEL, ADDRESS, PACKET_LENGTH));

  /* initialize status pins */
  DDRD |= (1 << BLUE_LED) | (1 << RED_LED);

  /* initialize drivers */
  init_motor_drivers();
  init_solenoid();
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
  for (i = 0; i < 2; i++) {
    ADCSRA &= ~(1 << ADEN);
    ADMUX = (1 << REFS0) | ((adc_mux_idx[i]) & ((1 << MUX0) | (1 << MUX1) | (1 << MUX2)));
    ADCSRB = ((adc_mux_idx[i]) & (1 << MUX5));
    ADCSRA |= (1 << ADEN);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    adc_val[i] = ADCW;
  }
}

void parse_radio_data() {
  switch(radio_buf[0]) {
  case 0xA0: {
    if (team==BLUE){
      clear(STATUS_PORT,BLUE_LED);
      m_wait(250);
      set(STATUS_PORT,BLUE_LED);
      m_wait(250);
      clear(STATUS_PORT,BLUE_LED);
      m_wait(250);
      set(STATUS_PORT,BLUE_LED);
    }
    if (team==RED){
      clear(STATUS_PORT,RED_LED);
      m_wait(250);
      set(STATUS_PORT,RED_LED);
      m_wait(250);
      clear(STATUS_PORT,RED_LED);
      m_wait(250);
      set(STATUS_PORT,RED_LED);
    }
    break;  // test, come back to make it flash
  }
  case 0xA1: {
    if (team != N_INIT) {
      m_green(TOGGLE);
      state = GO_TO_GOAL;
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
    state = GO_TO_GOAL;
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
