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


/* main function */
int main(void) {
  /* system initialize */
  init();

  /* localization variables */
  uint16_t constellation[12];
  uint16_t ordered_points[4][2];
  float center[2], height, orientation, x_const, y_const, yaw_const;

  /* controls and filtering variables */
  float dt = 0.0;
  int imu[9];
  float x_ddot = 0.0, x_dot = 0.0, x = 0.0, x_ddot_prev = 0.0,
    y_ddot = 0.0, y_dot = 0.0, y = 0.0, y_ddot_prev = 0.0,
    yaw = 0.0, yaw_dot = 0.0, prev_yaw_dot = 0.0,
    /*alpha_lp,*/ alpha_hp,
    cutoff_high = 0.001/*, cutoff_low = 30.0*/;
  float RC_high = 1/(cutoff_high*2*PI);
  /*float RC_low = 1/(cutoff_low*2*PI);*/

  /* main loop */
  while(1) {
    /* check if we got a new packet */
    if (radio_flag) {
      radio_flag = 0;
      parse_radio_data();
    }

    /* get delta t */
    dt = (float)(TCNT3 + tim3_ovf*65536)/16000000;
    tim3_ovf = 0;
    TCNT3 = 0;

    /* get constellation */
    m_wii_read(constellation);
    match_points(constellation, ordered_points);
    localize(ordered_points, center, &orientation, &height);
    inverse_kinematics(center, &orientation, &height, &x_const, &y_const, &yaw_const);

    //m_usb_tx_string("B,");
    m_usb_tx_int(ordered_points[0][0]);
    m_usb_tx_string(",");
    m_usb_tx_int(ordered_points[0][1]);
    m_usb_tx_string(",");
    m_usb_tx_int(ordered_points[1][0]);
    m_usb_tx_string(",");
    m_usb_tx_int(ordered_points[1][1]);
    m_usb_tx_string(",");
    m_usb_tx_int(ordered_points[2][0]);
    m_usb_tx_string(",");
    m_usb_tx_int(ordered_points[2][1]);
    m_usb_tx_string(",");
    m_usb_tx_int(ordered_points[3][0]);
    m_usb_tx_string(",");
    m_usb_tx_int(ordered_points[3][1]);
    m_usb_tx_string(",");
    m_usb_tx_int((int)x_const);
    m_usb_tx_string(",");
    m_usb_tx_int((int)y_const);
    m_usb_tx_string(",");
    m_usb_tx_int((int)(yaw_const*180/PI));
    //m_usb_tx_string(",E\n");
    m_usb_tx_string("\n");

    /* process IMU data */
    if (m_imu_raw(imu)) {
      //alpha_lp = dt/(dt+RC_low);
      alpha_hp = RC_high/(dt+RC_high);
      yaw_dot = yaw_dot*alpha_hp + (imu[5]*GYRO_SCALE - prev_yaw_dot)*alpha_hp;
      yaw += dt*yaw_dot;
      if (yaw > PI)
	yaw -= 2*PI;
      if (yaw <= -PI)
	yaw += 2*PI;
      prev_yaw_dot = imu[5]*GYRO_SCALE;

      //x_ddot = x_ddot*(1-alpha_lp) + alpha_lp*(ACC_SCALE*imu[0]*sin(yaw) + ACC_SCALE*imu[1]*cos(yaw));
      //y_ddot = y_ddot*(1-alpha_lp)+alpha_lp*(-ACC_SCALE*imu[0]*cos(yaw) + ACC_SCALE*imu[1]*sin(yaw));
      x_ddot = x_ddot*alpha_hp + (-ACC_SCALE*imu[0]*sin(yaw) - ACC_SCALE*imu[1]*cos(yaw) - x_ddot_prev)*alpha_hp;
      x_ddot_prev = -ACC_SCALE*imu[0]*sin(yaw) - ACC_SCALE*imu[1]*cos(yaw);
      x_dot += x_ddot*dt;
      x += x_dot*dt;

      y_ddot = y_ddot*alpha_hp + (ACC_SCALE*imu[0]*cos(yaw) - ACC_SCALE*imu[1]*sin(yaw) - y_ddot_prev)*alpha_hp;
      y_ddot_prev = ACC_SCALE*imu[0]*cos(yaw) - ACC_SCALE*imu[1]*sin(yaw);
      y_dot += y_ddot*dt;
      y += y_dot*dt;
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
  m_usb_init();
  m_bus_init();
  while (!m_rf_open(CHANNEL, ADDRESS, PACKET_LENGTH));
  m_wii_open();
  m_imu_init(0,0);

  /* initialize drivers */
  init_motor_drivers();
  init_solenoid();

  /* intialize system timer */
  TCCR3A = 0x00; // normal compare output mode operation
  TCNT3 = 0x00; // intial timer value = 0
  TIMSK3 = 0x01; // enable overflow interrupt
  TCCR3B = 0x01; // enable timer

  sei();
}

void parse_radio_data() {
  switch(radio_buf[0]) {
  case 0xA0: break; // test
  case 0xA1: m_red(ON); break; // play
  case 0xA2: m_green(ON); break; // goal R
  case 0xA3: break; // goal B
  case 0xA4: break; // pause
  case 0xA5: break;
  case 0xA6: break; // half time
  case 0xA7: break; // game over
  case 0xA8: break; // enemy positions
  default: break;
  }
}

/* system timer overflow */
ISR(TIMER3_OVF_vect) {tim3_ovf++;}

/* read in radio packet */
ISR(INT2_vect) {
  m_rf_read(radio_buf, PACKET_LENGTH); 
  radio_flag = 1;
}
