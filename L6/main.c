#include "m_general.h"
#include "m_bus.h"
#include "m_bus.c"
#include "m_imu.h"
#include "m_imu.c"
#include "m_usb.h"
#include "m_usb.c"
#include <stdint.h>
#include <math.h>

#define PI 3.14159265359
#define SCALE (9.81/16384)

volatile uint64_t tim3_ovf;

void init();

int main(void) {
  init();

  /* initialize physical parameters */
  float x_acc = 0.0, z_acc = 0.0, pitch = 0.0, pitch_gyro = 0.0;
  float alpha_lp = 0.0, alpha_hp = 0.0;
  float alpha = 0.98;
  float cutoff_low = 30.0, cutoff_high = 30.0;
  float RC_low = 1/(cutoff_low*2*PI);
  float RC_high = 1/(cutoff_high*2*PI);
  float Kp = 0.0, Kd = 0.0, Ki = 0.0;
  float dt = 0.0;

  /* initialize IMU storage buffer */
  int data[9];

  /* establish USB connection */
  while(!m_usb_isconnected());
  if(!m_imu_init(0, 0)) {
    PORTE &= ~(1 << 6);
    while(1);
  }

  /* main loop */
  while(1) {
    if (m_imu_raw(data)) {
      /* get delta t */
      dt = (float)(TCNT3 + tim3_ovf*65536)/16000000;
      tim3_ovf = 0;
      TCNT3 = 0;

      /* calculate low and high pass gain */
      alpha_lp = dt/(RC_low+dt);
      alpha_hp = RC_high/(dt+RC_high);

      /* recalculate x and z accelerations and pitch angle */
      x_acc = x_acc*alpha_lp+(1-alpha_lp)*SCALE*data[0];
      z_acc = z_acc*alpha_lp+(1-alpha_lp)*SCALE*data[2];
      pitch_gyro = pitch_gyro*alpha_hp+(1-alpha_hp)*dt*data[4];
      pitch = atan2f(x_acc,z_acc)*alpha + (1-alpha)*pitch_gyro;

      m_usb_tx_int((int)(pitch*180/PI));
      m_usb_tx_string("\n\r");
      /* m_usb_tx_int(data[0]); */
      /* m_usb_tx_string("\t"); */
      /* m_usb_tx_int(data[1]); */
      /* m_usb_tx_string("\t"); */
      /* m_usb_tx_int(data[2]); */
      /* m_usb_tx_string("\t"); */
      /* m_usb_tx_int(data[3]); */
      /* m_usb_tx_string("\t"); */
      /* m_usb_tx_int(data[4]); */
      /* m_usb_tx_string("\t"); */
      /* m_usb_tx_int(data[5]); */
      /* m_usb_tx_string("\n\r"); */
    }
  }
  return 0;
}

void init() {
  cli();

  /* set clock to full 16MHz */
  CLKPR = 1 << CLKPCE;
  CLKPR = 0x00;

  /* usb connection indicator */
  DDRE |= (1 << 6);
  PORTE |= (1 << 6);

  /* initialize i2c port and usb port */
  m_bus_init();
  m_usb_init();

  /* configure system timer */
  TCCR3A = 0x00; // normal compare output mode operation
  TCNT3 = 0x00; // intial timer value = 0
  TIMSK3 = 0x01; // enable overflow interrupt
  TCCR3B = 0x01; // enable timer

  sei();
}

ISR(TIMER3_OVF_vect) {
  tim3_ovf++;
}
