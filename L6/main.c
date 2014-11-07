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
#define ACC_SCALE (9.81/16384)
#define GYRO_SCALE (125*PI/180/16384)

volatile uint64_t tim3_ovf;

void init();

int main(void) {
  init();

  /* initialize physical parameters */
  float x_acc = 0.0, z_acc = 0.0, pitch = 0.0, prev_err = 0.0, pitch_gyro = 0.0, d_err = 0.0;
  float alpha_lp = 0.0, alpha_hp = 0.0;
  float alpha = 0.98;
  float cutoff_low = 30.0, cutoff_high = 30.0;
  float RC_low = 1/(cutoff_low*2*PI);
  float RC_high = 1/(cutoff_high*2*PI);
  float Kp = 1.0, Kd = 0.0, Ki = 0.0;
  float dt = 0.0;
  float torque = 0.0, duty_cycle = 0.0, sat_duty_cycle = 0.0;
  float pitch_d = 0.0;
  float err = 0.0, err_int = 0.0;

  /* initialize IMU storage buffer */
  int data[9];

  /* establish USB connection */
  //while(!m_usb_isconnected());
  if(!m_imu_init(0, 0)) {
    PORTE |= (1 << 6);
    while(1);
  }
  else
    PORTE &= ~(1 << 6);

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
      x_acc = x_acc*alpha_lp+(1-alpha_lp)*ACC_SCALE*data[1];
      z_acc = z_acc*alpha_lp+(1-alpha_lp)*ACC_SCALE*data[2];
      pitch_gyro = pitch_gyro*alpha_hp+(1-alpha_hp)*dt*GYRO_SCALE*data[3];
      pitch = atan2f(x_acc,z_acc)*alpha + (1-alpha)*pitch_gyro;

      /* calculate error, error derivative, and integral error */
      err = pitch_d - pitch;
      d_err = (err - prev_err)/dt;
      prev_err = err;
      err_int = (fabs(err_int + err) >= 10) ? 10*(err_int > 0) : err_int+err;

      /* set motor direction */
      if (err > 0) {
	PORTB |= (1 << 5);
	PORTC &= ~(1 << 6);
      }
      else {
	PORTB &= ~(1 << 5);
	PORTC |= (1 << 6);
      }

      /* set duty cycle */
      duty_cycle = fabs(Kp*err + Kd*d_err + Ki*err_int);
      if (duty_cycle > 0.8)
	duty_cycle = 0.8;
      OCR1B = fabs(duty_cycle)*OCR1A;

      /* m_usb_tx_int(OCR1B); */
      /* m_usb_tx_string("\n\r"); */
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

  /* configure motor direction pins */
  DDRB |= (1 << 6);
  DDRB |= (1 << 5);
  DDRC |= (1 << 6);

  /* configure motor pwm timer */
  TCCR1A |= (1 << WGM10) | (1 << WGM11); // waveform 15
  TCCR1B |= (1 << WGM12) | (1 << WGM13);

  TCCR1A |= (1 << COM1B1);
  OCR1A = 1200; // set to 14kHz at 0% duty cycle
  OCR1B = 0;
  TCCR1B |= (1 << CS10);

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
