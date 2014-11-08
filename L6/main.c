#include <avr/eeprom.h>
#include "m_general.h"
#include "m_bus.h"
#include "m_bus.c"
#include "m_rf.h"
#include "m_rf.c"
#include "m_imu.h"
#include "m_imu.c"
#include "m_usb.h"
#include "m_usb.c"
#include <stdint.h>
#include <math.h>

#define PI 3.14159265359
#define g 9.81
#define ACC_SCALE (g/16384)
#define GYRO_SCALE (125*PI/180/16384)
#define DUTY_LIMIT 0.9

#define packet_length 8
#define channel 2
#define address 0x10

typedef enum {
  KP_POS = 46,
  KD_POS = KP_POS+sizeof(float),
  KI_POS = KD_POS+sizeof(float),
  INT_CAP_POS = KI_POS+sizeof(float)
};

volatile uint64_t tim3_ovf;
volatile int read_flag;

void init();

int main(void) {
  init();
  
  /* radio module initialize */
  char packet[packet_length] = {0,0,0,0,0,0,0,0};
  char* buffer=&packet[0];
  char success;

  /* set gains */
  eeprom_update_float((float*)KP_POS, 4.2);
  eeprom_update_float((float*)KD_POS, 0.23);
  eeprom_update_float((float*)KI_POS, 0.17);
  eeprom_update_float((float*)INT_CAP_POS, 50.0);

  /* initialize physical parameters */
  float x_acc = 0.0, z_acc = 0.0, pitch = 0.0, prev_err = 0.0, pitch_gyro = 0.0, d_err = 0.0;
  float alpha_lp = 0.0, alpha_hp = 0.0;
  float alpha = 0.98;
  float cutoff_low = 30.0, cutoff_high = 0.1;
  float RC_low = 1/(cutoff_low*2*PI);
  float RC_high = 1/(cutoff_high*2*PI);
  float dt = 0.0;
  float duty_cycle = 0.0;
  float pitch_d = -PI/180;
  float err = 0.0, err_int = 0.0;
  float command = 0.0;

  /* obtain controller gains */
  float Kp = eeprom_read_float((float*)KP_POS),
    Kd = eeprom_read_float((float*)KD_POS),
    Ki = eeprom_read_float((float*)KI_POS),
    int_cap = eeprom_read_float((float*)INT_CAP_POS);
  
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

  /* while(1) { */
  /*   m_usb_tx_int((int)(Kp*10)); */
  /*   m_usb_tx_string(" "); */
  /*   m_usb_tx_int((int)(Kd*10)); */
  /*   m_usb_tx_string(" "); */
  /*   m_usb_tx_int((int)(Ki*10)); */
  /*   m_usb_tx_string(" "); */
  /*   m_usb_tx_int((int)(int_cap*10)); */
  /*   m_usb_tx_string("\n\r"); */
  /* } */

  /* main loop */
  while(1) {
    if (read_flag) {
      success = m_rf_read(buffer, packet_length);

      if (success) {
	m_green(ON);
	m_red(OFF);
      }
      else {
	m_red(OFF);
	m_red(ON);
      }
      Kp = (float)(256*packet[0]+packet[1])/1000.0;
      Kd = (float)(256*packet[2]+packet[3])/1000.0;
      Ki = (float)(256*packet[4]+packet[5])/1000.0;
      int_cap = (float)(256*packet[6]+packet[7])/1000.0;

      eeprom_update_float((float*)KP_POS, Kp);
      eeprom_update_float((float*)KD_POS, Kd);
      eeprom_update_float((float*)KI_POS, Ki);
      eeprom_update_float((float*)INT_CAP_POS, int_cap);
      
      read_flag = 0;
    }

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
      pitch += dt*GYRO_SCALE*data[3];
      if (abs(x_acc)+abs(z_acc)> -2*g && abs(x_acc)+abs(z_acc)< 2*g)
	pitch = pitch*alpha + atan2f(x_acc,z_acc)*(1-alpha);

      /* calculate error, error derivative, and integral error */
      err = pitch_d - pitch;
      d_err = (err - prev_err)/dt;
      prev_err = err;
      err_int = (fabs(err_int + err) >= int_cap) ? int_cap*(2*(err_int+err > 0)-1) : err_int+err;

      /* set duty cycle */
      command = Kp*err + Kd*d_err + Ki*err_int;
      if (fabs(command) > DUTY_LIMIT)
	duty_cycle = DUTY_LIMIT;
      else
	duty_cycle = fabs(command);

      /* set motor direction */
      if (command > 0) {
	PORTB |= (1 << 5);
	PORTC &= ~(1 << 6);
      }
      else {
	PORTB &= ~(1 << 5);
	PORTC |= (1 << 6);
      }
      OCR1B = duty_cycle*OCR1A;
      
      if (fabs(pitch) > 80*PI/180) {
	OCR1B = 0;
	while(1);
      }
      //m_usb_tx_int((int)(err_int));
      //m_usb_tx_int((int)(GYRO_SCALE*data[3]));
      //m_usb_tx_string("\n\r");
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
  while(!m_rf_open(channel, address, packet_length));
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

ISR(INT2_vect) {
  read_flag = 1;
}
