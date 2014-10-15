#include "m_general.h"
#include "m_bus.c"
#include "m_rf.c"
#include <math.h>

#define PACKET_LENGTH 0x03
#define ADDRESS 0x4D
#define CHANNEL 0x01
#define RESOLUTION_50 50.0
#define RESOLUTION_30 30.0
#define PI 3.14159265

char buf[3];
volatile int ovf;
volatile int idx;
volatile float frequency;
float sin_table_30[31]={0.0000000, 0.2012985, 0.3943559, 0.5712682, 0.7247928, 0.8486443, 0.9377521, 0.9884683, 0.9987165, 0.9680771,
			0.8978046, 0.7907757, 0.6513725, 0.4853020, 0.2993631, 0.1011683, -0.1011683, -0.2993631, -0.4853020, -0.6513725,
			-0.7907757, -0.8978046, -0.9680771, -0.9987165, -0.9884683, -0.9377521, -0.8486443, -0.7247928, -0.5712682, -0.3943559, -0.2012985};

float sin_table_50[51]={0.0000000, 0.1253332, 0.2486899, 0.3681245, 0.4817537, 0.5877852, 0.6845471, 0.7705132, 0.8443279, 0.9048271,
		     0.9510565, 0.9822872, 0.9980267, 0.9980267, 0.9822872, 0.9510565, 0.9048271, 0.8443279, 0.7705132, 0.6845471, 
		     0.5877852, 0.4817537, 0.3681245, 0.2486899, 0.1253332, 0.0000000, -0.1253332, -0.2486899, -0.3681245, -0.4817537, 
		     -0.5877852, -0.6845471, -0.7705132, -0.8443279, -0.9048271, -0.9510565, -0.9822872, -0.9980267, -0.9980267, -0.9822872,
		     -0.9510565, -0.9048271, -0.8443279, -0.7705132, -0.6845471, -0.5877852, -0.4817537, -0.3681245, -0.2486899, -0.1253332, -0.0000000};

ISR(TIMER1_COMPA_vect) {
  PORTB |= (1 << 4);
  TCNT1 = 0;
  idx++;
  idx %= (int)(RESOLUTION_30+1);
  OCR1B = (int)((0.5*sin_table_30[idx]+0.5)*OCR1A);
}

ISR(TIMER1_COMPB_vect) {
  PORTB &= ~(1 << 4);
}

ISR(INT2_vect) {
  m_rf_read(buf, PACKET_LENGTH);
  PORTE ^= (1 << 6);
}

int main(void) {
  /* disable global interrupts */
  cli();

  /* intialize i2c and rf modules */
  m_bus_init();
  m_rf_open(CHANNEL, ADDRESS, PACKET_LENGTH);

  /* configure output pins */
  DDRB |= 1 << 4; // pwm pin
  PORTB &= ~(1 << 4);

  DDRE |= 1 << 6; // rf status pin
  PORTE |= (1 << 6);

  /* no prescaler. set clock to full 16MHz */
  CLKPR = 1 << CLKPCE;
  CLKPR = 0x00;

  /* configure timer 1 */
  TCCR1A = 0x00; // regular timer 1 output compare operation
  TCNT1 = 0x00; // set initial timer value to 0
  OCR1A = 48484; // set output compare A default value
  OCR1B = 24242; // set output compare B default value
  TIMSK1 = 0x06; // enable A and B output compares
  TCCR1B = 0x01; // enable timer

  /* enable global interrupts */
  sei();

  /* local variables */
  frequency = 440.0;
  idx = 0;
  OCR1A = 1616;//(int)(16000000.0/frequency/RESOLUTION);

  /* main loop */
  while(1);

  /* return */
  return 0;
}
