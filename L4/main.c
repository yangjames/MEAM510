#include <stdint.h>
#include "m_general.h"
#include "m_bus.c"
#include "m_rf.c"

#define PACKET_LENGTH 0x03
#define ADDRESS 0x4D
#define CHANNEL 0x01
#define RESOLUTION_50 50

char buf[3] = {0,0,0};
volatile int idx;
volatile int timer3_ovf;

int sin_table_50[50]={0x80,0x8f,0x9f,0xae,0xbd,0xca,0xd7,0xe2,0xeb,0xf3,
		      0xf9,0xfd,0xff,0xff,0xfd,0xf9,0xf3,0xeb,0xe2,0xd7,
		      0xca,0xbd,0xae,0x9f,0x8f,0x80,0x70,0x60,0x51,0x42,
		      0x35,0x28,0x1d,0x14,0xc,0x6,0x2,0x0,0x0,0x2,
		      0x6,0xc,0x14,0x1d,0x28,0x35,0x42,0x51,0x60,0x70};

ISR(TIMER3_OVF_vect) {
  timer3_ovf++;
  if (timer3_ovf*65536+TCNT3 >= 160000*buf[2]) {
    TCCR1B &= ~(1 << CS10);
    TCNT1 = 0;
    TCCR3B &= ~(1 << CS30);
    TCNT3 = 0;
    timer3_ovf = 0;
    PORTB &= ~(1 << 6);
  }
}

ISR(TIMER1_OVF_vect) {
  idx++;
  idx %= RESOLUTION_50;
}

ISR(INT2_vect) {
  m_rf_read(buf, PACKET_LENGTH);
  TCCR1B |= (1 << CS10);
  TCCR3B |= (1 << CS30);
  PORTE ^= (1 << 6);
}

int main(void) {
  idx = 0;

  /* disable global interrupts */
  cli();

  /* intialize i2c and rf modules */
  m_bus_init();
  m_rf_open(CHANNEL, ADDRESS, PACKET_LENGTH);

  /* configure output pins */
  DDRB |= (1 << 6); // pwm pin
  PORTB &= ~(1 << 6);

  DDRE |= 1 << 6; // rf status pin
  PORTE |= (1 << 6);

  /* no prescaler. set clock to full 16MHz */
  CLKPR = 1 << CLKPCE;
  CLKPR = 0x00;

  /* configure timer 1 */
  TCCR1A = 0x00; // reset timer configuration registers
  TCCR1B = 0x00;

  TCCR1A |= (1 << WGM10) | (1 << WGM11); // set to mode 15 waveform gen
  TCCR1B |= (1 << WGM12) | (1 << WGM13);

  TCCR1A |= (1 << COM1B1); // clear at OCR1B, set at OCR1A rollover
  TCCR1A &= ~(1 << COM1B0);

  TCNT1 = 0x00; // enable overflow interrupt
  OCR1A = 0; // set output compare A default value
  OCR1B = 0; // set output compare B default value

  TIMSK1 = 0x01; // enable timer overflow interrupt
  //TCCR1B |= (1 << CS10); // enable timer

  /* configure timer 3 */
  TCCR3A = 0x00;
  TCCR3B = 0x00;
  TIMSK3 = 0x01; // enable timer overflow interrupt
  
  /* enable global interrupts */
  sei();

  /* local variables */
  /* main loop */
  while(1) {
    OCR1A = 160000000/(*(int*)buf*RESOLUTION_50);
    OCR1B = (long)sin_table_50[idx]*(long)OCR1A/(long)0xff;
  }

  /* return */
  return 0;
}
