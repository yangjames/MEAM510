#include "config.h"
#include "solenoid.h"

volatile uint64_t tim0_ovf;

void init_solenoid() {
  /* set direction pin of solenoid kicker */
  KICKER_DDR_PORT |= (1 << KICKER);
  KICKER_PORT &= ~(1 << KICKER);

  /* configure solenoid timer */
  TIMSK0 |= (1);
}

void kick() {
  KICKER_PORT |= (1 << KICKER);
  TCCR0B |= 0x01;
}

ISR(TIMER0_OVF_vect) {
  if (tim0_ovf == 2750) {
    KICKER_PORT &= ~(1 << KICKER);
    TCCR0B &= ~(0x01);
    tim0_ovf = 0;
  }
  else
    tim0_ovf++;
}
