#pragma once
#include "config.h"

void init_motor_drivers(void); // setup timers for motor driving
void set_direction(int motor, int direction); // set direction of motor
void set_duty_cycle(int motor, float duty_cycle); // set duty cycle of motor
void disable_motors(void); // disable motors
void enable_motors(void); // enable motors
