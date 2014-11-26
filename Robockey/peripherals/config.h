#pragma once
#include "m_includes.h"
#include <stdint.h>

/* localization values and definitions */
#define PI 3.14159265359
#define g 9.81
#define ACC_SCALE (g/16384)
#define GYRO_SCALE (125*PI/180/16384)

/* motor pins and definitions */
#define MOTOR_DDR_PORT DDRB
#define MOTOR_PORT PORTB
#define L_EN 6
#define R_EN 7
#define L_DIR_1 0
#define L_DIR_2 1
#define R_DIR_1 2
#define R_DIR_2 3

enum {
  MOTOR_L = 1,
  MOTOR_R
};

enum {
  FORWARD = 1,
  BACKWARD,
  BRAKE
};

/* status LED pins */
#define STATUS_PORT PORTD
#define STATUS_1 3
#define STATUS_2 5

/* solenoid pins */
#define KICKER_DDR_PORT DDRD
#define KICKER_PORT PORTD
#define KICKER 7

/* radio parameters */
#define ADDRESS 0x04
#define CHANNEL 0x01
#define PACKET_LENGTH 10

/* state machine */
typedef enum {
  INITIALIZE = 1,
  GO
};

typedef enum {
  RED = 1,
  BLUE,
  N_INIT
};
