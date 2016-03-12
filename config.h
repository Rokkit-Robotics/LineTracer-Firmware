#ifndef CONFIG_H
#define CONFIG_H

/**
 * Line sensors config
 */
#define NUM_SENSORS 6
#define TIMEOUT 1000


/**
 * HC-SR04 sensors config
 */


/**
 * Chassis config
 */
#define MOTOR_LEFT_DIR 4
#define MOTOR_LEFT_PWM 5
#define MOTOR_RIGHT_PWM 6
#define MOTOR_RIGHT_DIR 7

#define MOTOR_LEFT_FWD 0
#define MOTOR_RIGHT_FWD 0

#define ENCODER_LEFT_A 2
#define ENCODER_LEFT_B 11
#define ENCODER_RIGHT_A 3
#define ENCODER_RIGHT_B 12

#define ENCODER_LEFT_A_PIN PIND
#define ENCODER_LEFT_A_BIT (1<<2)
#define ENCODER_LEFT_B_PIN PINB
#define ENCODER_LEFT_B_BIT (1<<3)

#define ENCODER_RIGHT_A_PIN PIND
#define ENCODER_RIGHT_A_BIT (1<<3)
#define ENCODER_RIGHT_B_PIN PINB
#define ENCODER_RIGHT_B_BIT (1<<4)

#define ENCODER_INV_LEFT 1
//#define ENCODER_INV_RIGHT 1

#endif
