#include "chassis.h"
#include <Arduino.h>

#include <avr/interrupt.h>
#include <util/atomic.h>

/**
 * Encoders
 */
 
static volatile long _enc_left = 0;
static volatile long _enc_right = 0;

// encoder signal timestamps, just to calculate speed
static volatile long _enc_left_ts = 0;
static volatile long _enc_right_ts = 0;

static volatile long _enc_left_dt = 0;
static volatile long _enc_right_dt = 0;

// Timer/Counter2 interrupt
// To clear encoder data to avoid overflow
ISR(TIMER2_OVF_vect)
{
    long m = micros();

    if (m - _enc_left_ts > ENC_MAX_SPEED) {
      _enc_left_ts = -1;
      _enc_left_dt = ENC_MAX_SPEED + 10; // to get 0 in division
    }

    if (m - _enc_right_ts > ENC_MAX_SPEED) {
      _enc_right_ts = -1;
      _enc_right_dt = ENC_MAX_SPEED + 10;
    }
}

static void enc_left_interrupt()
{
  _enc_left++;

  long m = micros();

  if (_enc_left_ts > 0) {
    long enc_left_dt = m - _enc_left_ts;
    if (enc_left_dt > 3000)
      _enc_left_dt = enc_left_dt;
  }
  _enc_left_ts = m;
}

static void enc_right_interrupt()
{
  _enc_right++;

  long m = micros();

  if (_enc_right_ts > 0) {
    long enc_right_dt = m - _enc_right_ts;
    if (enc_right_dt > 3000)
      _enc_right_dt = enc_right_dt;

  }
  _enc_right_ts = m;
}

void enc_init()
{
  pinMode(ENCODER_LEFT, INPUT);
  pinMode(ENCODER_RIGHT, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), enc_left_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), enc_right_interrupt, CHANGE);

  // init Timer/Counter 2 for encoder control
  TCCR2A = 0;
  TIMSK2 |= (1<<TOIE2);
  TCCR2A |= (7 << CS20);
}

long enc_getPath(int side)
{
  if (side == LEFT)
    return _enc_left;
  else
    return _enc_right;
}

// TODO: migrate to ARM asap! Floating point will 
// be more pleasant here
int enc_getSpeed(int side)
{
  long dt;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (side == LEFT) {
      dt = _enc_left_dt;
    } else {
      dt = _enc_right_dt;
    }
  }

  // will be so suddenly
  if (dt == 0)
    dt = 1;

  return ENC_MAX_SPEED / dt;
}

void enc_reset()
{
  enc_reset(LEFT);
  enc_reset(RIGHT);
}

void enc_reset(int side)
{
  if (side == LEFT)
    _enc_left = 0;
  else
    _enc_right = 0;
}

/**
 * Motors
 */

void motors_init()
{
  pinMode(MOTOR_LEFT_DIR, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR, OUTPUT);
  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
}

void motors_writeSide(int val, int side)
{  
  if (side == LEFT) {
    if (val < 0) {
      digitalWrite(MOTOR_LEFT_DIR, !MOTOR_LEFT_FWD);
      val = -val;
    } else {
      digitalWrite(MOTOR_LEFT_DIR, MOTOR_LEFT_FWD);
    }

    analogWrite(MOTOR_LEFT_PWM, val > 255 ? 255 : val);
  } else {
    if (val < 0) {
      digitalWrite(MOTOR_RIGHT_DIR, !MOTOR_RIGHT_FWD);
      val = -val;
    } else {
      digitalWrite(MOTOR_RIGHT_DIR, MOTOR_RIGHT_FWD);
    }

    analogWrite(MOTOR_RIGHT_PWM, val > 255 ? 255 : val);    
  }
}

void motors_write(int left, int right)
{
  motors_writeSide(left, LEFT);
  motors_writeSide(right, RIGHT);
}

/**
 * Chassis - motors with stabilization
 */

static const pid_config *_spd_config, *_cross_config;
static pid_state spd_left_state, spd_right_state, cross_state;
 
void chassis_init(const pid_config &spd, const pid_config &cross)
{
  motors_init();
  enc_init();

  _spd_config = &spd;
  _cross_config = &cross;
}


void chassis_setSpeed(int spd, int side, bool reset)
{
  if (reset || spd == 0) {
    pid_reset(side == LEFT ? spd_left_state : spd_right_state);
  }
  // to calculate dt
  static long _lastCall_left = millis(), _lastCall_right = millis();
  
  int measuredSpeed = enc_getSpeed(side);

  int dt;
  if (side == LEFT) {
    dt = millis() - _lastCall_left;
    _lastCall_left = millis();
  } else {
    dt = millis() - _lastCall_right;
    _lastCall_right = millis();
  }

  if (dt == 0)
    dt = 1;

  //Serial.print("[");
  //Serial.print(measuredSpeed);
  //Serial.print("]");
  
  int power = pid_step(spd - measuredSpeed, dt, *_spd_config, side == LEFT ? spd_left_state : spd_right_state);
  //Serial.print("=");
  //Serial.print(power);
  //Serial.print("=");

  motors_writeSide(power, side);
}

void chassis_write(int left, int right, bool reset)
{
  chassis_setSpeed(left, LEFT, reset);
  chassis_setSpeed(right, RIGHT, reset);
}



