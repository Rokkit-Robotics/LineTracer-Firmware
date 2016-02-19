#include "pid.h"
#include <Arduino.h>

float pid_step(long error, long dt, const pid_config &p, pid_state &s)
{
  float sum = p.p * error;

  //Serial.print(">");
  //Serial.print(error);

  s.integral += error * dt;
  if (s.integral > p.i_max)
    s.integral = p.i_max;
  else if (s.integral < -p.i_max)
    s.integral = -p.i_max;

  sum += p.i * s.integral;

  //Serial.print("I");
  //Serial.print(sum);

  sum += p.d * (error - s.last_error) / dt;
  s.last_error = error;

  //Serial.print("D");
  //Serial.print(sum);
  //Serial.print("<");

  return sum;
}

void pid_reset(pid_state &s)
{
  s.last_error = 0;
  s.integral = 0;
}
