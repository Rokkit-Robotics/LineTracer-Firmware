#ifndef PID_H
#define PID_H

#include <stdint.h>

struct pid_config {
  float p;
  float i;
  float d;
  long i_max;
};

struct pid_state {
  int last_error;
  long integral;
};

float pid_step(long error, long dt, const pid_config &p, pid_state &s);
void pid_reset(pid_state &s);

#endif
