#ifndef INCLUDE_CHASSIS_H
#define INCLUDE_CHASSIS_H

#include "config.h"
#include "pid.h"

#ifndef LEFT
#define LEFT 0
#endif

#ifndef RIGHT
#define RIGHT 1
#endif

#ifndef FORWARD
#define FORWARD 1
#endif

#ifndef BACKWARD
#define BACKWARD !(FORWARD)
#endif

/// Default encoder max speed
/// Used as a divident in speed formula
#define ENC_MAX_SPEED 128000

void motors_init();
void motors_write(int left, int right);

void enc_init();
int enc_getSpeed(int side);
long enc_getPath(int side);
void enc_reset(); 
void enc_reset(int side);

void chassis_init(const pid_config &spd, const pid_config &cross);
void chassis_write(int left, int right, bool reset = false);

#endif
