/*
 * pid.h
 * Header file for a pid controller implementation
 */

#ifndef PID_H_
#define PID_H_

#include "type.h"

#define MANUAL 0
#define AUTOMATIC 1
 
#define DIRECT 0
#define REVERSE 1

/* Various PID constants. Change as needed */
#define PID_ROLL_P 2
#define PID_ROLL_I 0.0001
#define PID_ROLL_D 0.0001

#define PID_PITCH_P 2
#define PID_PITCH_I 0.0001
#define PID_PITCH_D 0.0001

#define PID_YAW_P 2
#define PID_YAW_I 0.0001
#define PID_YAW_D 0.0001

#define PID_X_P 0.1
#define PID_X_I 0
#define PID_X_D 0

#define PID_Y_P 0.1
#define PID_Y_I 0
#define PID_Y_D 0

#define PID_Z_P 0.1
#define PID_Z_I 0
#define PID_Z_D 0

#define PID_MAX_OUT 20.0
#define PID_MIN_OUT -20.0

typedef struct pid_t {
		/* working variables */
		unsigned long lastTime;
		float Input, Output, Setpoint;
		float ITerm, lastInput;
		float kp, ki, kd;
		float SampleTime;
		float outMin, outMax;
		int inAuto;
} pid_t;

/* Public fuctions for a pid */
void pid_init(pid_t* pid);
void pid_compute(pid_t* pid, float timeNow);
void SetTunings(pid_t* pid, float Kp, float Ki, float Kd);
void SetSampleTime(pid_t* pid, int NewSampleTime);
void SetOutputLimits(pid_t* pid, float Min, float Max);
void SetMode(pid_t* pid, int Mode);
void SetControllerDirection(pid_t* pid, int Direction);
float getMotorSpeed(pid_t* pid);

#endif /* PID_H_ */
