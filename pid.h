#ifndef PID_H_
#define PID_H_

#include "type.h"

#define MANUAL 0
#define AUTOMATIC 1
 
#define DIRECT 0
#define REVERSE 1

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

#endif /* PID_H_ */
