/* 
 * pid.c
 * C code for a PID implementation for a quadcopter
 *
 * Eddie Hsu, Spring 2014
 *
 */

#include "pid.h"
 
int controllerDirection = DIRECT;

void pid_compute(pid_t* pid, float timeNow)
{
		float error, dInput, elapsedTime;
	
		// Return if in auto mode
		if(!pid->inAuto) return;
	
		elapsedTime = timeNow - pid->lastTime;
		if(elapsedTime >= pid->SampleTime)
		{
				/* Compute all the working error variables */
				error = pid->Setpoint - pid->Input;
				pid->ITerm+= (pid->ki * error);
				if(pid->ITerm > pid->outMax) pid->ITerm= pid->outMax;
				else if(pid->ITerm < pid->outMin) pid->ITerm = pid->outMin;
				dInput = (pid->Input - pid->lastInput);
	 
				/* Compute PID Output */
				pid->Output = pid->kp * error + pid->ITerm - pid->kd * dInput;
				if(pid->Output > pid->outMax) pid->Output = pid->outMax;
				else if(pid->Output < pid->outMin) pid->Output = pid->outMin;
	 
				/* Remember some variables for next time */
				pid->lastInput = pid->Input;
				pid->lastTime = timeNow;
		}
}
 
void SetTunings(pid_t* pid, float Kp, float Ki, float Kd)
{
		float SampleTimeInSec;
	
		if (Kp<0 || Ki<0|| Kd<0) return;
 
		SampleTimeInSec = ((double)pid->SampleTime)/1000;
		pid->kp = Kp;
		pid->ki = Ki * SampleTimeInSec;
		pid->kd = Kd / SampleTimeInSec;
 
		if(controllerDirection ==REVERSE)
		{
				pid->kp = (0 - pid->kp);
				pid->ki = (0 - pid->ki);
				pid->kd = (0 - pid->kd);
		}
}
 
void SetSampleTime(pid_t* pid, int NewSampleTime)
{
		if (NewSampleTime > 0)
		{
				double ratio = (double)NewSampleTime / (double)pid->SampleTime;
				pid->ki *= ratio;
				pid->kd /= ratio;
				pid->SampleTime = (unsigned long)NewSampleTime;
		}
}
 
void SetOutputLimits(pid_t* pid, float Min, float Max)
{
   if(Min > Max) return;
   pid->outMin = Min;
   pid->outMax = Max;
 
   if(pid->Output > pid->outMax) pid->Output = pid->outMax;
   else if(pid->Output < pid->outMin) pid->Output = pid->outMin;
 
   if(pid->ITerm > pid->outMax) pid->ITerm = pid->outMax;
   else if(pid->ITerm < pid->outMin) pid->ITerm = pid->outMin;
}
 
void SetMode(pid_t* pid, int Mode)
{
    int newAuto = (Mode == AUTOMATIC);
    if(newAuto == !pid->inAuto)
    {
        pid_init(pid);
    }
    pid->inAuto = newAuto;
}
 
void pid_init(pid_t* pid)
{
		pid->SampleTime = 0.001;
		pid->inAuto = 0;
		pid->lastInput = pid->Input;
		pid->ITerm = pid->Output;
		if(pid->ITerm > pid->outMax) pid->ITerm = pid->outMax;
		else if(pid->ITerm < pid->outMin) pid->ITerm = pid->outMin;
}
 
void SetControllerDirection(pid_t* pid, int Direction)
{
		controllerDirection = Direction;
}

float getMotorSpeed(pid_t* pid) {
		float motorSpeed;
		motorSpeed = pid->Output / PID_MAX_OUT;
		if(motorSpeed < -1) {
				motorSpeed = -1;
		} else if(motorSpeed > 1) {
				motorSpeed = 1;
		}
		
		return motorSpeed;
}
