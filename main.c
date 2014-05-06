/*
ECE 395 Quadcopter
main.c
*/

#include "driver_config.h"
#include "ssp.h"
#include "cpu_lpc1000.h"
#include "uart.h"
#include "mpu.h"
#include "kalman.h"
#include "i2c.h"
#include "timer32.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <rt_misc.h>
#include <string.h>
#include "pid.h"
#include "motors.h"

#define TEST_THRUST_COEFF 0.5

/* angle variables */
float acc_angle_x, acc_angle_y, acc_angle_z;
float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
float kal_roll_angle, kal_pitch_angle, kal_yaw_angle;

/* pid vars */
float pid_roll_output, pid_pitch_output, pid_yaw_output;

/* motor speeds */
float motor_speed_1, motor_speed_2, motor_speed_3, motor_speed_4;
float motor_speed_pitch, motor_speed_roll, motor_speed_yaw;

/* time variables. all are in seconds */
float dt, now, startTime, lastReadTime, lastBlinkTime;	

/* kalman filters for each axis */
kalman_t kal_roll, kal_pitch, kal_yaw;

/* 6 pids, one to stabilize roll pitch and yaw and one to stabilize movement */
pid_t pid_roll, pid_pitch, pid_yaw;
pid_t pid_x, pid_y, pid_z;

int i;
float max_acc = 0;
int ledState = 0;

void configureGPIO()
{
		//enable clocks to GPIO block
		LPC_SYSCON->SYSAHBCLKCTRL |= (1UL <<  6);
		LPC_SYSCON->SYSAHBCLKCTRL |= (1UL <<  16);
	
		//set port 0_7 to output (high current drain in LPC1114)
    LPC_GPIO0->DIR |= (1<<7);
		//set port 1_4 to input
		LPC_GPIO1->DIR &= ~(1<<4);

}

void ledOn()
{
		LPC_GPIO0->DATA &= ~(1<<7);
		ledState = 1;
}

void ledOff()
{						 
		LPC_GPIO0->DATA |= (1<<7);
		ledState = 0;
}

void toggleLed() {
		if(ledState) {
				ledOff();
		} else {
				ledOn();
		}
}

int main()
{
	configureGPIO();

	delay32Ms(1, 1000);  //allow time for devices to power up
	
	/* initialize serial */
	UARTInit(_100KHZ);
	
	/* initialize MPU */
	I2CInit(I2CMASTER);
	while(MPU6050_init() != 0)
	{
		delay32Ms(1,1);
	} 
	MPU6050_setZero();
	
	/* Initalize motors */
	motors_init();
	
	/* Initalize all the kalman filters */
	kalman_init(&kal_roll);
	kalman_init(&kal_pitch);
	kalman_init(&kal_yaw);
	
	/* Initalize the PID's */
	pid_init(&pid_roll);
	pid_init(&pid_pitch);
	pid_init(&pid_yaw);
	pid_init(&pid_x);
	pid_init(&pid_y);
	pid_init(&pid_z);
	
	/* Set the PID's setpoints to 0 since we're just worrying about stabilization*/
	pid_roll.Setpoint = 0;
	pid_pitch.Setpoint = 0;
	pid_yaw.Setpoint = 0;
	pid_x.Setpoint = 0;
	pid_y.Setpoint = 0;
	pid_z.Setpoint = 0;
	
	/* Set PID's max and min outputs */
	SetOutputLimits(&pid_roll, PID_MIN_OUT, PID_MAX_OUT);
	SetOutputLimits(&pid_pitch, PID_MIN_OUT, PID_MAX_OUT);
	SetOutputLimits(&pid_yaw, PID_MIN_OUT, PID_MAX_OUT);
	SetOutputLimits(&pid_x, PID_MIN_OUT, PID_MAX_OUT);
	SetOutputLimits(&pid_y, PID_MIN_OUT, PID_MAX_OUT);
	SetOutputLimits(&pid_z, PID_MIN_OUT, PID_MAX_OUT);
	
	/* Set the PID's P, I, and D coefficients */
	SetTunings(&pid_roll, PID_ROLL_P, PID_ROLL_I, PID_ROLL_D);
	SetTunings(&pid_pitch, PID_PITCH_P, PID_PITCH_I, PID_PITCH_D);
	SetTunings(&pid_yaw, PID_YAW_P, PID_YAW_I, PID_YAW_D);
	SetTunings(&pid_x, PID_X_P, PID_X_I, PID_X_D);
	SetTunings(&pid_y, PID_Y_P, PID_Y_I, PID_Y_D);
	SetTunings(&pid_z, PID_Z_P, PID_Z_I, PID_Z_D);
	
	/* Enable PID automatic mode */
	SetMode(&pid_roll, AUTOMATIC);
	SetMode(&pid_pitch, AUTOMATIC);
	SetMode(&pid_yaw, AUTOMATIC);
	SetMode(&pid_x, AUTOMATIC);
	SetMode(&pid_y, AUTOMATIC);
	SetMode(&pid_z, AUTOMATIC);
	
	delay32Ms(1, 1000);  //Give the PID's a second to get started
	
	/* Initalize Motors */
	
	/* Set up timer */
	i = 0; 		
	init_timer32(1, 48); //us
	enable_timer32(1);
	startTime = (float)read_timer32(0) / 1000000;
	lastReadTime = startTime;
	lastBlinkTime = startTime;
	
	/* TEST - Set motor 1's speed to 0.2 */
	//set_motor(1, 1);
	//set_motor(2, 1);
	//set_motor(3, 1);
	//set_motor(4, 1);
	
	while (1)
	{
		/* Read current accel/gyro values */
		gyro_x 	= 	MPU6050_getGyroX_degree();
		gyro_y 	= 	MPU6050_getGyroY_degree();
		gyro_z 	= 	MPU6050_getGyroZ_degree();
		acc_x 	=   MPU6050_getAccel_x();
		acc_y 	=		MPU6050_getAccel_y();
		acc_z 	= 	MPU6050_getAccel_z();
		
		/* Convert accelometer data to angles */
		acc_angle_x = atan2(acc_y, acc_z) * 180/3.14159265358979323;
		acc_angle_y = atan2(acc_x , acc_z) * 180/3.14159265358979323;
		acc_angle_z = atan2(acc_x , acc_y) * 180/3.14159265358979323;
		
		/* Calculate dt */
		now = (float)read_timer32(1) / 1000000;
		dt = now - lastReadTime;
		lastReadTime = now;
		
		/* Grab the kalman angles for roll pitch and yaw*/
		kal_roll_angle = kalman_update(&kal_roll, acc_angle_x, gyro_x, dt);
		kal_pitch_angle = kalman_update(&kal_pitch, acc_angle_y, gyro_y, dt);
		kal_yaw_angle = kalman_update(&kal_yaw, acc_angle_z, gyro_z, dt);
		
		/* Update the PID's input values */
		pid_roll.Input = kal_roll_angle;
		pid_pitch.Input = kal_pitch_angle;
		pid_yaw.Input = kal_yaw_angle;
		
		/* Perform PID calculations */
		pid_compute(&pid_roll, now);
		pid_compute(&pid_pitch, now);
		pid_compute(&pid_yaw, now);
		
		/* Grab our calculated PID values */
		pid_roll_output = pid_roll.Output;
		pid_pitch_output = pid_pitch.Output;
		pid_yaw_output = pid_yaw.Output;
		
		/* Get the new motor speeds */
		motor_speed_roll = getMotorSpeed(&pid_roll);
		motor_speed_pitch = getMotorSpeed(&pid_pitch);
		motor_speed_yaw = getMotorSpeed(&pid_yaw);
		
		motor_speed_1 = TEST_THRUST_COEFF;
		motor_speed_2 = TEST_THRUST_COEFF;
		motor_speed_3 = TEST_THRUST_COEFF;
		motor_speed_4 = TEST_THRUST_COEFF;
		
		// Pitch control
		motor_speed_1 += (0.5 * motor_speed_pitch);
		motor_speed_2 -= (0.5 * motor_speed_pitch);
		motor_speed_3 -= (0.5 * motor_speed_pitch);
		motor_speed_4 += (0.5 * motor_speed_pitch);
		
		// Roll control
		motor_speed_1 -= (0.5 * motor_speed_roll);
		motor_speed_2 -= (0.5 * motor_speed_roll);
		motor_speed_3 += (0.5 * motor_speed_roll);
		motor_speed_4 += (0.5 * motor_speed_roll);
		
		if(motor_speed_1 > 1) {
				motor_speed_1 = 1;
		} else if(motor_speed_1 < 0) {
				motor_speed_1 = 0;
		}
		if(motor_speed_2 > 1) {
				motor_speed_2 = 1;
		} else if(motor_speed_2 < 0) {
				motor_speed_2 = 0;
		}
		if(motor_speed_3 > 1) {
				motor_speed_3 = 1;
		} else if(motor_speed_3 < 0) {
				motor_speed_3 = 0;
		}
		if(motor_speed_4 > 1) {
				motor_speed_4 = 1;
		} else if(motor_speed_4 < 0) {
				motor_speed_4 = 0;
		}
		
		/* Set all the motors */
		set_motor(1, motor_speed_1);
		set_motor(2, motor_speed_2);
		set_motor(3, motor_speed_3);
		set_motor(4, motor_speed_4);
		
		/* Blink the led to show its working! */
		if(now - lastBlinkTime > 1) {
				toggleLed();
				lastBlinkTime = now;
		}
	}
}
