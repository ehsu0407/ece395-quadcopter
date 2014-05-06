/*
 * motors.h
 * Header file for motor control
 *
 * Eddie Hsu - Spring 2014, ECE 395
 *
 *	Diagram of motor assignment:
 *
 * 	(3)		   Front     (4)
 * 	   *              *
 * 	     *          *
 * 	       *      *
 * 	         ****
 * 	Left     *  *    Right
 * 	         **** 
 * 	       *      *
 * 	     *          *
 * 	   *              *
 * 	(2)      Back      (1)
 */

#ifndef MOTORS_H
#define MOTORS_H

#define MOTOR_MIN_SPEED 426
#define MOTOR_MAX_SPEED 228
#define MOTOR_PWM_PERIOD 1520

#define MOTOR_BOTTOM_RIGHT 1
#define MOTOR_BOTTOM_LEFT 2
#define MOTOR_TOP_LEFT 3
#define MOTOR_TOP_RIGHT 4

/* Motor initalization function */
extern void motors_init(void);

/* Below are the functions used to set each individual motor's speed.
 * Each function takes a value from 0 to 1 with 0 being stopped, and 1 being full throttle */
extern int set_motor(int motorNum, float motorSpeed);

#endif /* MOTORS_H */
