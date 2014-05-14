/*
 * motors.h
 * Header file for motor control
 *
 * Eddie Hsu - Spring 2014, ECE 395
 *
 *	Diagram of motor assignment:
 *
 * 	(4)		   Front     (1)
 * 	   *              *
 * 	     *          *
 * 	       *      *
 * 	         ****
 * 	Left     *  *    Right
 * 	         **** 
 * 	       *      *
 * 	     *          *
 * 	   *              *
 * 	(3)      Back      (2)
 */

#ifndef MOTORS_H
#define MOTORS_H

#define MOTOR_INIT_SPEED 420
#define MOTOR_MIN_SPEED 400
#define MOTOR_MAX_SPEED 190
#define MOTOR_PWM_PERIOD 1520

#define MOTOR_TOP_RIGHT 1
#define MOTOR_BOTTOM_RIGHT 2
#define MOTOR_BOTTOM_LEFT 3
#define MOTOR_TOP_LEFT 4

/* Motor initalization function */
extern void motors_init(void);

/* Below are the functions used to set each individual motor's speed.
 * Each function takes a value from 0 to 1 with 0 being stopped, and 1 being full throttle */
extern int set_motor(int motorNum, float motorSpeed);

#endif /* MOTORS_H */
