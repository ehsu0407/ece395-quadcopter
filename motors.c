/* 
 * motors.c
 * C code for motor control
 *
 * Eddie Hsu - Spring 2014, ECE 395
 */

#include <stdio.h>
#include "LPC11xx.h"
#include "motors.h"

/* 
 * Motor initalization function. Call this in your main function to setup appropiate ports for PWM operation.
 * Use ports 0_8, 0_9, 1_6, and 1_7
 * Port Assignment:
 * 		Motor 1 - 0_8
 *		Motor 2 - 0_9
 *		Motor 3 - 1_7
 *		Motor 4 - 1_6
 * Half the motors are driven by a 16 bit timer and half are driven by a 32 bit timer.
 */
void motors_init(void)
{	
	//Set up 16 bit timer0 for PWM operation
    LPC_IOCON->PIO0_8         = (LPC_IOCON->PIO0_8 & ~(0x3FF)) | 0x2;     //set up pin for PWM use (sec 7.4.23)
		LPC_IOCON->PIO0_9         = (LPC_IOCON->PIO0_9 & ~(0x3FF)) | 0x2;     //set up pin for PWM use (sec 7.4.24)
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<7);                                  //enable clock signal to 16 bit timer0 (sec 3.5.14)
		LPC_TMR16B0->PR           = SystemCoreClock / (1000000) - 1;      //set prescaler max value, not used here (sec 18.7.4)
    LPC_TMR16B0->MCR          = 0x80;                                     //set for reset on counter match (sec 18.7.6)
    LPC_TMR16B0->CCR          = 0;                                        //set to timer mode (sec 18.7.11)
    LPC_TMR16B0->PWMC         = 0x3;                                      //set channel zero and one to PWM mode (sec 18.7.12)
		LPC_TMR16B0->MR2          = MOTOR_PWM_PERIOD;                                     //set value for period (sec 18.7.7)
    LPC_TMR16B0->MR1          = MOTOR_MIN_SPEED;                                      //set value for duty cycle (sec 18.7.7)
		LPC_TMR16B0->MR0          = MOTOR_MIN_SPEED;                                      //set value for duty cycle (sec 18.7.7)
    LPC_TMR16B0->TCR          |= 0x3;                                     //enable and reset counter (sec 18.7.2)
    LPC_TMR16B0->TCR          &= ~(0x2);                                  //clear reset bit (sec 18.7.2)
		
	//Set up 32 bit timer0 for PWM operation
		LPC_IOCON->PIO1_6         = (LPC_IOCON->PIO1_6 & ~(0x3FF)) | 0x2;     //set up pin for PWM use (sec 7.4.24)
		LPC_IOCON->PIO1_7         = (LPC_IOCON->PIO1_7 & ~(0x3FF)) | 0x2;     //set up pin for PWM use (sec 7.4.24)		
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<9);                                  //enable clock signal to 16 bit timer0 (sec 3.5.14)
		LPC_TMR32B0->PR           = SystemCoreClock / (1000000) - 1;      //set prescaler max value, not used here (sec 18.7.4)
    LPC_TMR32B0->MCR          = 0x80;                                     //set for reset on counter match (sec 18.7.6)
    LPC_TMR32B0->CCR          = 0;                                        //set to timer mode (sec 18.7.11)
    LPC_TMR32B0->PWMC         = 0x3;                                      //set channel zero and one to PWM mode (sec 18.7.12)
		LPC_TMR32B0->MR2          = MOTOR_PWM_PERIOD;                                     //set value for period (sec 18.7.7)
    LPC_TMR32B0->MR1          = MOTOR_MIN_SPEED;                                      //set value for duty cycle (sec 18.7.7)
		LPC_TMR32B0->MR0          = MOTOR_MIN_SPEED;                                      //set value for duty cycle (sec 18.7.7)
    LPC_TMR32B0->TCR          |= 0x3;                                     //enable and reset counter (sec 18.7.2)
    LPC_TMR32B0->TCR          &= ~(0x2);                                  //clear reset bit (sec 18.7.2)
}

/* Controls the motor speeds. */
int set_motor(int motorNum, float motorSpeed) {
		
		int dutyCycle;
		/* Convert motor speed to a pwm duty cycle */
		dutyCycle = ((MOTOR_MAX_SPEED - MOTOR_MIN_SPEED) * motorSpeed) + MOTOR_MIN_SPEED;
	
		switch(motorNum) {
			case MOTOR_BOTTOM_RIGHT:			// Set the bottom right motor assigned to port 0_9
				LPC_TMR16B0->MR1 = dutyCycle;
				return 1;
			case MOTOR_BOTTOM_LEFT:			// Set the bottom left motor assigned to port 1_7
				LPC_TMR32B0->MR1 = dutyCycle;
				return 1;
			case MOTOR_TOP_LEFT:			// Set the top left motor assigned to port 1_6
				LPC_TMR32B0->MR0 = dutyCycle;
				return 1;
			case MOTOR_TOP_RIGHT:			// Set the top right motor assigned to port 0_8
				LPC_TMR16B0->MR0 = dutyCycle;
				return 1;
			default:		// Invalid motor number, return -1
				return -1;
		}
}


