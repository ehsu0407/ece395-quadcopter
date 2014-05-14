/*
 * kalman.h
 * Header file for a kalman filter implementation
 *
 * Original code from http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
 * Modified by Eddie Hsu
 * ECE 395, Spring 2014
 */

#ifndef _Kalman_h
#define _Kalman_h

#include "kalman.h"

typedef struct kalman_t {
		/* Kalman filter variables */
		float Q_angle; // Process noise variance for the accelerometer
		float Q_bias; // Process noise variance for the gyro bias
		float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

		float angle; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
		float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
		float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

		float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
		float K[2]; // Kalman gain - This is a 2x1 matrix
		float y; // Angle difference - 1x1 matrix
		float S; // Estimate error - 1x1 matrix
} kalman_t;

void kalman_init(kalman_t* filter);
float kalman_update(kalman_t* filter, float newAngle, float newRate, float dt);

#endif
