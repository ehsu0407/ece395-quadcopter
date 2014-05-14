/*
 * kalman.c
 * C code to kalman filter gyroscopic and accelerometer values
 *
 * Original code from http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
 * Modified by Eddie Hsu
 * ECE 395, Spring 2014
 */

#include "kalman.h"

void kalman_init(kalman_t* filter){

    /* We will set the varibles like so, these can also be tuned by the user */
  //  Q_angle = 0.001;
  //  Q_bias = 0.003;
  //  R_measure = 0.03;

		filter->Q_angle = 0.005;
		filter->Q_bias = 0.0001;
		filter->R_measure = 0.1;
    filter->bias = 0; // Reset bias
    filter->P[0][0] = 0; // Since we assume tha the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    filter->P[0][1] = 0;
    filter->P[1][0] = 0;
    filter->P[1][1] = 0;
}

float kalman_update(kalman_t* filter, float newAngle, float newRate, float dt){

    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds

		// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
		// Modified by Kristian Lauszus
		// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

		// Discrete Kalman filter time update equations - Time Update ("Predict")
		// Update xhat - Project the state ahead
		/* Step 1 */
		filter->rate = newRate - filter->bias;
		filter->angle += dt * filter->rate;

		// Update estimation error covariance - Project the error covariance ahead
		/* Step 2 */
		filter->P[0][0] += dt * (dt*filter->P[1][1] - filter->P[0][1] - filter->P[1][0] + filter->Q_angle);
		filter->P[0][1] -= dt * filter->P[1][1];
		filter->P[1][0] -= dt * filter->P[1][1];
		filter->P[1][1] += filter->Q_bias * dt;

		// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
		// Calculate Kalman gain - Compute the Kalman gain
		/* Step 4 */
		filter->S = filter->P[0][0] + filter->R_measure;
		/* Step 5 */
		filter->K[0] = filter->P[0][0] / filter->S;
		filter->K[1] = filter->P[1][0] / filter->S;

		// Calculate angle and bias - Update estimate with measurement zk (newAngle)
		/* Step 3 */
		filter->y = newAngle - filter->angle;
		/* Step 6 */
		filter->angle += filter->K[0] * filter->y;
		filter->bias += filter->K[1] * filter->y;

		// Calculate estimation error covariance - Update the error covariance
		/* Step 7 */
		filter->P[0][0] -= filter->K[0] * filter->P[0][0];
		filter->P[0][1] -= filter->K[0] * filter->P[0][1];
		filter->P[1][0] -= filter->K[1] * filter->P[0][0];
		filter->P[1][1] -= filter->K[1] * filter->P[0][1];

		return filter->angle;
}
