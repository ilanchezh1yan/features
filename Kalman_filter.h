/*
 * Kalman_filter.h
 *
 *  Created on: Oct 25, 2024
 *      Author: ilanc
 */

/*
 * ----------------FOR THEORITICAL REFERENCE---------------
 * https://in.mathworks.com/videos/understanding-kalman-filters-part-4-optimal-state-estimator-algorithm--1493129749201.html?s_tid=vid_pers_recs
 * https://www.mit.edu/course/16/16.070/www/project/PF_kalman_intro.pdf#:~:text=The%20Kalman%20filter%20is%20a%20set%20of%20mathematical,precise%20nature%20of%20the%20modeled%20system%20is%20un-known.
 * https://thekalmanfilter.com/kalman-filter-explained-simply/
 *
 * ----------------MAY REFERED ON YOUR INTREST-------------
 * https://www.am.chalmers.se/~thab/IMAC/2010/PDFs/Papers/s17p004.pdf
 *
 */

#ifndef INC_KALMAN_FILTER_H_
#define INC_KALMAN_FILTER_H_

#define A 1.8            // Discrete system matrix
#define H 1              // Measurement matrix of state space model
#define Q 0.9         // Process noise
#define R 3  // measurement noise

struct parameters {
	float predict;
	float estimate;
};

struct predict{
	float state;
	float err_covariance;
};

struct update{
	float kalman_gain;
	float estimate;
	float err_covariance;
};

#ifdef KF
static struct parameters KF_update(uint16_t , struct predict );
static struct predict KF_prediction(struct parameters );
#endif
float Kalman_filter(uint16_t , void *);


#endif /* INC_KALMAN_FILTER_H_ */
