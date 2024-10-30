/*
 * kalman_filter.c
 *
 *  Created on: Oct 26, 2024
 *      Author: ilanc
 */
#define KF

#include "main.h"
#include "Kalman_filter.h"

static struct parameters KF_update(uint16_t measure, struct predict KF_predict)
{
	struct update KF_update;
	struct parameters KF_para;

	//compute Kalman gain
	KF_update.kalman_gain = (KF_predict.err_covariance * H) / (H * KF_predict.err_covariance + R);

	//update estimate with measurement
	KF_update.estimate = KF_predict.state + KF_update.kalman_gain * (measure - H * KF_predict.state);

	//update error covariance
	KF_update.err_covariance = KF_predict.err_covariance * (1 - KF_update.kalman_gain * H);

	KF_para.estimate = KF_update.estimate;
	KF_para.predict = KF_update.err_covariance;

	return KF_para;
}

static struct predict KF_prediction(struct parameters KF_para)
{
	struct predict KF_predict;

	//Project the state ahead
	KF_predict.state = A * KF_para.estimate;

	//project error covariance ahead
	KF_predict.err_covariance = A * KF_para.predict + Q;

	return KF_predict;

}

float Kalman_filter(uint16_t measurement, void *KF_para)
{
	struct parameters *KF_param;
	struct predict KF_predict;

	KF_param = (struct parameters *)KF_para;
	KF_predict = KF_prediction(*KF_param);
	*KF_param = KF_update(measurement, KF_predict);

	return KF_param->estimate;
}

