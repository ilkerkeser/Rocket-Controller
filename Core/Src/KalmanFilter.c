//
// Created by ILKER KESER on 25.11.2024.
//

#include "KalmanFilter.h"
#include <math.h>

void  KalmanFilter_Init(KalmanFilter_t *kalman,float mea_e,float est_e, float q){
    kalman->_err_measure = mea_e;
    kalman->_err_estimate = est_e;
    kalman->_q = q;

    kalman->_current_estimate =0.0f;
    kalman->_last_estimate =0.0f;
    kalman->_kalman_gain =0.0f;
}

float updateEstimate(KalmanFilter_t *kalman,float mea){
    kalman->_kalman_gain = kalman->_err_estimate /(kalman->_err_estimate + kalman->_err_measure);
    kalman->_current_estimate = kalman->_last_estimate + kalman->_kalman_gain * (mea - kalman->_last_estimate);
    kalman->_err_estimate = (1.0f - kalman->_kalman_gain) * kalman->_err_estimate +
                            fabsf(kalman->_last_estimate - kalman->_current_estimate)*kalman->_q;
    kalman->_last_estimate = kalman->_current_estimate;

    return kalman->_current_estimate;
}


void setMeasurementError(KalmanFilter_t *kalman,float mea_e){
    kalman->_err_measure = mea_e;
}

void setEstimateError(KalmanFilter_t *kalman,float est_e){
    kalman->_err_estimate = est_e;
}

void setProcessNoise(KalmanFilter_t *kalman,float q){
    kalman->_q = q;
}

float getKalmanGain(KalmanFilter_t *kalman){
    return kalman->_kalman_gain;
}


float getEstimateError(KalmanFilter_t *kalman){
    return kalman->_err_estimate;
}
