//
// Created by ILKER KESER on 25.11.2024.
//

#ifndef KALMAN_KALMANFILTER_H
#define KALMAN_KALMANFILTER_H

typedef struct {
    float _err_measure;
    float _err_estimate;
    float _q;
    float _current_estimate;
    float _last_estimate;
    float _kalman_gain;
}KalmanFilter_t;

void  KalmanFilter_Init(KalmanFilter_t *kalman,float mea_e,float est_e, float q);
float updateEstimate(KalmanFilter_t *kalman,float mea);
void setMeasurementError(KalmanFilter_t *kalman,float mea_e);
void setEstimateError(KalmanFilter_t *kalman,float est_e);
void setProcessNoise(KalmanFilter_t *kalman,float q);
float getKalmanGain(KalmanFilter_t *kalman);
float getEstimateError(KalmanFilter_t *kalman);

#endif //KALMAN_KALMANFILTER_H
