/*
 * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
 * Created by Denys Sene, January, 1, 2017.
 * Released under MIT License - see LICENSE file for details.
 */
#include <math.h>

#define NumOfTOF 6
#define Init_Mea_E 1
#define Init_Est_E 1
#define Init_Q (0.01)

typedef struct kalmanFilter {
  float _err_measure;
  float _err_estimate;
  float _q;
  float _current_estimate;
  float _last_estimate;
  float _kalman_gain;
} TOFKalmanFilter;

TOFKalmanFilter TOFKalmanFilters[NumOfTOF];

void SimpleKalmanFilterInit() {
  SimpleKalmanFilter(Init_Mea_E, Init_Est_E, Init_Q, 0);
  SimpleKalmanFilter(Init_Mea_E, Init_Est_E, Init_Q, 1);
  SimpleKalmanFilter(Init_Mea_E, Init_Est_E, Init_Q, 2);
  SimpleKalmanFilter(Init_Mea_E, Init_Est_E, Init_Q, 3);
}
void SimpleKalmanFilter(float mea_e, float est_e, float q, int idx) {
  TOFKalmanFilters[idx]._err_measure = mea_e;
  TOFKalmanFilters[idx]._err_estimate = est_e;
  TOFKalmanFilters[idx]._q = q;
}

float updateEstimate(float mea, int idx) {
  TOFKalmanFilters[idx]._kalman_gain = TOFKalmanFilters[idx]._err_estimate / (TOFKalmanFilters[idx]._err_estimate + TOFKalmanFilters[idx]._err_measure);
  TOFKalmanFilters[idx]._current_estimate = TOFKalmanFilters[idx]._last_estimate + TOFKalmanFilters[idx]._kalman_gain * (mea - TOFKalmanFilters[idx]._last_estimate);
  TOFKalmanFilters[idx]._err_estimate =  ((float)1.0 - TOFKalmanFilters[idx]._kalman_gain)*TOFKalmanFilters[idx]._err_estimate + fabsf(TOFKalmanFilters[idx]._last_estimate-TOFKalmanFilters[idx]._current_estimate)*TOFKalmanFilters[idx]._q;
  TOFKalmanFilters[idx]._last_estimate = TOFKalmanFilters[idx]._current_estimate;
  return TOFKalmanFilters[idx]._current_estimate;
}

void setMeasurementError(float mea_e, int idx) {
  TOFKalmanFilters[idx]._err_measure = mea_e;
}

void setEstimateError(float est_e, int idx) {
  TOFKalmanFilters[idx]._err_estimate = est_e;
}

void setProcessNoise(float q, int idx) {
  TOFKalmanFilters[idx]._q = q;
}

float getKalmanGain(int idx) {
  return TOFKalmanFilters[idx]._kalman_gain;
}

float getEstimateError(int idx) {
  return TOFKalmanFilters[idx]._err_estimate;
}

//SimpleKalmanFilter::SimpleKalmanFilter(float mea_e, float est_e, float q)
//{
//  _err_measure=mea_e;
//  _err_estimate=est_e;
//  _q = q;
//}
//
//float SimpleKalmanFilter::updateEstimate(float mea)
//{
//  _kalman_gain = _err_estimate/(_err_estimate + _err_measure);
//  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
//  _err_estimate =  (1.0 - _kalman_gain)*_err_estimate + fabs(_last_estimate-_current_estimate)*_q;
//  _last_estimate=_current_estimate;
//
//  return _current_estimate;
//}

//void SimpleKalmanFilter::setMeasurementError(float mea_e)
//{
//  _err_measure=mea_e;
//}

//void SimpleKalmanFilter::setEstimateError(float est_e)
//{
//  _err_estimate=est_e;
//}

//void SimpleKalmanFilter::setProcessNoise(float q)
//{
//  _q=q;
//}

//float SimpleKalmanFilter::getKalmanGain() {
//  return _kalman_gain;
//}
//
//float SimpleKalmanFilter::getEstimateError() {
//  return _err_estimate;
//}
