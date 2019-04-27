/*
 * SimpleKalmanFilter - a Kalman Filter implementation for single variable models.
 * Created by Denys Sene, January, 1, 2017.
 * Released under MIT License - see LICENSE file for details.
 */

#ifndef SimpleKalmanFilter_h
#define SimpleKalmanFilter_h

void SimpleKalmanFilterInit();
void SimpleKalmanFilter(float mea_e, float est_e, float q, int idx);
float updateEstimate(float mea, int idx);
void setMeasurementError(float mea_e, int idx);
void setEstimateError(float est_e, int idx);
void setProcessNoise(float q, int idx);
float getKalmanGain(int idx);
float getEstimateError(int idx);

#endif
