#ifndef PID_FILTER_H
#define PID_FILTER_H

#include "cmath"

struct PIDFilter {
    float K[4] = {0.0}; // P, I, D, F
    float sumError;
    float prevError;

    float setpoint;
    float measurement;
    float feedForward;

    float wrap_error(float error) {
        bool wrapped = false;
        while (error >= 3.141592) {
            error -= 2 * 3.141592;
            wrapped = true;
        }
        while (error < -3.141592) {
            error += 2 * 3.141592;
            wrapped = true;
        }
        if (wrapped) error *= -1;
        return error;
    }

    float filter(float dt, bool wrap = false) {
        float error = wrap ? wrap_error(setpoint - measurement) : setpoint - measurement;
        sumError += error * dt;
        float output = (K[0] * error) + (K[2] * ((error - prevError) / dt));
            // + (K[1] * sumError)
            // + (K[2] * ((error - prevError) / dt));
            // + (K[3] * feedForward);
        prevError = error;
        if (fabs(output) > 1.0) output /= fabs(output);
        return output;
    }
};

#endif // PID_FILTER_H