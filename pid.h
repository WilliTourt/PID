#pragma once

#include <cmath>

/**
 * Formulations and equations of PID controller:
 * 
 * Standard: u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt
 *                = P + I + D
 * 
 * Where:
 * u(t) is the output
 * e(t) = target - measurement
 * de(t)/dt = (e(t) - e(t-1))/dt
 *          = d(target)/dt - d(measurement)/dt
 * ∫e(t)dt = (e(t-1) + e(t-2) + ... + e(0)) * dt
 * dt is the sample time
 * Kp, Ki, Kd are gains
 * 
 * 
 * 
 * Other formulations:
 * 
 * - Proportional on measurement:
 *      P = - Kp * measurement
 * 
 * - Weighted proportional:
 *      P = Kp*(b * target - measurement)
 *      where b is a weighting factor between 0 and 1
 * 
 * 
 * - Integral clamped:
 *      I = Ki * min(∫e(t)dt, upperLimit)
 *
 * - Conditional integral:
 *      I = Ki*∫e(t)dt, if e(t) < error threshold, 0 otherwise
 * 
 * 
 * - Derivative on measurement:
 *      D = - Kd * d(measurement)/dt
 * 
 * - Weighted derivative:
 *      D = Kd*(c * d(target)/dt - d(measurement)/dt)
 *      where c is a weighting factor between 0 and 1
 * 
 */

#define PID_DEFAULT_SAMPLE_TIME 1.0f

#define PID_DEFAULT_PROP_WEIGHT 1.0f
#define PID_DEFAULT_CLAMP_INT_LIMIT 200.0f
#define PID_DEFAULT_COND_INT_ERR_THRESHOLD 20.0f
#define PID_DEFAULT_DERIVATIVE_WEIGHT 1.0f
#define PID_DEFAULT_DEADZONE 0.0f

class PIDCtrller {
    public:

        enum class ProportionalMode_t {
            Standard,
            OnMeasurement,
            Weighted
        };

        enum class IntegralMode_t {
            Standard,
            Clamped,
            Conditional
        };

        enum class DerivativeMode_t {
            Standard,
            OnMeasurement,
            Weighted
        };

        PIDCtrller(float kp, float ki, float kd);

        void setParams(float kp, float ki, float kd);
        void setTarget(float target);
        void setSampleTime(float sample_time);

        void setProportionalMode(ProportionalMode_t mode);
        void setProportionalMode(ProportionalMode_t mode, float weight);
        void setIntegralMode(IntegralMode_t mode);
        void setIntegralMode(IntegralMode_t mode, float value);
        void setDerivativeMode(DerivativeMode_t mode);
        void setDerivativeMode(DerivativeMode_t mode, float weight);
        void setDeadzone(float deadzone);

        float calc(float curr_measurement, float upperLimit, float lowerLimit);

        void reset();

        inline float getLastOutput() const { return _output; }
        inline float getError() const { return _error; }
        inline float getIntegral() const { return _error_integral; }

    private:

        float _kp, _ki, _kd;
        float _sample_time;

        float _output;
        float _target;
        float _error, _error_integral, _prev_error;
        float _prev_measurement, _prev_target;

        float _proportional_weight;
        float _clamped_integral_limit;
        float _cond_integral_error_threshold;
        float _derivative_weight;

        ProportionalMode_t _proportional_mode;
        IntegralMode_t _integral_mode;
        DerivativeMode_t _derivative_mode;

        float _deadzone;

};
