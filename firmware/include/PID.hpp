#include <Arduino.h>

#ifndef PID_HPP
#define PID_HPP

class PID {
public:
    PID(float max, float min, float kp, float ki, float kd, float *set_point, float *input);

    float compute();

private:
    float kp_;
    float ki_;
    float kd_;

    float max_;
    float min_;

    float *set_point_;
    float *input_;

    float error_;
    float integral_;
    float derivative_;
    float prev_error_;
};

PID::PID(float max, float min, float kp, float ki, float kd, float *set_point, float *input) {
    max_ = max;
    min_ = min;
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;

    error_ = 0;
    integral_ = 0;
    derivative_ = 0;
    prev_error_ = 0;

    set_point_ = set_point;
    input_ = input;
}

float PID::compute() {
    error_ = *set_point_ - *input_;
    integral_ += error_;
    derivative_ = error_ - prev_error_;
    prev_error_ = error_;

    if (integral_ > max_) {
        integral_ = max_;
    } else if (integral_ < min_) {
        integral_ = min_;
    }

    float output = (kp_ * error_) + (ki_ * integral_) + (kd_ * derivative_);
    if (output > max_) {
        output = max_;
    } else if (output < min_) {
        output = min_;
    }

    return output;
}

#endif // PID_HPP