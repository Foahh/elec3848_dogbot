#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP
#define ENCODER_OPTIMIZE_INTERRUPTS

#include <Arduino.h>
#include <Encoder.h>
#include <PID.hpp>

#define DIRA1 34 // Motor A Direction +
#define DIRA2 35 // Motor A Direction -
#define ECDAA 18 // Motor A Encoder PIN A
#define ECDAB 31 // Motor A Encoder PIN B
#define PWMA 12  // Motor A PWM

#define DIRB1 36 // Motor B Direction +
#define DIRB2 37 // Motor B Direction -
#define ECDBA 19 // Motor B Encoder PIN A
#define ECDBB 38 // Motor B Encoder PIN B
#define PWMB 8   // Motor B PWM

#define DIRC1 43 // Motor C Direction +
#define DIRC2 42 // Motor C Direction -
#define ECDCA 3  // Motor C Encoder PIN A
#define ECDCB 49 // Motor C Encoder PIN B
#define PWMC 6   // Motor C PWM


#define DIRD1 A5 // Motor D Direction +
#define DIRD2 A4 // Motor D Direction -
#define ECDDA 2  // Motor D Encoder PIN A
#define ECDDB A1 // Motor D Encoder PIN B
#define PWMD 5   // Motor D PWM

#define MAX_PWM 255

#define kP 0.750000
#define kI 0.001000
#define kD 0.000500

constexpr float COUNT_PER_ROTATION = 1320.0;
constexpr float PULSE_PER_RADIAN = COUNT_PER_ROTATION / (2.0 * PI);

class MotorController {
public:
    MotorController();

    static void begin();

    void control();

    void setTargetSpeed(float speed_lf, float speed_rf, float speed_lb, float speed_rb); // pulse/ms

    void forward();
    void backward();
    void left();
    void right();
    void stop();

    void sendFeedback();

private:
    inline static void setPWM_(int pwm_lf, int pwm_rf, int pwm_lb, int pwm_rb);

    inline void measure_();

    Encoder encoder_lf_;
    Encoder encoder_rf_;
    Encoder encoder_lb_;
    Encoder encoder_rb_;

    long encoder_value_lf_;
    long encoder_value_rf_;
    long encoder_value_lb_;
    long encoder_value_rb_;

    long encoder_value_prev_lf_;
    long encoder_value_prev_rf_;
    long encoder_value_prev_lb_;
    long encoder_value_prev_rb_;

    long encoder_value_delta_lf_;
    long encoder_value_delta_rf_;
    long encoder_value_delta_lb_;
    long encoder_value_delta_rb_;

    float target_speed_lf_;
    float target_speed_rf_;
    float target_speed_lb_;
    float target_speed_rb_;

    float measured_speed_lf_;
    float measured_speed_rf_;
    float measured_speed_lb_;
    float measured_speed_rb_;

    unsigned long curr_time_;
    unsigned long prev_time_;
    unsigned long delta_time_;
    unsigned long interval_;

    PID pid_lf_;
    PID pid_rf_;
    PID pid_lb_;
    PID pid_rb_;
};

MotorController::MotorController() : encoder_lf_(ECDAA, ECDAB), encoder_rf_(ECDBA, ECDBB),
                                     encoder_lb_(ECDCA, ECDCB), encoder_rb_(ECDDA, ECDDB),
                                     pid_lf_(MAX_PWM, -MAX_PWM, kI, kP, kD, &target_speed_lf_, &measured_speed_lf_),
                                     pid_rf_(MAX_PWM, -MAX_PWM, kI, kP, kD, &target_speed_rf_, &measured_speed_rf_),
                                     pid_lb_(MAX_PWM, -MAX_PWM, kI, kP, kD, &target_speed_lb_, &measured_speed_lb_),
                                     pid_rb_(MAX_PWM, -MAX_PWM, kI, kP, kD, &target_speed_rb_, &measured_speed_rb_) {
    encoder_value_lf_ = 0;
    encoder_value_rf_ = 0;
    encoder_value_lb_ = 0;
    encoder_value_rb_ = 0;

    encoder_value_prev_lf_ = 0;
    encoder_value_prev_rf_ = 0;
    encoder_value_prev_lb_ = 0;
    encoder_value_prev_rb_ = 0;

    encoder_value_delta_lf_ = 0;
    encoder_value_delta_rf_ = 0;
    encoder_value_delta_lb_ = 0;
    encoder_value_delta_rb_ = 0;

    target_speed_lf_ = 0;
    target_speed_rf_ = 0;
    target_speed_lb_ = 0;
    target_speed_rb_ = 0;

    measured_speed_lf_ = 0;
    measured_speed_rf_ = 0;
    measured_speed_lb_ = 0;
    measured_speed_rb_ = 0;

    curr_time_ = 0;
    prev_time_ = 0;
    delta_time_ = 0;
    interval_ = 0;
}

void MotorController::begin() {
    pinMode(DIRA1, OUTPUT);
    pinMode(DIRA2, OUTPUT);
    pinMode(DIRB1, OUTPUT);
    pinMode(DIRB2, OUTPUT);
    pinMode(DIRC1, OUTPUT);
    pinMode(DIRC2, OUTPUT);
    pinMode(DIRD1, OUTPUT);
    pinMode(DIRD2, OUTPUT);

    pinMode(ECDAA, INPUT);
    pinMode(ECDAB, INPUT);
    pinMode(ECDBA, INPUT);
    pinMode(ECDBB, INPUT);
    pinMode(ECDCA, INPUT);
    pinMode(ECDCB, INPUT);
    pinMode(ECDDA, INPUT);
    pinMode(ECDDB, INPUT);

    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(PWMC, OUTPUT);
    pinMode(PWMD, OUTPUT);

    pinMode(DIRA1, OUTPUT);
    pinMode(DIRA2, OUTPUT);
    pinMode(DIRB1, OUTPUT);
    pinMode(DIRB2, OUTPUT);
    pinMode(DIRC1, OUTPUT);
    pinMode(DIRC2, OUTPUT);
    pinMode(DIRD1, OUTPUT);
    pinMode(DIRD2, OUTPUT);
}

inline void MotorController::setPWM_(int pwm_lf, int pwm_rf, int pwm_lb, int pwm_rb) {
    if (pwm_lf == 0) {
        digitalWrite(DIRA1, LOW);
        digitalWrite(DIRA2, LOW);
    } else{
        digitalWrite(DIRA1, pwm_lf < 0 ? HIGH : LOW);
        digitalWrite(DIRA2, pwm_lf > 0 ? HIGH : LOW);
    
    }

    if (pwm_rf == 0) {
        digitalWrite(DIRB1, LOW);
        digitalWrite(DIRB2, LOW);
    } else{
        digitalWrite(DIRB1, pwm_rf < 0 ? HIGH : LOW);
        digitalWrite(DIRB2, pwm_rf > 0 ? HIGH : LOW);
    }

    if (pwm_lb == 0) {
        digitalWrite(DIRC1, LOW);
        digitalWrite(DIRC2, LOW);
    } else{
        digitalWrite(DIRC1, pwm_lb < 0 ? HIGH : LOW);
        digitalWrite(DIRC2, pwm_lb > 0 ? HIGH : LOW);
    }

    if (pwm_rb == 0) {
        digitalWrite(DIRD1, LOW);
        digitalWrite(DIRD2, LOW);
    } else{
        digitalWrite(DIRD1, pwm_rb < 0 ? HIGH : LOW);
        digitalWrite(DIRD2, pwm_rb > 0 ? HIGH : LOW);
    }

    analogWrite(PWMA, abs(pwm_lf));
    analogWrite(PWMB, abs(pwm_rf));
    analogWrite(PWMC, abs(pwm_lb));
    analogWrite(PWMD, abs(pwm_rb));
}

void MotorController::forward(){
    setTargetSpeed(1.0, 1.0, 1.0, 1.0);
}

void MotorController::backward(){
    setTargetSpeed(-1.0, -1.0, -1.0, -1.0);
}

void MotorController::left(){
    setTargetSpeed(-1.0, 1.0, 1.0, -1.0);
}

void MotorController::right(){
    setTargetSpeed(1.0, -1.0, -1.0, 1.0);
}

void MotorController::stop(){
    setTargetSpeed(0.0, 0.0, 0.0, 0.0);
}


void MotorController::setTargetSpeed(float speed_lf, float speed_rf, float speed_lb, float speed_rb) {
    target_speed_lf_ = speed_lf;
    target_speed_rf_ = speed_rf;
    target_speed_lb_ = speed_lb;
    target_speed_rb_ = speed_rb;
}

void MotorController::control() {
    curr_time_ = millis();
    delta_time_ = curr_time_ - prev_time_;
    if (delta_time_ <= 10) {
        return;
    }
    prev_time_ = curr_time_;

    measure_();

    measured_speed_lf_ = (float) encoder_value_delta_lf_ / (float) delta_time_;
    measured_speed_rf_ = (float) encoder_value_delta_rf_ / (float) delta_time_;
    measured_speed_lb_ = (float) encoder_value_delta_lb_ / (float) delta_time_;
    measured_speed_rb_ = (float) encoder_value_delta_rb_ / (float) delta_time_;


    setPWM_((int) pid_lf_.compute(), (int) pid_rf_.compute(),
            (int) pid_lb_.compute(), (int) pid_rb_.compute());
}

inline void MotorController::measure_() {
    encoder_value_prev_lf_ = encoder_value_lf_;
    encoder_value_prev_rf_ = encoder_value_rf_;
    encoder_value_prev_lb_ = encoder_value_lb_;
    encoder_value_prev_rb_ = encoder_value_rb_;

    encoder_value_lf_ = -encoder_lf_.read();
    encoder_value_rf_ = encoder_rf_.read();
    encoder_value_lb_ = -encoder_lb_.read();
    encoder_value_rb_ = encoder_rb_.read();

    encoder_value_delta_lf_ = encoder_value_lf_ - encoder_value_prev_lf_;
    encoder_value_delta_rf_ = encoder_value_rf_ - encoder_value_prev_rf_;
    encoder_value_delta_lb_ = encoder_value_lb_ - encoder_value_prev_lb_;
    encoder_value_delta_rb_ = encoder_value_rb_ - encoder_value_prev_rb_;
}

void MotorController::sendFeedback() {
    measure_();
    Serial.print(encoder_value_lf_);
    Serial.print(",");
    Serial.print(encoder_value_rf_);
    Serial.print(",");
    Serial.print(encoder_value_lb_);
    Serial.print(",");
    Serial.println(encoder_value_rb_);
}

#endif
