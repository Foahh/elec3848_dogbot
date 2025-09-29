// constexpr float COUNT_PER_ROTATION = 1320.0;
// constexpr float PULSE_PER_RADIAN = COUNT_PER_ROTATION / (2.0 * PI);

// Pulse Per Second in MAX_PWM for each motor (used for hand calculations)
// constexpr float MOTOR_RADIUS = 0.04;
// constexpr float DISTANCE_PER_PULSE = 2.0 * PI * MOTOR_RADIUS / COUNT_PER_ROTATION;
// constexpr float MAX_PULSE_A = 7922.490543;
// constexpr float MAX_PULSE_B = 8264.861650;
// constexpr float MAX_PULSE_C = 7898.824677;
// constexpr float MAX_PULSE_D = 6715.471914;
// constexpr float MAX_VELOCITY_A = MAX_PULSE_A * DISTANCE_PER_PULSE;
// constexpr float MAX_VELOCITY_B = MAX_PULSE_B * DISTANCE_PER_PULSE;
// constexpr float MAX_VELOCITY_C = MAX_PULSE_C * DISTANCE_PER_PULSE;
// constexpr float MAX_VELOCITY_D = MAX_PULSE_D * DISTANCE_PER_PULSE;

//// Drive Motor in MAX_PWM for 60 second, and serial print encoder values
//// https://www.dynapar.com/knowledge/how-to-calculate-encoder-resolution
//void MotorController::measure_pulses_per_ms()
//{
//    unsigned long curr_time = millis();
//    unsigned long end_time = curr_time + 60000;
//    unsigned long prev_time = 0;
//
//    setPWM_(MAX_PWM, MAX_PWM, MAX_PWM, MAX_PWM);
//
//    while (curr_time < end_time)
//    {
//        curr_time = millis();
//        interval_ = curr_time - prev_time;
//        if (interval_ >= 1000)
//        {
//            measure_();
//            Serial.print(interval_);
//            Serial.print(",");
//            Serial.print(encoder_value_delta_lf_);
//            Serial.print(",");
//            Serial.print(encoder_value_delta_rf_);
//            Serial.print(",");
//            Serial.print(encoder_value_delta_lb_);
//            Serial.print(",");
//            Serial.print(encoder_value_delta_rb_);
//            Serial.println();
//            prev_time = millis();
//        }
//    }
//    setPWM_(0, 0, 0, 0);
//}
//
//// estimate encoder per revolution pulses by hand
//[[noreturn]] void MotorController::measure_encoder()
//{
//    while (true)
//    {
//        measure_();
//        if (encoder_value_delta_lf_ != 0 || encoder_value_delta_rf_ != 0 || encoder_value_delta_lb_ != 0 || encoder_value_delta_rb_ != 0)
//        {
//            Serial.print(encoder_value_lf_);
//            Serial.print(",");
//            Serial.print(encoder_value_rf_);
//            Serial.print(",");
//            Serial.print(encoder_value_lb_);
//            Serial.print(",");
//            Serial.print(encoder_value_rb_);
//            Serial.println();
//        }
//    }
//}