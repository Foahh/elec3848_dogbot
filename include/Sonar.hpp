#ifndef SONAR_HPP
#define SONAR_HPP

#include <Arduino.h>

#define TRIG_PIN A2
#define ECHO_PIN A3

class Sonar
{
public:
    Sonar();
    static void setup();
    inline void update();
    inline void send();

private:
    unsigned long start_time;
    unsigned long duration;
    bool done = true;
};

Sonar::Sonar() {}

void Sonar::setup()
{
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

inline void Sonar::update()
{
    if (done)
    {
        done = false;
        start_time = millis();
        digitalWrite(TRIG_PIN, LOW);
    }

    if (millis() > start_time + 2)
    {
        digitalWrite(TRIG_PIN, HIGH);
    }

    if (millis() > start_time + 10)
    {
        digitalWrite(TRIG_PIN, LOW);
        duration = pulseIn(ECHO_PIN, HIGH);
        done = true;
    }
}

inline void Sonar::send()
{
    Serial.println(duration); // distance_in_cm = duration / 58.2
}

#endif