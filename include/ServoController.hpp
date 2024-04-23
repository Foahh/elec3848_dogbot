
#include <Servo.h>

#define FOREARM_SERVO_PIN 48
#define GRIPPER_SERVO_PIN 47

class ServoController
{
public:
    ServoController();

    void setServoPosition(int forearm, int gripper);

private:
    Servo forearm_;
    Servo gripper_;
};

ServoController::ServoController() : forearm_(), gripper_()
{
    forearm_.attach(FOREARM_SERVO_PIN);
    gripper_.attach(GRIPPER_SERVO_PIN);
}

void ServoController::setServoPosition(int forearm, int gripper)
{
    forearm_.write(forearm);
    gripper_.write(gripper);
}
