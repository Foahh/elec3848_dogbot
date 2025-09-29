#include "Display.hpp"
#include "MotorController.hpp"
#include "Sonar.hpp"

#include <Arduino.h>
#include <Servo.h>

#define FOREARM_SERVO_PIN 47
#define GRIPPER_SERVO_PIN 48

MotorController motor;
Servo forearm;
Servo gripper;
Sonar sonar;

int forearm_angle = 90;
int gripper_angle = 30;

//====================================================================//

const byte buffer_length = 64;

char char_buffer[buffer_length];
char temp_char_buffer[buffer_length];
char *token;
char rc;

float motor_args[4] = {0.0};
int servo_args[2] = {0};

const char start_marker = '<';
const char end_marker = '>';

boolean data_available = false;
boolean recv_in_progress = false;
byte ndx = 0;

// https://forum.arduino.cc/t/serial-input-basics-updated/382007/3
inline void serialReceiver()
{
    if (Serial.available() > 0 && !data_available)
    {
        rc = (char)Serial.read();
        if (rc == start_marker)
        {
            recv_in_progress = true;
            ndx = 0;
            data_available = false;
        }
        else if (recv_in_progress)
        {
            if (rc != end_marker)
            {
                char_buffer[ndx] = rc;
                ndx++;
                if (ndx >= buffer_length)
                {
                    ndx = buffer_length - 1;
                }
            }
            else
            {
                char_buffer[ndx] = '\0';
                recv_in_progress = false;
                ndx = 0;
                data_available = true;
            }
        }
    }
}

inline void serialHandler()
{
    switch (strtok(temp_char_buffer, ",")[0])
    {
    case 'S':
        Serial.println("<OK>");
        break;
    case 'M':
        for (int i = 0; i < 4; i++)
        {
            token = strtok(nullptr, ",");
            motor_args[i] = atof(token);
        }
        motor.setTargetSpeed(motor_args[0], motor_args[1], motor_args[2], motor_args[3]);
        Serial.println("<OK>");
        break;
    case 'E':
        motor.sendFeedback();
        break;
    case 'P':
        for (int i = 0; i < 2; i++)
        {
            token = strtok(nullptr, ",");
            servo_args[i] = atoi(token);
        }
        forearm_angle = servo_args[0];
        gripper_angle = servo_args[1];
        Serial.println("<OK>");
        break;
    case 'U':
        sonar.send();
        break;
    case 'w':
        motor.forward();
        Serial.println("<OK>");
        break;
    case 's':
        motor.left();
        Serial.println("<OK>");
        break;
    case 'a':
        motor.backward();
        Serial.println("<OK>");
        break;
    case 'd':
        motor.right();
        Serial.println("<OK>");
        break;
    case 'x':
        motor.stop();
        Serial.println("<OK>");
        break;
    default:
        Serial.println("<ERROR>");
        break;
    }
}

void setup()
{
    forearm.attach(FOREARM_SERVO_PIN);
    gripper.attach(GRIPPER_SERVO_PIN);
    MotorController::setup();
    Sonar::setup();
    Serial.begin(115200);
}

void loop()
{
    forearm.write(forearm_angle);
    gripper.write(gripper_angle);
    serialReceiver();
    if (data_available)
    {
        strcpy(temp_char_buffer, char_buffer);
        serialHandler();
        data_available = false;
    }
    motor.control();
    sonar.update();
}