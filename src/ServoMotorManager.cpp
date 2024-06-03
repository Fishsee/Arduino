#include "ServoMotorManager.h"
#include <Servo.h>
#include <Arduino.h>

// Define the servo pins
#define SERVO_PIN 7
#define SERVO_PIN_2 5

Servo servo1;
Servo servo2;

void setupServoMotor() {
    servo1.attach(SERVO_PIN);
    servo2.attach(SERVO_PIN_2);
}

void moveServos(int position) {
    servo1.write(position);
    servo2.write(position);
}
