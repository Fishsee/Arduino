#include "SensorManager.h"
#include <Wire.h>
#include <Arduino.h>

#define SERVO_PIN 7

void setupServoMotor() {
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);
}

void servo(int pulse) {
    for (int i = 0; i < 8; i++) {
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(SERVO_PIN, LOW);
    }
}
