#include <Arduino.h>
#include "BluetoothManager.h"
#include "DisplayManager.h"
#include "EEPROMManager.h"
#include "SensorManager.h"
#include "ServoMotorManager.h"
#include "WiFiManager.h"
#include "Common.h"

void setup() {
    Serial.begin(115200);
    while (!Serial);

    setupLEDs(); // Changed to setupLEDs() function declaration in SensorManager.h
    setupLCD();
    setupBluetooth();
    setupSensors();
    setupEEPROM();
    setupWiFi();
    setupServoMotor();

    delay(500);
}

void loop() {
    handleBluetooth();
    handleWiFi();
    updateSensors();
    updateDisplay();
}
