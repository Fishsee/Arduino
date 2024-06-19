#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "BluetoothManager.h"
#include "EEPROMManager.h"
#include "SensorManager.h"
#include "ServoMotorManager.h"
#include "WiFiManager.h"
#include "Common.h"

void setup() {
    Serial.begin(115200);
    while (!Serial);

    setupLEDs();
    setupBluetooth();
    setupSensors();
    setupEEPROM();
    setupWiFi();
    setupServoMotor();
}

void loop() {
    handleBluetooth();
    updateSensors();
    handleWiFi();
    handleBrightnessControl();
}
