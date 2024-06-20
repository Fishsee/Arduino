#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "SensorManager.h"

void setup() {
    Serial.begin(115200);
    while (!Serial);

    setupLEDs();
    setupSensors();
}

void loop() {
    updateSensors();
}
