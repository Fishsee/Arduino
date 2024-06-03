#include "SensorManager.h"
#include <Wire.h>  // Add this line
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Ultrasonic.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>

#define TEMPERATURE_PIN 2
#define FLOW_PIN 3
#define PH_SENSOR_PIN A1
#define TURBIDITY_PIN A2
#define ULTRASONIC_PIN 8
#define LED_PIN 4
#define NUM_LEDS 60
#define ATTINY1_HIGH_ADDR 0x78
#define ATTINY2_LOW_ADDR 0x77
#define NO_TOUCH 0xFE

OneWire oneWire(TEMPERATURE_PIN);
DallasTemperature sensors(&oneWire);
Ultrasonic ultrasonic(ULTRASONIC_PIN);
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_RGB + NEO_KHZ800);

int turbidity_status = 0;
unsigned char low_data[8] = {0};
unsigned char high_data[12] = {0};
#define THRESHOLD 100
unsigned long int avgValue;
float b;
int buf[10], temp;
int flow_frequency;
int water_level = 0;
int distance_cm = 0;
int light_level = 0;
int turbidity = 0;
float tempC = 0.0;
float phValueCurrent = 0.0;

void setupSensors() {
    sensors.begin();
    strip.begin();
    pinMode(TURBIDITY_PIN, INPUT);
    pinMode(FLOW_PIN, INPUT);
    digitalWrite(FLOW_PIN, HIGH); 
    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flow, RISING);
}

void updateSensors() {
    distance_cm = ultrasonic.distanceRead(CM);
    light_level = analogRead(A0);
    water_level = getWaterLevel();
    sensors.requestTemperatures();
    tempC = sensors.getTempCByIndex(0);
    checkTurbidity();
    readPH();
    getLightLevel();
    Serial.println("Distance:");
    Serial.println(distance_cm);

    Serial.println("Light:");
    Serial.println(light_level);

    Serial.println("Water:");
    Serial.println(water_level);

    Serial.println("Temperatuur:");
    Serial.println(tempC);

    Serial.println("Troebelheid:");
    Serial.println(turbidity);

    Serial.println("PH:");
    Serial.println(phValueCurrent);
}

float getTemp() {
    sensors.requestTemperatures();
    return sensors.getTempCByIndex(0);
}

void checkTurbidity() {
    const int minAnalogValue = 0;
    const int maxAnalogValue = 640;
    int sensorValue = analogRead(TURBIDITY_PIN);
    turbidity = map(sensorValue, minAnalogValue, maxAnalogValue, 100, 0);
    turbidity = constrain(turbidity, 0, 100);
    turbidity_status = turbidity;
}

void flow() {
    flow_frequency++;
}

void getLightLevel() {
    if (light_level <= 200) {
        colorWipe(strip.Color(255, 255, 255)); 
    }
    else {
        colorWipe(strip.Color(0, 0, 0)); 
    }
    
}

void readPH() {
    for (int i = 0; i < 10; i++) {
        buf[i] = analogRead(PH_SENSOR_PIN);
        delay(10);
    }
    for (int i = 0; i < 9; i++) {
        for (int j = i + 1; j < 10; j++) {
            if (buf[i] > buf[j]) {
                temp = buf[i];
                buf[i] = buf[j];
                buf[j] = temp;
            }
        }
    }
    avgValue = 0;
    for (int i = 2; i < 8; i++)
        avgValue += buf[i];
    float phValue = (float)avgValue * 5.0 / 1024 / 6;
    phValue = 3.5 * phValue + 0.54;
    phValueCurrent = phValue;
}

void getHigh12SectionValue() {
    Wire.beginTransmission(ATTINY1_HIGH_ADDR);
    Wire.write(NO_TOUCH);
    Wire.endTransmission();
    delay(50);
    Wire.requestFrom(ATTINY1_HIGH_ADDR, 12);
    if (Wire.available() == 12) {
        for (int i = 0; i < 12; i++) {
            high_data[i] = Wire.read();
        }
    }
}

void getLow8SectionValue() {
    Wire.beginTransmission(ATTINY2_LOW_ADDR);
    Wire.write(NO_TOUCH);
    Wire.endTransmission();
    delay(50);
    Wire.requestFrom(ATTINY2_LOW_ADDR, 8);
    if (Wire.available() == 8) {
        for (int i = 0; i < 8; i++) {
            low_data[i] = Wire.read();
        }
    }
}

int getWaterLevel() {
    getLow8SectionValue();
    getHigh12SectionValue();

    uint32_t touch_val = 0;
    uint8_t trig_section = 0;
    for (int i = 0; i < 8; i++) {
        if (low_data[i] > THRESHOLD) {
            touch_val |= 1 << i;
        }
    }
    for (int i = 0; i < 12; i++) {
        if (high_data[i] > THRESHOLD) {
            touch_val |= (uint32_t)1 << (8 + i);
        }
    }
    while (touch_val & 0x01) {
        trig_section++;
        touch_val >>= 1;
    }

    return trig_section * 5;
}

String gatherSensorDataAsJson() {
    DynamicJsonDocument doc(512);
    doc["tempC"] = tempC;
    doc["distance_cm"] = distance_cm;
    doc["light_level"] = light_level;
    doc["water_level"] = water_level;
    doc["flow_rate"] = flow_frequency;
    doc["phValue"] = phValueCurrent;
    doc["turbidity"] = turbidity;
    String jsonData;
    serializeJson(doc, jsonData);
    Serial.println(jsonData);
    return jsonData;
}

void setupLEDs() {
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
}

void colorWipe(uint32_t color) {
    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, color);
    }
    strip.show();
}

