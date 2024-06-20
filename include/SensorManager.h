#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Ultrasonic.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <Servo.h>

void setupSensors();
void updateSensors();
float getTemp();
void checkTurbidity();
void flow();
void readPH();
void getHigh12SectionValue();
void getLow8SectionValue();
int getWaterLevel();
String gatherSensorDataAsJson();
void setupLEDs();
void colorWipe(uint32_t color);
void getLightLevel();
void setupServoMotor();
void moveServo1(int position);
void moveServo2(int position);
void setBrightness(uint8_t brightness);
void commandRecieve();

extern int water_level;
extern int distance_cm;
extern int light_level;
extern float tempC;
extern int turbidity;
extern float phValueCurrent;
extern int flow_frequency;

#endif
