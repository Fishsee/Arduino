#include <Arduino.h>
#include <WiFiS3.h>
#include <ArduinoBLE.h>
#include <HttpClient.h>
#include <WiFiClient.h>
const int servoPin = 7;
#define servoOpen 1600 // value for servo being open
#define servoClosed 950 // value for servo being closed
#define servoDelay 20 


void setup() {

   pinMode(servoPin, OUTPUT);
   digitalWrite(servoPin, LOW);

}

void servo(int pulse);


void loop(){
    servo(servoOpen);
    delay(5000);
    servo(servoClosed);
    delay(5000);
}

void servo(int pulse) {
      for (int i = 0; i < 8; i++) {
        digitalWrite(servoPin, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(servoPin, LOW);
      }
}