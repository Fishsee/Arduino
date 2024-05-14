#include <Arduino.h>
#include <NewPing.h> //sonar library

#define TRIG_PIN 7 // recieve pin
#define ECHO_PIN 12 // send pin
#define MAX_DISTANCE 200 //maximum distance of reading
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE)

void readSonar();

//reads the sonar
void readSonar() {
      int distance = sonar.ping_cm();
}