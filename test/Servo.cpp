#include <Arduino.h>
const int pin = 1;

void servo(int pulse) {
      for (int i = 0; i < 8; i++) {
        digitalWrite(pin, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(pin, LOW);
      }
}