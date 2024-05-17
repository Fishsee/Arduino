/////////////////////////////////////////////////////////////////
//             Arduino turbidity meter Project                    //
//                                                             //
//             www.edisonsciencecorner.blogspot.com            //
/////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include "rgb_lcd.h"

int sensorPin = A2;

rgb_lcd lcd;

void setup()
{
  lcd.begin(16,2);
  pinMode(3,OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

}

void loop() {
  int sensorValue = analogRead(sensorPin);
  int turbidity = map(sensorValue, 0,640, 100, 0);
  delay(100);
  lcd.setCursor(0, 0);
  lcd.print("turbidity:");
  lcd.print("   ");
  lcd.setCursor(10, 0);
  lcd.print(turbidity);
  delay(100);
  if (turbidity < 20) {
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
    lcd.setCursor(0, 1);
    lcd.print(" its CLEAR ");
  }
  if ((turbidity > 10) && (turbidity < 50)) {
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
    digitalWrite(9, LOW);
    lcd.setCursor(0, 1);
    lcd.print(" its CLOUDY ");
  }
  if (turbidity > 50) {
    digitalWrite(7, LOW);
    digitalWrite(8, LOW);
    digitalWrite(9, HIGH);
    lcd.setCursor(0, 1);
    lcd.print(" its DIRTY ");
  }

}
