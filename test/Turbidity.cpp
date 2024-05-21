/////////////////////////////////////////////////////////////////
//             Arduino turbidity meter Project                 //
//                                                             //
//             www.edisonsciencecorner.blogspot.com            //
/////////////////////////////////////////////////////////////////
#include <Arduino.h>
#include "rgb_lcd.h"

const int sensorPin = A2;  // Analog pin for the turbidity sensor

rgb_lcd lcd;

void setup() {
  
  lcd.begin(16, 2);  // Initialize the LCD with 16 columns and 2 rows
  lcd.setRGB(255, 255, 255);
  lcd.clear();
}

void loop() {
  int sensorValue = analogRead(sensorPin);  // Read the analog sensor value
  int turbidity = map(sensorValue, 0, 640, 100, 0);  // Map the sensor value to a turbidity percentage
  delay(100);  // Debounce delay

  // Clear the first row and print turbidity value
  lcd.setCursor(0, 0);
  lcd.print("turbidity:    ");  // Clear previous value by adding spaces
  lcd.print(turbidity);

  // Determine and display the water quality
  lcd.setCursor(0, 1);
  if (turbidity < 20) {
    lcd.setCursor(0, 1);
    lcd.print(" its CLEAR   ");
  } else if (turbidity <= 50) {
    lcd.setCursor(0, 1);
    lcd.print(" its CLOUDY  ");
  } else {
    lcd.setCursor(0, 1);
    lcd.print(" its DIRTY   ");
  }
  

  delay(1000);  // Delay to update readings every second
}
