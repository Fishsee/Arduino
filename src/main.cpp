#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h> // Voor I2C-communicatie
#include "rgb_lcd.h" // Inclusief de bibliotheek voor de Grove RGB LCD
#include "Ultrasonic.h" // Inclusief de bibliotheek voor de ultrasone sensor

#define LED_PIN     4
#define NUM_LEDS    60

// Pinnen voor de Grove RGB LCD
#define LCD_ADDRESS 0x62 // I2C-adres van de Grove LCD RGB Backlight

// Pin voor de Grove Ultrasonic-sensor
#define ULTRASONIC_PIN  3 // Verbind deze met de enkele pin van de Grove Ultrasonic-sensor

// Pin voor de Grove Turbidity Sensor Meter
#define TURBIDITY_PIN A1 // Verbind deze met de pin van de Grove Turbidity Sensor Meter

// Pin voor de Grove Light Sensor v1.2
#define LIGHT_SENSOR_PIN A0 // Verbind deze met de analoge pin van de Grove Light Sensor

// Pin voor de Grove Water Level Sensor (10cm)
#define WATER_LEVEL_PIN A3 // Verbind deze met de analoge pin van de Grove Water Level Sensor

rgb_lcd lcd;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
Ultrasonic ultrasonic(ULTRASONIC_PIN);

void colorWipe(uint32_t color) {
  for(int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void setup() {
  strip.begin();
  colorWipe(strip.Color(0, 0, 0));  // Zet de LED-strip uit bij het opstarten

  // Initialiseer de I2C-bus
  Wire.begin();

  // Initialiseer de Grove RGB LCD
  lcd.begin(16, 2);
  lcd.setRGB(0, 255, 0); // Stel de achtergrondkleur van het LCD-scherm in op rood
  lcd.print("Hello, world!");
  delay(1000);
  lcd.clear();
}

void loop() {
  // Meet de afstand met de ultrasone sensor
  int distance_cm = (int)ultrasonic.distanceRead(CM); // Converteer naar int om decimalen te verwijderen

  // Meet de troebelheid met de Turbidity Sensor Meter
  int turbidity_value = analogRead(TURBIDITY_PIN);

  // Meet het lichtniveau met de Light Sensor v1.2
  int light_level = analogRead(LIGHT_SENSOR_PIN);

  // Meet de waterstand met de Water Level Sensor (10cm)
  int water_level = analogRead(WATER_LEVEL_PIN);

  // Bepaal het percentage waterstand (aangenomen dat 0 het minimum en 1023 het maximum is)
  float percentage_water = (float)(1023 - water_level) / 1023 * 100;

  // Weergeef de metingen op het LCD-scherm
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("W:");
  lcd.print(percentage_water, 1); // Toon percentage met 1 decimaal precisie
  lcd.print("% ");
  lcd.print("A:");
  lcd.print(distance_cm);
  lcd.print("cm");
  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(turbidity_value);
  lcd.print(" L:");
  lcd.print(light_level);

  // Schakel de LED-strip in of uit op basis van het gemeten lichtniveau
  if (light_level < 400) {
    // Als het lichtniveau laag is, schakel de LED-strip aan
    colorWipe(strip.Color(255, 255, 255));  // Wit
  } else {
    // Als het lichtniveau hoog is, schakel de LED-strip uit
    colorWipe(strip.Color(0, 0, 0));  // Uit
  }

  delay(50);
}

