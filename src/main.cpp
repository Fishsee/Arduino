#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include "Ultrasonic.h"
#include <WiFi.h> // Changed from WiFiNINA to WiFi

const char* ssid = "Broodjegehaktbal";
const char* password = "12345678";
int status = WL_IDLE_STATUS;

#define LED_PIN     4
#define NUM_LEDS    60

#define LCD_ADDRESS 0x62

#define ULTRASONIC_PIN  3

#define TURBIDITY_PIN A1

#define LIGHT_SENSOR_PIN A0

#define WATER_LEVEL_PIN A3

rgb_lcd lcd;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
Ultrasonic ultrasonic(ULTRASONIC_PIN);

void colorWipe(uint32_t color);
void printWiFiStatus();

void setup() {
  strip.begin();
  colorWipe(strip.Color(0, 0, 0));
  Serial.begin(9600);
  Wire.begin();

  // Initialiseer de Grove RGB LCD
  lcd.begin(16, 2);
  lcd.setRGB(0, 255, 0);
  lcd.print("Hello, world!");
  delay(500);
  lcd.clear();

  // Check for the WiFi module
  if (WiFi.status() == WL_NO_SHIELD) {
    lcd.print("WiFi module fail");
    while (true);
  }

  // Attempt to connect to WiFi network
  lcd.print("Attempting to connect");
  lcd.setCursor(0, 1);
  lcd.print(ssid);
  lcd.setCursor(0, 0);
  delay(500);

  while (status != WL_CONNECTED) {
    lcd.print("Connecting...");
    status = WiFi.begin(ssid, password);

    // Wait 10 seconds for connection
    delay(5000);

    if (status == WL_CONNECTED) {
      lcd.clear();
      lcd.print("Connected to WiFi");
      delay(500);
      printWiFiStatus();
    } else {
      lcd.clear();
      lcd.print("Failed to connect");
    }
  }
}

void loop() {
  // Measure distance with the ultrasonic sensor
  int distance_cm = (int)ultrasonic.distanceRead(CM);

  // Measure turbidity
  int turbidity_value = analogRead(TURBIDITY_PIN);

  // Measure light level
  int light_level = analogRead(LIGHT_SENSOR_PIN);

  // Measure water level
  int water_level = analogRead(WATER_LEVEL_PIN);

  // Calculate water level percentage
  float percentage_water = (float)(1023 - water_level) / 1023 * 100;

  // Display measurements on the LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("W:");
  lcd.print(water_level, 1);
  lcd.print("% A:");
  lcd.print(distance_cm);
  lcd.print("cm");
  lcd.setCursor(0, 1);
  lcd.print("T:");
  lcd.print(turbidity_value);
  lcd.print(" L:");
  lcd.print(light_level);

  // Control the LED strip based on light level
  if (light_level < 400) {
    colorWipe(strip.Color(255, 255, 255));
  } else {
    colorWipe(strip.Color(0, 0, 0));
  }

  delay(5000);
}

void colorWipe(uint32_t color) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

void printWiFiStatus() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
