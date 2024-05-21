#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
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

#define TURBIDITY_PIN A2

#define LIGHT_SENSOR_PIN A0

#define WATER_LEVEL_PIN A3

#define TEMPERATURE_PIN 2

char turbidity_status[10] = "Null";

OneWire oneWire(TEMPERATURE_PIN);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

rgb_lcd lcd;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
Ultrasonic ultrasonic(ULTRASONIC_PIN);

void colorWipe(uint32_t color);
void printWiFiStatus();
float getTemp();
void checkTurbidity();

unsigned long previousMillis = 0;
const long interval = 1000;

void setup() {
  strip.begin();
  colorWipe(strip.Color(0, 0, 0));
  Serial.begin(9600);
  Wire.begin();
  pinMode(TURBIDITY_PIN, INPUT);

  // Initialiseer de Grove RGB LCD
  lcd.begin(16, 2);
  lcd.setRGB(0, 255, 0);
  lcd.print("Hello, world!");
  delay(500);
  lcd.clear();

  // // Check for the WiFi module
  // if (WiFi.status() == WL_NO_SHIELD) {
  //   lcd.print("WiFi module fail");
  //   while (true);
  // }

  // // Attempt to connect to WiFi network
  // lcd.print("Attempting to connect");
  // lcd.setCursor(0, 1);
  // lcd.print(ssid);
  // lcd.setCursor(0, 0);
  // delay(500);

  // while (status != WL_CONNECTED) {
  //   lcd.print("Connecting...");
  //   status = WiFi.begin(ssid, password);

  //   // Wait 10 seconds for connection
  //   delay(5000);

  //   if (status == WL_CONNECTED) {
  //     lcd.clear();
  //     lcd.print("Connected to WiFi");
  //     delay(500);
  //     printWiFiStatus();
  //   } else {
  //     lcd.clear();
  //     lcd.print("Failed to connect");
  //   }
  // }
}

void loop() {
  unsigned long currentMillis = millis();

  // Non-blocking delay
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Measure distance with the ultrasonic sensor
    int distance_cm = (int)ultrasonic.distanceRead(CM);

    // Measure light level
    int light_level = analogRead(LIGHT_SENSOR_PIN);

    // Measure water level
    int water_level = analogRead(WATER_LEVEL_PIN);

    // Calculate water level percentage
    float percentage_water = (float)(1023 - water_level) / 1023 * 100;

    // Measure temperature
    sensors.requestTemperatures();

    // Display measurements on the LCD
    lcd.clear();
    lcd.setRGB(255, 255, 255);
    lcd.setCursor(0, 0);
    lcd.print("W:");
    lcd.print(percentage_water, 1);
    lcd.print("% A:");
    lcd.print(distance_cm);
    lcd.print("cm");
    lcd.setCursor(0, 1);
    lcd.print("C:");
    lcd.print(sensors.getTempCByIndex(0));
    lcd.print(" T:");
    checkTurbidity();
    lcd.print(turbidity_status);

    // Control the LED strip based on light level
    if (light_level < 400) {
      colorWipe(strip.Color(255, 255, 255));
    } else {
      colorWipe(strip.Color(0, 0, 0));
    }
  }
}

void checkTurbidity() {
  int sensorValue = analogRead(TURBIDITY_PIN);  // Read the analog sensor value
  int turbidity = map(sensorValue, 0, 640, 100, 0);  // Map the sensor value to a turbidity percentage

  Serial.print(sensorValue);
  // Determine and display the water quality 0 is clear 1 is cloudy 2 is dirty
  if (turbidity < 20) {
    strcpy(turbidity_status, "Clear");
  } else if (turbidity <= 50) {
    strcpy(turbidity_status, "Unclear");
  } else {
    strcpy(turbidity_status, "Gross");
  }
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
