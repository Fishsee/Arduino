#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <HTTPClient.h> 
#include <ArduinoJson.h> 
#include <OneWire.h>
#include <NewPing.h>
#include "rgb_lcd.h"
#include <Grove_LED_Bar.h>
#include <Servo.h>

// Definieer WiFi-credentials
#define WIFI_SSID "WIFISSID"
#define WIFI_PASSWORD "WIFIWW"

// Defineer API-URL
#define API_URL "http://example.com/api/data"
#define WIFI_API_URL "http://example.com/api/wifi"

// Pin definities
#define PH_SENSOR_PIN A0
#define DS18B20_PIN 2
#define SONAR_TRIG_PIN 7
#define SONAR_ECHO_PIN 12
#define SERVO_PH_PIN 8
#define SERVO_FEED_PIN 9
#define PH_THRESHOLD_LOW 6.5
#define PH_THRESHOLD_HIGH 7.5
#define LIGHT_SENSOR_PIN A1
#define LED_STRIP_PIN 3
#define WATER_LEVEL_PIN A2
#define WATER_LEVEL_THRESHOLD 10 // Drempelwaarde voor waterpeil
#define WATER_PUMP_PIN 4 // Pin voor aansturing waterpomp

char ssid[] = WIFI_SSID;
char pass[] = WIFI_PASSWORD;
int status = WL_IDLE_STATUS;

rgb_lcd lcd;
Grove_LED_Bar ledBar(LED_STRIP_PIN, 2, 0);  // Klok pin, Data pin, Oriëntatie
OneWire ds(DS18B20_PIN);  // op digitale pin 2
NewPing sonar(SONAR_TRIG_PIN, SONAR_ECHO_PIN, 200);
Servo servoPH;
Servo servoFeed;

unsigned long previousPHServoTime = 0;
unsigned long previousFeedServoTime = 0;
const long servoInterval = 10000; // Interval voor servo beweging in milliseconden

// Functiedeclaraties
void setup();
void loop();
void connectToWiFi();
void printWiFiStatus();
float readPH();
float readTemperature();
int readSonar();
int readLightLevel();
int readWaterLevel();
void controlPHIndicator(float pHValue);
void controlLEDStrip(int lightLevel);
void controlWaterPump(int waterLevel);
void controlPHServo();
void controlFeedServo();
void sendSensorData(float pHValue, float temperature, int waterLevel, int sonarDistance);
void setWiFiCredentials(String ssid, String password);

void setup() {
  // Initialisatie
  Serial.begin(9600);
  lcd.begin(16, 2);
  ledBar.begin();
  ledBar.setGreenToRed(true);
  servoPH.attach(SERVO_PH_PIN);
  servoFeed.attach(SERVO_FEED_PIN);

  // Verbinding maken met WiFi
  connectToWiFi();
  pinMode(WATER_PUMP_PIN, OUTPUT);
}

void loop() {
  // Huidige tijd ophalen
  unsigned long currentMillis = millis();

  // Sensorwaarden uitlezen
  float pHValue = readPH();
  float temperature = readTemperature();
  int distance = readSonar();
  int lightLevel = readLightLevel();
  int waterLevel = readWaterLevel();

  // Sensoren aansturen
  controlPHIndicator(pHValue);
  controlLEDStrip(lightLevel);
  controlWaterPump(waterLevel);

  // Servo's aansturen met tussenpozen
  if (currentMillis - previousPHServoTime >= servoInterval) {
    previousPHServoTime = currentMillis;
    controlPHServo();
  }

  if (currentMillis - previousFeedServoTime >= servoInterval) {
    previousFeedServoTime = currentMillis;
    controlFeedServo();
  }

  // Sensorgegevens naar API sturen
  sendSensorData(pHValue, temperature, waterLevel, distance);

  // Wachten voordat de lus opnieuw wordt uitgevoerd
  delay(1000);
}

void connectToWiFi() {
  // Verbinding maken met WiFi-netwerk
  while (status != WL_CONNECTED) {
    Serial.print("Poging tot verbinden met SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }
  Serial.println("Verbonden met WiFi");
  printWiFiStatus();
}

void printWiFiStatus() {
  // WiFi-status afdrukken
  IPAddress ip = WiFi.localIP();
  Serial.print("IP-adres: ");
  Serial.println(ip);
}

float readPH() {
  // pH-waarde uitlezen
  int sensorValue = analogRead(PH_SENSOR_PIN);
  float voltage = sensorValue * (5.0 / 1024.0);
  float pHValue = 3.5 * voltage; // Aannemende dat calibratie is gedaan
  return pHValue;
}

float readTemperature() {
  // Temperatuur uitlezen
  byte data[12];
  byte addr[8];
  ds.reset();
  ds.search(addr);
  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);

  delay(1000);

  ds.reset();
  ds.select(addr);
  ds.write(0xBE);

  for (int i = 0; i < 9; i++) {
    data[i] = ds.read();
  }

  int16_t raw = (data[1] << 8) | data[0];
  float temperature = (float)raw / 16.0;
  return temperature;
}

int readSonar() {
  // Afstand met ultrasone sensor meten
  int distance = sonar.ping_cm();
  return distance;
}

int readLightLevel() {
  // Lichtniveau uitlezen
  int lightLevel = analogRead(LIGHT_SENSOR_PIN);
  return lightLevel;
}

int readWaterLevel() {
  // Waterpeil uitlezen
  int waterLevel = analogRead(WATER_LEVEL_PIN);
  return map(waterLevel, 0, 1023, 0, 20); // Mapping analoge waarde naar waterpeil in cm
}

void controlPHIndicator(float pHValue) {
  // pH-indicator besturen
  if (pHValue < PH_THRESHOLD_LOW || pHValue > PH_THRESHOLD_HIGH) {
    lcd.setRGB(255, 0, 0); // Rood
  } else {
    lcd.setRGB(0, 255, 0); // Groen
  }
}

void controlLEDStrip(int lightLevel) {
  // LED-strip besturen
  if (lightLevel < 100) { // Aannemende dat deze drempel duisternis aangeeft
    ledBar.setLevel(10); // Volledige helderheid
  } else {
    ledBar.setLevel(0); // Uitzetten
  }
}

void controlWaterPump(int waterLevel) {
  // Waterpomp besturen op basis van waterpeil
  if (waterLevel < WATER_LEVEL_THRESHOLD) {
    digitalWrite(WATER_PUMP_PIN, HIGH); // Pomp aanzetten
  } else {
    digitalWrite(WATER_PUMP_PIN, LOW); // Pomp uitzetten
  }
}

void controlPHServo() {
  // pH-servo besturen
  servoPH.write(90); // Aanpassen van de hoek indien nodig
  delay(1000); // Wachten tot de servo de gewenste positie bereikt
}

void controlFeedServo() {
  // Voerservo besturen
  servoFeed.write(90); // Aanpassen van de hoek indien nodig
  delay(1000); // Wachten tot de servo de gewenste positie bereikt
}

void sendSensorData(float pHValue, float temperature, int waterLevel, int sonarDistance) {
  // Sensorgegevens verzenden naar Laravel API
  WiFiClient client;
  HTTPClient http;

  // JSON-object maken
  StaticJsonDocument<200> doc;
  doc["pH"] = pHValue;
  doc["temperature"] = temperature;
  doc["waterLevel"] = waterLevel;
  doc["sonarDistance"] = sonarDistance;

  // JSON naar string omzetten
  String jsonString;
  serializeJson(doc, jsonString);

  // HTTP POST-verzoek maken
  http.begin(client, API_URL);
  http.addHeader("Content-Type", "application/json");

  // Verzoek verzenden
  int httpResponseCode = http.POST(jsonString);

  // Controleer op succesvolle verzending
  if (httpResponseCode > 0) {
    Serial.print("Sensorgegevens verzonden naar API. Statuscode: ");
    Serial.println(httpResponseCode);
  } else {
    Serial.print("Fout bij verzenden van sensorgegevens. Statuscode: ");
    Serial.println(httpResponseCode);
  }

  // Verbinding verbreken
  http.end();
}

void setWiFiCredentials(String ssid, String password) {
  // Instellen van WiFi-gegevens
  WiFi.begin(ssid, password);

  // Wachten tot de verbinding tot stand is gebracht
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Verbinden met WiFi...");
  }

  Serial.println("WiFi verbonden");
  Serial.println("IP-adres: ");
  Serial.println(WiFi.localIP());
}
