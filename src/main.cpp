#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "rgb_lcd.h"
#include "Ultrasonic.h"
#include "WiFiS3.h"
#include <ArduinoBLE.h>

// Netwerk gegevens om in te kunnen loggen op netwerk
char ssid[32];
char pass[64];

// UUID voor de bluetooth module
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1"

// Pin declaraties
#define LED_PIN     4
#define NUM_LEDS    60
#define LCD_ADDRESS 0x62
#define ULTRASONIC_PIN  8
#define TURBIDITY_PIN A2
#define LIGHT_SENSOR_PIN A0
#define TEMPERATURE_PIN 2
#define FLOW_PIN  3
#define PH_SENSOR_PIN A1   
#define BUTTON_PIN 5
#define ATTINY1_HIGH_ADDR   0x78
#define ATTINY2_LOW_ADDR   0x77
#define NO_TOUCH       0xFE

// Variables
char turbidity_status[10] = "Null";
unsigned char low_data[8] = {0};
unsigned char high_data[12] = {0};
#define THRESHOLD      100
unsigned long int avgValue;
float b;
int buf[10], temp;
volatile int flow_frequency;
unsigned int l_hour;
unsigned long currentTime;
unsigned long cloopTime;
unsigned long previousMillis = 0;
const long interval = 1000;
int currentScreen = 0;
int water_level = 0;
int distance_cm = 0;
int light_level = 0;
float tempC = 0.0;
float phValueCurrent = 0.0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50; 

// Specifieke libraty variables
OneWire oneWire(TEMPERATURE_PIN);
DallasTemperature sensors(&oneWire);
rgb_lcd lcd;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_RGB + NEO_KHZ800);
Ultrasonic ultrasonic(ULTRASONIC_PIN);
BLEService commandService(SERVICE_UUID); 
BLECharacteristic commandCharacteristic(CHARACTERISTIC_UUID, BLERead | BLEWrite, 20);

// Predeclered functions
void colorWipe(uint32_t color);
void printWiFiStatus();
float getTemp();
void checkTurbidity();
void flow();
void readPH();
void getHigh12SectionValue(void);
void getLow8SectionValue(void);
int getWaterLevel(void);
void scanNetworks();
void handleNetworkSelection(String request);
void updateDisplay();
void updateCurrentScreen();

void setup() {
    // Begin functions
    Serial.begin(115200);
    while(!Serial);
    strip.begin();
    Wire.begin();
    lcd.begin(16, 2);
    lcd.setRGB(255, 255, 255);
    lcd.print("FishSee!");
    if (!BLE.begin()) {
        lcd.setCursor(0,1);
        lcd.print("BT Faal, Reboot");
        while (1);
    }

    // Function handlers
    colorWipe(strip.Color(0, 0, 0));
    BLE.setLocalName("FishSee");
    BLE.setAdvertisedService(commandService);
    commandService.addCharacteristic(commandCharacteristic);
    BLE.addService(commandService);
    BLE.advertise();
    lcd.setCursor(0,1);
    lcd.print("Bluetooth actief!");
  
    // Initializeren pinnen
    pinMode(TURBIDITY_PIN, INPUT);
    pinMode(FLOW_PIN, INPUT);
    pinMode(BUTTON_PIN, OUTPUT);
    digitalWrite(FLOW_PIN, HIGH); 
    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flow, RISING); 

    currentTime = millis();
    cloopTime = currentTime;
    delay(500);
}

void loop() {
    BLEDevice central = BLE.central();
    if(central) {
        lcd.setCursor(0,0);
        lcd.print("Bluetooth gelukt");
        lcd.setCursor(0,1);
        lcd.print(central.address());
        while(central.connected()) {
            if(commandCharacteristic.written()) {
                int len = commandCharacteristic.valueLength();
                char command[len + 1];
                memcpy(command, commandCharacteristic.value(), len);
                command[len] = '\0';
                if(strcmp(command, "aan") == 0) {
                  colorWipe(strip.Color(255, 255, 255));
                } else if(strcmp(command, "uit") == 0) {
                  colorWipe(strip.Color(0, 0, 0));
                } else {
                  Serial.println("Unknown command");
                }
            }
        }
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Bluetooth:");
        lcd.setCursor(0,1);
        lcd.print("Geen verbinding");
        delay(1000);
    }
    unsigned long currentMillis = millis();
    currentTime = millis();
    if(currentTime >= (cloopTime + 1000)) {
      cloopTime = currentTime; 
      noInterrupts();
      l_hour = (flow_frequency * 60 / 7.5); 
      flow_frequency = 0; 
      interrupts();
    }
    distance_cm = ultrasonic.distanceRead(CM);
    light_level = analogRead(LIGHT_SENSOR_PIN);
    water_level = getWaterLevel();
    sensors.requestTemperatures();
    tempC = sensors.getTempCByIndex(0);
    int reading = digitalRead(BUTTON_PIN);
    Serial.print("Button State:");
    Serial.println(reading);
    if (reading == HIGH ) {
        Serial.println("Gedrukt");
        currentScreen++;
        if (currentScreen > 6) {
            currentScreen = 0;
        }
        Serial.println(currentScreen);
        updateCurrentScreen();
    }
    if (light_level < 400) {
        colorWipe(strip.Color(255, 255, 255));
    } else {
        colorWipe(strip.Color(0, 0, 0));
    }
    updateCurrentScreen();
}

void flow() {
    flow_frequency++;
}

void checkTurbidity() {
    int sensorValue = analogRead(TURBIDITY_PIN); 
    int turbidity = map(sensorValue, 0, 640, 100, 0);  
    if(turbidity < 20) {
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

void readPH(){
    for(int i=0;i<10;i++) { 
        buf[i]=analogRead(PH_SENSOR_PIN);
        delay(10);
    }
    for(int i=0;i<9;i++) {
        for(int j=i+1;j<10;j++) {
            if(buf[i]>buf[j]) {
                temp=buf[i];
                buf[i]=buf[j];
                buf[j]=temp;
            }
        }
    }
    avgValue=0;
    for(int i=2;i<8;i++) {                     
        avgValue+=buf[i];
    }
    float phValue=(float)avgValue*5.0/1024/6; 
    phValue=3.5*phValue;               
    phValueCurrent = phValue;       
}

void getHigh12SectionValue(void) {
    Wire.beginTransmission(ATTINY1_HIGH_ADDR);
    Wire.write(NO_TOUCH);
    Wire.endTransmission();
    delay(50);
    Wire.requestFrom(ATTINY1_HIGH_ADDR, 12);
    if (Wire.available() == 12) {
        for (int i = 0; i < 12; i++) {
        high_data[i] = Wire.read();
        }
    }
}

void getLow8SectionValue(void) {
  Wire.beginTransmission(ATTINY2_LOW_ADDR);
  Wire.write(NO_TOUCH);
  Wire.endTransmission();
  delay(50);
  Wire.requestFrom(ATTINY2_LOW_ADDR, 8);
  if (Wire.available() == 8) {
    for (int i = 0; i < 8; i++) {
      low_data[i] = Wire.read();
    }
  }
}

int getWaterLevel() {
    getLow8SectionValue();
    getHigh12SectionValue();

    uint32_t touch_val = 0;
    uint8_t trig_section = 0;
    for (int i = 0; i < 8; i++) {
        if (low_data[i] > THRESHOLD) {
        touch_val |= 1 << i;
        }
    }
    for (int i = 0; i < 12; i++) {
        if (high_data[i] > THRESHOLD) {
        touch_val |= (uint32_t)1 << (8 + i);
        }
    }
    while (touch_val & 0x01) {
        trig_section++;
        touch_val >>= 1;
    }

    return trig_section * 5;  
}

void updateCurrentScreen() {
  lcd.clear();

  switch (currentScreen) {
    case 0:
        lcd.clear();
        lcd.print("Water Level:");
        lcd.setCursor(0, 1);
        lcd.print(water_level);
        lcd.print("%");
        break;
    case 1:
        lcd.clear();
        lcd.print("Distance:");
        lcd.setCursor(0, 1);
        lcd.print(distance_cm);
        lcd.print(" cm");
        break;
    case 2:
        lcd.clear();
        lcd.print("Light Level:");
        lcd.setCursor(0, 1);
        lcd.print(light_level);
        break;
    case 3:
        lcd.clear();
        lcd.print("Temperature:");
        lcd.setCursor(0, 1);
        lcd.print(tempC);
        lcd.print(" C");
        break;
    case 4:
        lcd.clear();
        checkTurbidity();
        lcd.print("Turbidity:");
        lcd.setCursor(0, 1);
        lcd.print(turbidity_status);
        break;
    case 5:
        lcd.clear();
        lcd.print("PH:"); 
        lcd.setCursor(0, 1);
        lcd.print(phValueCurrent);
        break;
    case 6:
        lcd.clear();
        lcd.print("Flow sensor:");
        lcd.setCursor(0,1);
        lcd.print(flow_frequency);
        break;
  }
}