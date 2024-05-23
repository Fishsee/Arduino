#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "rgb_lcd.h"
#include "Ultrasonic.h"
#include <WiFi.h>

const char* ssid = "Broodjegehaktbal";
const char* password = "12345678";
int status = WL_IDLE_STATUS;

#define LED_PIN     4
#define NUM_LEDS    60

#define LCD_ADDRESS 0x62

#define ULTRASONIC_PIN  3

#define TURBIDITY_PIN A2

#define LIGHT_SENSOR_PIN A0

#define TEMPERATURE_PIN 2

#define PH_SENSOR_PIN A1          

char turbidity_status[10] = "Null";

OneWire oneWire(TEMPERATURE_PIN);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

rgb_lcd lcd;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
Ultrasonic ultrasonic(ULTRASONIC_PIN);

unsigned char low_data[8] = {0};
unsigned char high_data[12] = {0};

#define NO_TOUCH       0xFE
#define THRESHOLD      100
#define ATTINY1_HIGH_ADDR   0x78
#define ATTINY2_LOW_ADDR   0x77

void colorWipe(uint32_t color);
void printWiFiStatus();
float getTemp();
void checkTurbidity();
void flow();
void readPH();
void getHigh12SectionValue(void);
void getLow8SectionValue(void);
int getWaterLevel(void);

unsigned long int avgValue;  // Store the average value of the sensor feedback
float b;
int buf[10], temp;
volatile int flow_frequency; // Measures flow sensor pulses
unsigned int l_hour; // Calculated litres/hour
const byte flowsensor = 0; // Sensor Input
unsigned long currentTime;
unsigned long cloopTime;

unsigned long previousMillis = 0;
const long interval = 1000;

void setup() {
  strip.begin();
  colorWipe(strip.Color(0, 0, 0));
  Serial.begin(9600);
  Wire.begin();
  pinMode(TURBIDITY_PIN, INPUT);
  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH); // Optional Internal Pull-Up
  attachInterrupt(digitalPinToInterrupt(flowsensor), flow, RISING); // Setup Interrupt
  currentTime = millis();
  cloopTime = currentTime;

  // Initialiseer de Grove RGB LCD
  lcd.begin(16, 2);
  lcd.setRGB(0, 255, 0);
  lcd.print("Hello, world!");
  delay(500);
  lcd.clear();

  // Initialiseer de waterniveausensor
  // Geen speciale initialisatie nodig voor I2C

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

void flow() // Interrupt function
{
   flow_frequency++;
}

void loop() {
  unsigned long currentMillis = millis();

  currentTime = millis();
   // Every second, calculate and print litres/hour
   if(currentTime >= (cloopTime + 1000))
   {
      cloopTime = currentTime; // Updates cloopTime

      // Disable interrupts while calculating
      noInterrupts();
      // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
      l_hour = (flow_frequency * 60 / 7.5); // (Pulse frequency x 60 min) / 7.5Q = flowrate in L/hour
      flow_frequency = 0; // Reset Counter
      interrupts(); // Enable interrupts

      // Serial.print(l_hour, DEC); // Print litres/hour
      // Serial.println(" L/hour");
   }

  // Non-blocking delay
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Measure distance with the ultrasonic sensor
    int distance_cm = (int)ultrasonic.distanceRead(CM);

    // Measure light level
    int light_level = analogRead(LIGHT_SENSOR_PIN);

    // Measure water level
    int water_level = getWaterLevel();

    // Calculate water level percentage
    float percentage_water = (float)(water_level) * 5;  // Each step is approximately 5%

    // Measure temperature
    sensors.requestTemperatures();
    readPH();

    // Display measurements on the LCD
    lcd.clear();
    lcd.setRGB(255, 255, 255);
    lcd.setCursor(0, 0);
    lcd.print("W:");
    lcd.print(water_level);
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

void readPH(){
  for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(PH_SENSOR_PIN);
    delay(10);
  }
  for(int i=0;i<9;i++)        //sort the analog from small to large
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)                      //take the average value of 6 center sample
    avgValue+=buf[i];
  Serial.println(avgValue);
  float phValue=(float)avgValue*5.0/1024/6; //convert the analog into millivolt
  phValue=3.5*phValue+0.54;                      //convert the millivolt into pH value
  Serial.println(phValue);
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

  return trig_section * 5;  // Each section represents approximately 5% of the water level
}

void getHigh12SectionValue() {
  memset(high_data, 0, sizeof(high_data));
  Wire.requestFrom(ATTINY1_HIGH_ADDR, 12);
  while (12 != Wire.available());

  for (int i = 0; i < 12; i++) {
    high_data[i] = Wire.read();
  }
  delay(10);
}

void getLow8SectionValue() {
  memset(low_data, 0, sizeof(low_data));
  Wire.requestFrom(ATTINY2_LOW_ADDR, 8);
  while (8 != Wire.available());

  for (int i = 0; i < 8; i++) {
    low_data[i] = Wire.read();
  }
  delay(10);
}
