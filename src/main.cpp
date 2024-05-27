#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <rgb_lcd.h>
#include <Ultrasonic.h>
#include <WiFiS3.h>
#include <ArduinoBLE.h>
#include <HttpClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>

// Network credentials
char ssid[32];
char pass[64];

// UUID for the Bluetooth module
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1"

// Pin declarations
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
#define SERVO_PIN 7

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
bool bluetoothActive = true; // Flag to indicate if Bluetooth is active
unsigned long lastSentTime = 0; // To track the last time data was sent
const long sendInterval = 3600000; // 1 hour in milliseconds
#define servoOpen 1600 // value for servo being open
#define servoClosed 950 // value for servo being closed
#define servoDelay 20 


// Specific library variables
OneWire oneWire(TEMPERATURE_PIN);
DallasTemperature sensors(&oneWire);
rgb_lcd lcd;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_RGB + NEO_KHZ800);
Ultrasonic ultrasonic(ULTRASONIC_PIN);
BLEService commandService(SERVICE_UUID); 
BLECharacteristic commandCharacteristic(CHARACTERISTIC_UUID, BLERead | BLEWrite, 128);

// Arrays to hold network information
const int MAX_NETWORKS = 20;
char networkSSIDs[MAX_NETWORKS][32];
int networkEncTypes[MAX_NETWORKS];
int numNetworks = 0;

// Function prototypes
void colorWipe(uint32_t color);
void printWiFiStatus();
float getTemp();
void checkTurbidity();
void flow();
void readPH();
void getHigh12SectionValue();
void getLow8SectionValue();
int getWaterLevel();
void scanNetworks();
void handleNetworkSelection();
void updateDisplay();
void updateCurrentScreen();
void displayMessage(String line1, String line2);
void connectToNetwork();
void enterPassword(const char* pwd);
String gatherSensorDataAsJson();
void sendSensorDataToApi(String jsonData);
void servo(int pulse);


void setup() {
    // Initialization
    Serial.begin(115200);
    while(!Serial);
    strip.begin();
    Wire.begin();
    lcd.begin(16, 2);
    lcd.setRGB(255, 255, 255);
    lcd.print("FishSee!");
    if (!BLE.begin()) {
        lcd.setCursor(0,1);
        lcd.print("BT Fail, Reboot");
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
    lcd.print("Bluetooth active!");
  
    // Initialize pins
    pinMode(TURBIDITY_PIN, INPUT);
    pinMode(FLOW_PIN, INPUT);
    pinMode(BUTTON_PIN, OUTPUT);
    digitalWrite(FLOW_PIN, HIGH); 
    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flow, RISING); 
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);

    currentTime = millis();
    cloopTime = currentTime;
    delay(500);
}

void loop() {
    if (bluetoothActive) {
        // Handle Bluetooth operations
        BLEDevice central = BLE.central();
        if(central) {
            displayMessage("Bluetooth connected", central.address());
            while(central.connected()) {
                if(commandCharacteristic.written()) {
                    int len = commandCharacteristic.valueLength();
                    char command[len + 1];
                    memcpy(command, commandCharacteristic.value(), len);
                    command[len] = '\0';
                    Serial.println(command); // For debugging purposes

                    if(strcmp(command, "aan") == 0) {
                        colorWipe(strip.Color(255, 255, 255));
                    } else if(strcmp(command, "uit") == 0) {
                        colorWipe(strip.Color(0, 0, 0));
                    } else if(strcmp(command, "scan") == 0) {
                        bluetoothActive = false; // Temporarily deactivate Bluetooth
                        scanNetworks();
                        bluetoothActive = true; // Reactivate Bluetooth after WiFi operations
                    } else if(strcmp(command, "list") == 0) {
                        bluetoothActive = false; // Temporarily deactivate Bluetooth
                        handleNetworkSelection();
                        bluetoothActive = true; // Reactivate Bluetooth after WiFi operations
                    } else if(strcmp(command, "status") == 0) {
                        bluetoothActive = false; // Temporarily deactivate Bluetooth
                        printWiFiStatus();
                        bluetoothActive = true; // Reactivate Bluetooth after WiFi operations
                    } else if(strncmp(command, "connect", 7) == 0) {
                        int networkIndex = atoi(&command[8]) - 1; // Extract network index
                        if(networkIndex >= 0 && networkIndex < numNetworks) {
                            strncpy(ssid, networkSSIDs[networkIndex], sizeof(ssid) - 1);
                            ssid[sizeof(ssid) - 1] = '\0'; // Ensure null-terminated string
                            displayMessage("Selected:", ssid);
                        } else {
                            Serial.println("Invalid network index");
                        }
                    } else if(strncmp(command, "password", 8) == 0) {
                        strncpy(pass, &command[9], sizeof(pass) - 1);
                        pass[sizeof(pass) - 1] = '\0'; // Ensure null-terminated string
                        bluetoothActive = false; // Temporarily deactivate Bluetooth
                        connectToNetwork();
                        bluetoothActive = true; // Reactivate Bluetooth after WiFi operations
                    } else {
                        Serial.println("Unknown command");
                    }
                }
            }
            displayMessage("Bluetooth:", "No connection");
            delay(1000);
        }
    } else {
        // Handle WiFi operations
        unsigned long wifiTimeout = millis() + 30000; // Timeout for WiFi operations
        scanNetworks();
        while (millis() < wifiTimeout) {
            // Wait for WiFi operations to complete
        }
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
        Serial.println("Pressed");
        currentScreen++;
        if (currentScreen > 6) {
            currentScreen = 0;
        }
        Serial.println(currentScreen);
        updateCurrentScreen();
    }
    if (light_level < 200) {
        colorWipe(strip.Color(255, 255, 255));
    } else {
        colorWipe(strip.Color(0, 0, 0));
    }
    updateCurrentScreen();

  if (WiFi.status() == WL_CONNECTED) {
    // Check if it's time to send sensor data to the API
    if (currentMillis - lastSentTime >= sendInterval) {
        lastSentTime = currentMillis;
        String jsonData = gatherSensorDataAsJson();
        sendSensorDataToApi(jsonData);
    }
  }
}

String gatherSensorDataAsJson() {
    // Gather all sensor data and format it into a JSON string
    DynamicJsonDocument doc(512); // Adjust size as needed
    doc["tempC"] = tempC;
    doc["distance_cm"] = distance_cm;
    doc["light_level"] = light_level;
    doc["water_level"] = water_level;
    doc["flow_rate"] = l_hour;
    doc["phValueCurrent"] = phValueCurrent;
    doc["turbidity_status"] = turbidity_status;
    String jsonData;
    serializeJson(doc, jsonData);
    return jsonData;
}

void sendSensorDataToApi(arduino::String sensorData) {
    WiFiClient wifiClient;
    HttpClient httpClient(wifiClient);

    const char* serverName = "example.com";
    const char* endpoint = "/api/data";
    const char* contentType = "application/json";

    // Begin the HTTP request
    httpClient.beginRequest();
    httpClient.post(serverName, endpoint, contentType, sensorData.c_str());
    httpClient.endRequest();

    // Read the response status code and body
    int statusCode = httpClient.responseStatusCode();
    String response;
    if (statusCode >= 0) {
        httpClient.skipResponseHeaders();
        while (httpClient.available()) {
            char c = httpClient.read();
            response += c;
        }
    } else {
        response = "Error receiving response";
    }

    // Print the response (optional)
    Serial.print("Status code: ");
    Serial.println(statusCode);
    Serial.print("Response: ");
    Serial.println(response);
}

void displayMessage(String line1, String line2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
}

void scanNetworks() {
    displayMessage("Wifi:", "Scanning....");
    if (WiFi.status() == WL_NO_SHIELD) {
        displayMessage("Wifi:", "No module");
        while (true);
    }
    
    numNetworks = WiFi.scanNetworks();
    if (numNetworks == 0) {
        displayMessage("Wifi:", "No Networks");
    } else {
        for (int i = 0; i < numNetworks && i < MAX_NETWORKS; ++i) {
            strncpy(networkSSIDs[i], WiFi.SSID(i), sizeof(networkSSIDs[i]) - 1);
            networkSSIDs[i][sizeof(networkSSIDs[i]) - 1] = '\0'; // Ensure null-terminated string
            networkEncTypes[i] = WiFi.encryptionType(i);
        }
    }   
    displayMessage("Wifi:", "Gevonden");
}

void handleNetworkSelection() {
    displayMessage("Select Network", "");
    for (int i = 0; i < numNetworks && i < MAX_NETWORKS; ++i) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(networkSSIDs[i]);
        Serial.print(" (");
        Serial.print(WiFi.RSSI(i));
        Serial.println(")");
    }
    if (commandCharacteristic.written()) {
        const uint8_t* value = commandCharacteristic.value();
        int len = commandCharacteristic.valueLength();
        char command[len + 1];
        strncpy(command, (char*)value, len);
        command[len] = '\0';
        int selection = atoi(command);
        if (selection > 0 && selection <= numNetworks) {
            displayMessage("Selected:", networkSSIDs[selection - 1]);
            strncpy(ssid, networkSSIDs[selection - 1], sizeof(ssid) - 1);
            ssid[sizeof(ssid) - 1] = '\0'; // Ensure null-terminated string

            // Assume password entry via Bluetooth
            displayMessage("Enter Password:", "");
            while (!commandCharacteristic.written()) {
                // Wait for password entry
            }
            if (commandCharacteristic.written()) {
                strncpy(pass, (char*)commandCharacteristic.value(), sizeof(pass) - 1);
                pass[sizeof(pass) - 1] = '\0'; // Ensure null-terminated string
                connectToNetwork();
            }
        }
    }
}

void connectToNetwork() {
    displayMessage("Connecting to:", ssid);
    WiFi.begin(ssid, pass);

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startTime < 10000)) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        displayMessage("Connected to:", ssid);
        printWiFiStatus();
    } else {
        displayMessage("Failed to", "connect");
    }
}

void printWiFiStatus() {
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    long rssi = WiFi.RSSI();
    Serial.print("Signal strength (RSSI):");
    Serial.println(rssi);
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

void getHigh12SectionValue() {
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

void getLow8SectionValue() {
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

void servo(int pulse) {
      for (int i = 0; i < 8; i++) {
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(SERVO_PIN, LOW);
      }
}

void updateCurrentScreen() {
  lcd.clear();

  switch (currentScreen) {
    case 0:
        lcd.print("Water Level:");
        lcd.setCursor(0, 1);
        lcd.print(water_level);
        lcd.print("%");
        break;
    case 1:
        lcd.print("Distance:");
        lcd.setCursor(0, 1);
        lcd.print(distance_cm);
        lcd.print(" cm");
        break;
    case 2:
        lcd.print("Light Level:");
        lcd.setCursor(0, 1);
        lcd.print(light_level);
        break;
    case 3:
        lcd.print("Temperature:");
        lcd.setCursor(0, 1);
        lcd.print(tempC);
        lcd.print(" C");
        break;
    case 4:
        checkTurbidity();
        lcd.print("Turbidity:");
        lcd.setCursor(0, 1);
        lcd.print(turbidity_status);
        break;
    case 5:
        lcd.print("PH:"); 
        lcd.setCursor(0, 1);
        lcd.print(phValueCurrent);
        break;
    case 6:
        lcd.print("Flow sensor:");
        lcd.setCursor(0,1);
        lcd.print(flow_frequency);
        break;
  }
}