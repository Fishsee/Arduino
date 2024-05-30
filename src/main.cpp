#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <rgb_lcd.h>
#include <Ultrasonic.h>
#include <ArduinoBLE.h>
#include <ArduinoHttpClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>

// Network credentials
char ssid[32];
char pass[64];

// UUIDs for Bluetooth module
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1"

// Pin definitions
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
#define MOTOR_PIN 0

// Variables
int turbidity_status = 0;
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
int turbidity = 0;
float tempC = 0.0;
float phValueCurrent = 0.0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50; 
bool bluetoothActive = true;
unsigned long lastSentTime = 0;
const long sendInterval = 3600000;
#define servoOpen 1600
#define servoClosed 950
#define servoDelay 20 
int ssidAddress = 0;
int passwordAddress = 32;
unsigned long lastApiCallTime = 0;    // Last time the sensor data was sent
const unsigned long apiCallInterval = 30000;  // Interval to send sensor data (e.g., every 60 seconds)

// Specific library instances
OneWire oneWire(TEMPERATURE_PIN);
DallasTemperature sensors(&oneWire);
rgb_lcd lcd;
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_RGB + NEO_KHZ800);
Ultrasonic ultrasonic(ULTRASONIC_PIN);
BLEService commandService(SERVICE_UUID); 
BLECharacteristic commandCharacteristic(CHARACTERISTIC_UUID, BLERead | BLEWrite, 128);

// Arrays for network information
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
String gatherSensorDataAsJson();
void sendSensorDataToApi(String jsonData);
void servo(int pulse);
void writeStringToEEPROM(int address, String data);
void readStringFromEEPROM(int address, char* buffer, int bufferSize);
bool isEEPROMEmpty();
void checkBluetoothConnection();

// Setup function
void setup() {
    Serial.begin(115200);
    while(!Serial);
    strip.begin();
    Wire.begin();
    lcd.begin(16, 2);
    lcd.setRGB(255, 255, 255);
    displayMessage("FishSee", "");
    delay(1000);
    colorWipe(strip.Color(0, 0, 0)); // Turn off LEDs at startup.
    checkBluetoothConnection();
    
    // Pin modes
    pinMode(TURBIDITY_PIN, INPUT);
    pinMode(FLOW_PIN, INPUT);
    pinMode(BUTTON_PIN, OUTPUT);
    digitalWrite(FLOW_PIN, HIGH); 
    attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flow, RISING); 
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);
    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, HIGH);
    
    if (isEEPROMEmpty()) {
        displayMessage("Wifi:", "Not Set");
    } else {
        readStringFromEEPROM(ssidAddress, ssid, sizeof(ssid));
        readStringFromEEPROM(passwordAddress, pass, sizeof(pass));
        connectToNetwork();
    }

    currentTime = millis();
    cloopTime = currentTime;
    delay(500);
}

// Loop function
void loop() {
    unsigned long currentMillis = millis();
    if (bluetoothActive) {
        BLEDevice central = BLE.central();
        if(central) {
            displayMessage("Bluetooth connected", central.address());
            while(central.connected()) {
                if(commandCharacteristic.written()) {
                    int len = commandCharacteristic.valueLength();
                    char command[len + 1];
                    memcpy(command, commandCharacteristic.value(), len);
                    command[len] = '\0';
                    Serial.println(command); // Debugging

                    // Process Bluetooth commands
                    if(strcmp(command, "aan") == 0) {
                        digitalWrite(MOTOR_PIN, LOW);
                    } else if(strcmp(command, "uit") == 0) {
                        digitalWrite(MOTOR_PIN, HIGH);
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
                        int networkIndex = atoi(&command[8]) - 1;
                        if(networkIndex >= 0 && networkIndex < numNetworks) {
                            strncpy(ssid, networkSSIDs[networkIndex], sizeof(ssid) - 1);
                            ssid[sizeof(ssid) - 1] = '\0';
                            displayMessage("Selected:", ssid);
                            writeStringToEEPROM(ssidAddress, ssid);
                        } else {
                            displayMessage("Wifi:", "Invalid network");
                        }
                    } else if(strncmp(command, "password", 8) == 0) {
                        strncpy(pass, &command[9], sizeof(pass) - 1);
                        pass[sizeof(pass) - 1] = '\0';
                        bluetoothActive = false; // Temporarily deactivate Bluetooth
                        connectToNetwork();
                        writeStringToEEPROM(passwordAddress, pass);
                        bluetoothActive = true; // Reactivate Bluetooth after WiFi operations
                    } else if(strncmp(command, "api", 8) == 0) {
                        String jsonData = gatherSensorDataAsJson();
                        sendSensorDataToApi(jsonData);
                    } else {
                        Serial.println("Unknown command");
                    }
                }
            }
            displayMessage("Bluetooth:", "No connection");
            delay(1000);
        }
    } else {
        unsigned long wifiTimeout = millis() + 30000; // Timeout for WiFi operations
        scanNetworks();
        while (millis() < wifiTimeout) {
            // Wait for WiFi operations to complete
        }
    }
    
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
    checkTurbidity();
    readPH();

    if (WiFi.status() == WL_CONNECTED) {
        // Send sensor data at intervals
        if (currentMillis - lastApiCallTime >= apiCallInterval) {
            lastApiCallTime = currentMillis;  // Update the last API call time
            String jsonData = gatherSensorDataAsJson();
            sendSensorDataToApi(jsonData);
        }
    }

    int reading = digitalRead(BUTTON_PIN);
    if (reading == HIGH) {
        currentScreen++;
        if (currentScreen > 7) {
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

void checkBluetoothConnection() {
    if (!BLE.begin()) {
        displayMessage("FishSee", "Bluetooth failed");
        while (1);
    }

    BLE.setLocalName("FishSee");
    BLE.setAdvertisedService(commandService);
    commandService.addCharacteristic(commandCharacteristic);
    BLE.addService(commandService);
    BLE.advertise();
    displayMessage("FishSee", "Bluetooth active");
    delay(1500);
}

// EEPROM check
bool isEEPROMEmpty() {
  for (int i = 0; i < EEPROM.length(); i++) {
    if (EEPROM.read(i) != 255) {
      return false; // EEPROM is not empty
    }
  }
  return true; // EEPROM is empty
}

// Write string to EEPROM
void writeStringToEEPROM(int address, String data) {
  char charBuf[data.length() + 1];
  data.toCharArray(charBuf, sizeof(charBuf));
  for (int i = 0; i < data.length(); i++) {
    EEPROM.write(address + i, charBuf[i]);
  }
  EEPROM.write(address + data.length(), '\0'); // Null-terminate the string
}

// Read string from EEPROM
void readStringFromEEPROM(int address, char* buffer, int bufferSize) {
  int i = 0;
  char ch = EEPROM.read(address + i);
  while (ch != '\0' && i < bufferSize - 1) {
    buffer[i] = ch;
    ch = EEPROM.read(address + ++i);
  }
  buffer[i] = '\0'; // Null-terminate the string
}

// Gather sensor data and format it as a JSON string
String gatherSensorDataAsJson() {
    DynamicJsonDocument doc(512);
    doc["tempC"] = tempC;
    doc["distance_cm"] = distance_cm;
    doc["light_level"] = light_level;
    doc["water_level"] = water_level;
    doc["flow_rate"] = flow_frequency;
    doc["phValue"] = phValueCurrent;
    doc["turbidity"] = turbidity;
    String jsonData;
    serializeJson(doc, jsonData);
    Serial.println(jsonData);
    return jsonData;
}

// Send sensor data to API
void sendSensorDataToApi(String sensorData) {
    WiFiSSLClient wifiSSLClient;
    HttpClient client = HttpClient(wifiSSLClient, "fishsee.aeternaserver.net", 443);    

    String contentType = "application/json";

    client.beginRequest();
    client.post("/api/data-send", contentType, sensorData);
    client.endRequest();

    int statusCode = client.responseStatusCode();
    String response;
    if (statusCode >= 0) {
        client.skipResponseHeaders();
        while (client.available()) {
            char c = client.read();
            response += c;
        }
    } else {
        response = "Error receiving response";
    }
    displayMessage("Connecting", "to server...");
    delay(1500);
    displayMessage("JSON:", "Sent");
    Serial.println("Response:");
    Serial.println(response);

    if (statusCode == -3) {
        displayMessage("Server:", "Unavailable");
        delay(1000);
        displayMessage("Server:", "Check connection");
        delay(1000);
        if (WiFi.status() == WL_CONNECTED) {
          displayMessage("Wifi:", String(WiFi.SSID()));
          delay(1000);
          displayMessage("Wifi strength:", String(WiFi.RSSI()));
        } else {
          displayMessage("Wifi:", "Not connected");
        }
        delay(1000);
    }
}

// Display messages on the LCD
void displayMessage(String line1, String line2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
}

// Scan for available networks
void scanNetworks() {
    displayMessage("Wifi:", "Scanning...");
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
            networkSSIDs[i][sizeof(networkSSIDs[i]) - 1] = '\0';
            networkEncTypes[i] = WiFi.encryptionType(i);
        }
    }   
    displayMessage("Wifi:", "Found");
}

// Handle network selection
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
            ssid[sizeof(ssid) - 1] = '\0'; 

            displayMessage("Enter Password:", "");
            while (!commandCharacteristic.written()) {
                // Wait for password input
            }
            if (commandCharacteristic.written()) {
                strncpy(pass, (char*)commandCharacteristic.value(), sizeof(pass) - 1);
                pass[sizeof(pass) - 1] = '\0';
                connectToNetwork();
            }
        }
    }
}

// Connect to a network
void connectToNetwork() {
    displayMessage("Connecting to:", ssid);
    Serial.print("Connecting..");
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

// Print WiFi status
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

// Flow sensor interrupt handler
void flow() {
    flow_frequency++;
}

// Check turbidity
void checkTurbidity() {
    const int minAnalogValue = 0;
    const int maxAnalogValue = 640;

    int sensorValue = analogRead(TURBIDITY_PIN);
    turbidity = map(sensorValue, minAnalogValue, maxAnalogValue, 100, 0);
    turbidity = constrain(turbidity, 0, 100);
    turbidity_status = turbidity;
}

// Change the color of LED strips
void colorWipe(uint32_t color) {
    for (int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, color);
    }
    strip.show();
}

// Read pH value
void readPH(){
  for(int i=0;i<10;i++) { 
    buf[i]=analogRead(PH_SENSOR_PIN);
    delay(10);
  }
  for(int i=0;i<9;i++)  {
    for(int j=i+1;j<10;j++) {
      if(buf[i]>buf[j]) {
        temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
  avgValue=0;
  for(int i=2;i<8;i++)    
    avgValue+=buf[i];
  float phValue=(float)avgValue*5.0/1024/6; 
  phValue=3.5*phValue+0.54;   
  phValueCurrent= phValue;      
}

// Read values from the high section
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

// Read values from the low section
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

// Determine the water level
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

// Control the servo
void servo(int pulse) {
    for (int i = 0; i < 8; i++) {
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(pulse);
        digitalWrite(SERVO_PIN, LOW);
    }
}

// Update the current screen
void updateCurrentScreen() {
    lcd.clear();

    switch (currentScreen) {
        case 0:
            displayMessage("Water Level:", String(water_level) + " %");
            break;
        case 1:
            displayMessage("Distance:", String(distance_cm) + " cm");
            break;
        case 2:
            displayMessage("Light Level:", String(light_level));
            break;
        case 3:
            displayMessage("Temperature:", String(tempC) + " C");
            break;
        case 4:
            displayMessage("Turbidity:", String(turbidity) + " %");
            break;
        case 5:
            displayMessage("PH:", String(phValueCurrent));
            break;
        case 6:
            displayMessage("Flow sensor:", String(flow_frequency) + " hz");
            break;
        case 7: 
            if (WiFi.status() == WL_CONNECTED) {
                displayMessage("Wifi:", String(WiFi.SSID()));
            } else {
                displayMessage("Wifi:", "Not connected");
            }
    }
}