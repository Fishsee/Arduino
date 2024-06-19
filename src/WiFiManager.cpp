#include "WiFiManager.h"
#include "EEPROMManager.h"
#include "SensorManager.h"
#include <ArduinoHttpClient.h>

char ssid[32];
char pass[64];
char networkSSIDs[20][32];
int networkEncTypes[20];
int numNetworks = 0;
int ssidAddress = 0;
int passwordAddress = 32;

unsigned long lastApiCallTime = 0;
const unsigned long apiCallInterval = 30000; // Interval to send sensor data (e.g., every 60 seconds)
const unsigned long brightnessApiCallInterval = 30000; // Interval to fetch brightness (e.g., every 60 seconds)
unsigned long lastBrightnessApiCallTime = 0;

void setupWiFi() {
    if (isEEPROMEmpty()) {
        Serial.println("Wifi: Not Set");
    } else {
        readStringFromEEPROM(ssidAddress, ssid, sizeof(ssid));
        readStringFromEEPROM(passwordAddress, pass, sizeof(pass));
        connectToNetwork();
    }
}

void connectToNetwork() {
    Serial.print("Connecting to: ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startTime < 10000)) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Connected to: ");
        Serial.println(ssid);
        printWiFiStatus();
    } else {
        Serial.println("Failed to connect");
    }
}

void scanNetworks() {
    Serial.println("Wifi: Scanning...");
    if (WiFi.status() == WL_NO_SHIELD) {
        Serial.println("Wifi: No module");
        while (true);
    }

    numNetworks = WiFi.scanNetworks();
    if (numNetworks == 0) {
        Serial.println("Wifi: No Networks");
    } else {
        for (int i = 0; i < numNetworks && i < 20; ++i) {
            strncpy(networkSSIDs[i], WiFi.SSID(i), sizeof(networkSSIDs[i]) - 1);
            networkSSIDs[i][sizeof(networkSSIDs[i]) - 1] = '\0';
            networkEncTypes[i] = WiFi.encryptionType(i);
        }
    }
    Serial.println("Wifi: Found");
}

void handleNetworkSelection() {
    Serial.println("Select Network:");
    for (int i = 0; i < numNetworks && i < 20; ++i) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(networkSSIDs[i]);
        Serial.print(" (");
        Serial.print(WiFi.RSSI(i));
        Serial.println(")");
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

void handleWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        unsigned long currentMillis = millis();
        if (currentMillis - lastApiCallTime >= apiCallInterval) {
            lastApiCallTime = currentMillis;  // Update the last API call time
            String jsonData = gatherSensorDataAsJson();
            sendSensorDataToApi(jsonData);
            handleBrightnessControl();
        }
    }
    else {
        Serial.println("WiFi Disconnected. Attempting reconnection...");
        setupWiFi();  // Attempt to reconnect
    }
}


void handleBrightnessControl() {
    if (WiFi.status() == WL_CONNECTED) {
        unsigned long currentMillis = millis();
        if (currentMillis - lastBrightnessApiCallTime >= brightnessApiCallInterval) {
            lastBrightnessApiCallTime = currentMillis;
            int brightness = getBrightnessFromApi();
            setBrightness(brightness);
            Serial.println("brightness:");
            Serial.print(brightness);
        }
    }
    else{
        Serial.println("not connected");
    }
}

int getBrightnessFromApi() {
    WiFiSSLClient wifiSSLClient;
    HttpClient client = HttpClient(wifiSSLClient, "fishsee.aeternaserver.net", 443);
    String token = "99|ab6lI1aunVoptNtDob8he1KuQmw6II6TFscpu34y6b1b952e";

    client.beginRequest();
    client.get("/api/brightness");
    client.sendHeader("Authorization", "Bearer " + token);
    client.endRequest();

    int statusCode = client.responseStatusCode();
    if (statusCode == 200) {
        String response = client.responseBody();
        Serial.println("Brightness from API: " + response);
        return response.toInt();
    } else {
        Serial.println("Failed to fetch brightness, Status code: " + String(statusCode));
        return 0;
    }
}


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
    Serial.println("Connecting to server...");
    Serial.println("JSON: Sent");
    Serial.println("Response:");
    Serial.println(response);

    if (statusCode == -3) {
        Serial.println("Server: Unavailable");
        Serial.println("Server: Check connection");
        if (WiFi.status() == WL_CONNECTED) {
            Serial.print("Wifi: ");
            Serial.println(String(WiFi.SSID()));
            Serial.println("Wifi strength: ");
            Serial.println(String(WiFi.RSSI()));
        } else {
            Serial.println("Wifi: Not connected");
        
        }
    }
}
