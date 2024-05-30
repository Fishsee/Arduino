#include "WiFiManager.h"
#include "DisplayManager.h"
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

void setupWiFi() {
    if (isEEPROMEmpty()) {
        displayMessage("Wifi:", "Not Set");
    } else {
        readStringFromEEPROM(ssidAddress, ssid, sizeof(ssid));
        readStringFromEEPROM(passwordAddress, pass, sizeof(pass));
        connectToNetwork();
    }
}

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
        for (int i = 0; i < numNetworks && i < 20; ++i) {
            strncpy(networkSSIDs[i], WiFi.SSID(i), sizeof(networkSSIDs[i]) - 1);
            networkSSIDs[i][sizeof(networkSSIDs[i]) - 1] = '\0';
            networkEncTypes[i] = WiFi.encryptionType(i);
        }
    }
    displayMessage("Wifi:", "Found");
}

void handleNetworkSelection() {
    displayMessage("Select Network", "");
    for (int i = 0; i < numNetworks && i < 20; ++i) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(networkSSIDs[i]);
        Serial.print(" (");
        Serial.print(WiFi.RSSI(i));
        Serial.println(")");
    }
    // Add your code here to handle the network selection using Bluetooth characteristic
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
        }
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
