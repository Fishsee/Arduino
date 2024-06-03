#include "BluetoothManager.h"
#include "WiFiManager.h"
#include "SensorManager.h"
#include "EEPROMManager.h"

#define MOTOR_PIN 0 // Define MOTOR_PIN here or in a common header

#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1"

BLEService commandService(SERVICE_UUID);
BLECharacteristic commandCharacteristic(CHARACTERISTIC_UUID, BLERead | BLEWrite, 128);


void setupBluetooth() {
    if (!BLE.begin()) {
        Serial.println("FishSee: Bluetooth failed");
        while (1);
    }

    BLE.setLocalName("FishSee");
    BLE.setAdvertisedService(commandService);
    commandService.addCharacteristic(commandCharacteristic);
    BLE.addService(commandService);
    BLE.advertise();
    pinMode(MOTOR_PIN, OUTPUT);
    Serial.println("FishSee Bluetooth active");
}

void handleBluetooth() {
    BLEDevice central = BLE.central();
    if (central) {
        Serial.print("Bluetooth connected");
        Serial.println(central.address());
        while (central.connected()) {
            if (commandCharacteristic.written()) {
                int len = commandCharacteristic.valueLength();
                char command[len + 1];
                memcpy(command, commandCharacteristic.value(), len);
                command[len] = '\0';
                Serial.println(command);

                if (strcmp(command, "aan") == 0) {
                    digitalWrite(MOTOR_PIN, LOW);
                } else if (strcmp(command, "uit") == 0) {
                    digitalWrite(MOTOR_PIN, HIGH);
                } else if (strcmp(command, "scan") == 0) {
                    scanNetworks();
                } else if (strcmp(command, "list") == 0) {
                    handleNetworkSelection();
                } else if (strcmp(command, "status") == 0) {
                    printWiFiStatus();
                } else if (strncmp(command, "connect", 7) == 0) {
                    int networkIndex = atoi(&command[8]) - 1;
                    if (networkIndex >= 0 && networkIndex < numNetworks) {
                        strncpy(ssid, networkSSIDs[networkIndex], sizeof(ssid) - 1);
                        ssid[sizeof(ssid) - 1] = '\0';
                        Serial.print("Selected:");
                        Serial.println(ssid);
                        writeStringToEEPROM(ssidAddress, ssid);
                    } else {
                        Serial.println("Wifi: Invalid network");
                    }
                } else if (strncmp(command, "password", 8) == 0) {
                    strncpy(pass, &command[9], sizeof(pass) - 1);
                    pass[sizeof(pass) - 1] = '\0';
                    connectToNetwork();
                    writeStringToEEPROM(passwordAddress, pass);
                } else if (strncmp(command, "api", 8) == 0) {
                    String jsonData = gatherSensorDataAsJson();
                    sendSensorDataToApi(jsonData);
                } else {
                    Serial.println("Unknown command");
                }
            }
        }
        Serial.println("Bluetooth: No connection");
    }
}
