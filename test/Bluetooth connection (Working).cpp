#include <Arduino.h>
#include "WiFiS3.h"
#include <ArduinoBLE.h>

// Define a unique UUID for the custom service and characteristic
#define SERVICE_UUID        "12345678-1234-5678-1234-56789abcdef0"
#define CHARACTERISTIC_UUID "12345678-1234-5678-1234-56789abcdef1"

BLEService commandService(SERVICE_UUID); // Create a BLE service

// Create a BLE characteristic for commands
BLECharacteristic commandCharacteristic(CHARACTERISTIC_UUID, BLERead | BLEWrite, 20);

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  // Set the local name of the BLE device
  BLE.setLocalName("ArduinoBLE");
  BLE.setAdvertisedService(commandService);

  // Add the characteristic to the service
  commandService.addCharacteristic(commandCharacteristic);

  // Add the service
  BLE.addService(commandService);

  // Start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop() {
  // Listen for BLE peripherals to connect
  BLEDevice central = BLE.central();

  // If a central is connected to the peripheral
  if (central) {
    Serial.print("Connected to central: ");
    // Print the central's MAC address
    Serial.println(central.address());

    // While the central is still connected to the peripheral
    while (central.connected()) {
      if (commandCharacteristic.written()) {
        // Get the value of the characteristic
        int len = commandCharacteristic.valueLength();
        char command[len + 1];
        memcpy(command, commandCharacteristic.value(), len);
        command[len] = '\0'; // Null-terminate the string

        Serial.print("Command received: ");
        Serial.println(command);

        // Handle different commands
        if (strcmp(command, "ON") == 0) {
          Serial.println("Turning ON the device...");
          // Add your code to turn ON the device
        } else if (strcmp(command, "OFF") == 0) {
          Serial.println("Turning OFF the device...");
          // Add your code to turn OFF the device
        } else {
          Serial.println("Unknown command");
        }
      }
    }

    // When the central disconnects, print it out:
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}
