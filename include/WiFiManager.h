#ifndef WIFIMANAGER_H
#define WIFIMANAGER_H

#include <WiFi.h>

extern char ssid[32];
extern char pass[64];
extern char networkSSIDs[20][32];
extern int networkEncTypes[20];
extern int numNetworks;
extern int ssidAddress;
extern int passwordAddress;

void setupWiFi();
void connectToNetwork();
void scanNetworks();
void handleNetworkSelection();
void printWiFiStatus();
void handleWiFi();
void sendSensorDataToApi(String sensorData); // Ensure this is declared

#endif
