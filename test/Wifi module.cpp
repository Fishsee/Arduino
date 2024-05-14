#include <SPI.h>
#include <WiFiNINA.h>
#include <Arduino.h>

char ssid[] = "WIFISSID";     // SSID van je WiFi-netwerk
char pass[] = "WIFIWW"; // Wachtwoord van je WiFi-netwerk

int status = WL_IDLE_STATUS; // Initaliseer de WiFi-status

void printWifiStatus();

void setup() {
  Serial.begin(9600); // Initialiseer de seriële communicatie
  
  // Maak verbinding met het WiFi-netwerk
  while (status != WL_CONNECTED) {
    Serial.print("Verbinding maken met WiFi-netwerk...");
    status = WiFi.begin(ssid, pass);

    // Wacht 10 seconden op verbinding
    delay(10000);
  }

  // Print het IP-adres van de Arduino in de seriële monitor
  Serial.println("Verbonden met WiFi-netwerk!");
  printWiFiStatus();
}

void loop() {
  // Voeg hier je code toe om acties uit te voeren terwijl de Arduino verbonden is met het internet
}

void printWiFiStatus() {
  // Print het IP-adres van de Arduino
  IPAddress ip = WiFi.localIP();
  Serial.print("IP-adres: ");
  Serial.println(ip);
}
