<h1 align="center">Project Fishsee <small>Arduino</small></h1>

###

<p align="center">Project innovatie periode 4 - INF1-B</p>

###

<img align="right" height="150" src="https://i.imgflip.com/8iqaih.gif"  />

###

# Installatie en Configuratie Handleiding

## Introductie
Welkom bij de installatiegids van uw nieuwe systeem. Volg deze instructies zorgvuldig om het systeem correct te configureren en te installeren.

## Vereisten
- Visual Studio Code
- Arduino Uno R4 WiFi
- PlatformIO extensie geïnstalleerd in Visual Studio Code
- Een Raspberry Pi voor seriele verbindingen (nodig voor het oplossen van een bekende fout)

## Installatie
1. **Visual Studio Code en PlatformIO:**
   - Zorg ervoor dat Visual Studio Code is geïnstalleerd op uw computer.
   - Installeer de PlatformIO extensie via de Visual Studio Code Marketplace.

2. **Project Setup:**
   - Start een nieuw project.
   - Open de file locatie en trek de git binnen:

```
git clone https://github.com/Fishsee/Arduino.git .
```

## Configuratie
De configuratie van de hardwarepins is gedefinieerd in het `Config.h` bestand. Pas deze configuratie aan op basis van de specifieke hardware setup van uw systeem:
```cpp
#define MOTOR_PIN 8
#define SERVO_PIN 7
#define SERVO_PIN_2 5
#define TEMPERATURE_PIN 2
#define FLOW_PIN 3
#define PH_SENSOR_PIN A1
#define TURBIDITY_PIN A2
#define ULTRASONIC_PIN 8
#define LED_PIN 4
#define NUM_LEDS 60
```

## Libraries
De libraries staan in de PlatformIO.ini.
Het kan zijn dat deze niet direct binnenladen, dit kun je forceren door Visual Studio Code opnieuw op te starten.

## Known Errors
Memory Overflow bij Te Veel Server Requests:
Wanneer er meer dan twee keer het verwachte aantal server requests wordt verzonden, kan er een memory overflow optreden.
Deze fout is opgelost door gebruik te maken van een seriële verbinding met een Raspberry Pi die de datastroom beheert en de belasting op het hoofdsysteem vermindert.
Verdere Ondersteuning
Mocht u tijdens de installatie of configuratie vragen hebben, kunt u contact opnemen met onze technische ondersteuning.

Dank u voor het kiezen van ons product. Volg deze handleiding zorgvuldig voor een optimale ervaring met uw nieuwe systeem.

## Code conventie
De code convetie word gevolgt met; https://www.makerguides.com/c-style-guide-for-arduino-projects/
