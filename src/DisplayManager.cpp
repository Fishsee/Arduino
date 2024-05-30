#include "DisplayManager.h"
#include "SensorManager.h"
#include "WiFiManager.h"
#include "Common.h"

extern Adafruit_NeoPixel strip;
extern int water_level;
extern int distance_cm;
extern int light_level;
extern float tempC;
extern int turbidity;
extern float phValueCurrent;
extern int flow_frequency;
extern void colorWipe(uint32_t color);

rgb_lcd lcd;
int currentScreen = 0;

void setupLCD() {
    lcd.begin(16, 2);
    lcd.setRGB(255, 255, 255);
    displayMessage("FishSee", "");
    delay(1000);
}

void displayMessage(String line1, String line2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
}

void updateDisplay() {
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
}

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
