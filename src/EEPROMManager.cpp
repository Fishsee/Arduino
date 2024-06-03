#include "EEPROMManager.h"
#include <EEPROM.h>

void setupEEPROM() {
    EEPROM.begin(); 
}

bool isEEPROMEmpty() {
    for (int i = 0; i < EEPROM.length(); i++) {
        if (EEPROM.read(i) != 255) {
            return false; 
        }
    }
    return true; 
}

void writeStringToEEPROM(int address, String data) {
    char charBuf[data.length() + 1];
    data.toCharArray(charBuf, sizeof(charBuf));
    for (int i = 0; i < data.length(); i++) {
        EEPROM.write(address + i, charBuf[i]);
    }
    EEPROM.write(address + data.length(), '\0'); 
#if defined(ESP8266) || defined(ESP32)
    EEPROM.commit(); 
#endif
}

void readStringFromEEPROM(int address, char* buffer, int bufferSize) {
    int i = 0;
    char ch = EEPROM.read(address + i);
    while (ch != '\0' && i < bufferSize - 1) {
        buffer[i] = ch;
        ch = EEPROM.read(address + ++i);
    }
    buffer[i] = '\0'; 
}
