#ifndef EEPROMMANAGER_H
#define EEPROMMANAGER_H

#include <Arduino.h>

void setupEEPROM();
bool isEEPROMEmpty();
void writeStringToEEPROM(int address, String data);
void readStringFromEEPROM(int address, char* buffer, int bufferSize);

#endif
