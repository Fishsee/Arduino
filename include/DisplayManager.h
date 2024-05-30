#ifndef DISPLAYMANAGER_H
#define DISPLAYMANAGER_H

#include <rgb_lcd.h>

void setupLCD();
void displayMessage(String line1, String line2);
void updateDisplay();
void updateCurrentScreen();

#endif
