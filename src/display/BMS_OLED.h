#pragma once

void setupOLED();
void updateOLEDStatus(char *statusText);
void updateOLEDError(char *errorText);
void oledPrint(const char *newStatusText);
void oledPrintln(const char *newStatusText);