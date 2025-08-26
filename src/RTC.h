#pragma once

#include <TimeLib.h> // Include the TimeLib library for time functions
#include <Arduino.h>

void RTCSetup();
String getFormattedTime(time_t t = now());
