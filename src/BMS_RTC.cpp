#include "BMS_RTC.h"
#include <TimeLib.h>
// #include <DS1307RTC.h> // a basic DS1307 library that returns time as a time_t
#include <Arduino.h> // Required for Serial and other core Arduino functions

time_t RTCTime;

/**
 * @brief Initializes the RTC and syncs the Time library.
 * It also prints the current time to the serial monitor for debugging.
 */
void RTCSetup()
{
    // Sync the Time library with the Teensy's built-in RTC
    setSyncProvider(getTeensy3Time);

    delay(100);

    // Print the time to the serial monitor for confirmation
    Serial.print("Current time: ");
    Serial.println(getFormattedTime());
}

/**
 * @brief Gets the current time and date as a formatted string.
 * @return A String object containing the time and date in the
 * "dd/mm/yy-HH:MM:SS" format.
 */
String getFormattedTime(time_t t)
{
    return String(day(t), DEC) + "/" +
           String(month(t), DEC) + "/" +
           String(year(t) % 100, DEC) + "-" + // Get last two digits of the year
           String(hour(t), DEC) + ":" +
           String(minute(t), DEC) + ":" +
           String(second(t), DEC);
}

time_t getTeensy3Time()
{
    return Teensy3Clock.get();
}
