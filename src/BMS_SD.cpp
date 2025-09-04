/**
 * @file BMS_SD.cpp
 * @brief Setup and Read/Write functions for Teensy 4.1 Build In SD Card
 *
 */

#include <SD.h>
#include "BMS_SD.h"
#include "Logger.h"
#include "globals.h"

#define LOG_FILE "BMS_log.txt"
// #define ERROR_FILE "BMS_ERROR_log.txt"

void SDInit()
{
  if (!SD.begin(BUILTIN_SDCARD))
  {
    Logger::error("SD card initialisation failed");
    sdStatus = SD_ERROR;
  }
  else
  {
    Logger::info("SD card initialised");
    sdStatus = SD_OK;
    if (!SD.exists(LOG_FILE))
    {
      Logger::info("BMS_log.txt not found, creating new file");
      File dataFile = SD.open(LOG_FILE, FILE_WRITE);
      dataFile.close();
    }
  }
}
void logtoSD(const char *dataString)
{
  File dataFile = SD.open(LOG_FILE, FILE_WRITE);
  if (dataFile)
  {
    dataFile.println(dataString);
    dataFile.close();
  }
  else
  {
    Logger::error("Error opening %s", LOG_FILE);
  }
}
