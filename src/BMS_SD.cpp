#include <SD.h>
#include "Logger.h"
#include "globals.h"

#define LOG_FILE "BMS_log.txt"
// #define ERROR_FILE "BMS_ERROR_log.txt"


void SDInit() {
  if (!SD.begin(BUILTIN_SDCARD)) {
    Logger::error("SD card initialisation failed");
    sdStatus = SD_ERROR;
  }else{
      Logger::info("SD card initialised");
      sdStatus = SD_OK;
  }
}


void logtoSD(String dataString) {
  File dataFile = SD.open(LOG_FILE, FILE_WRITE);
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
  } else {
    Logger::error("error opening log.txt");
  }
}
