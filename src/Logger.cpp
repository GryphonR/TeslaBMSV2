/*
   Logger.cpp

  Copyright (c) 2013 Collin Kidder, Michael Neuweiler, Charles Galpin

  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the
  "Software"), to deal in the Software without restriction, including
  without limitation the rights to use, copy, modify, merge, publish,
  distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so, subject to
  the following conditions:

  The above copyright notice and this permission notice shall be included
  in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#include "Logger.h"
#include "globals.h"
#include "BMS_RTC.h"
#include "BMS_SD.h"

Logger::LogLevel Logger::logLevel = Logger::Info;
Logger::LogLevel Logger::serialLogLevel = Logger::Info;
Logger::LogLevel Logger::sdLogLevel = Logger::Info;
Logger::LogLevel Logger::oledLogLevel = Logger::Info;
uint32_t Logger::lastLogTime = 0;

char logBuffer[256];

unsigned int bufferIndex = 0;

/*
   Output a debug message with a variable amount of parameters.
   printf() style, see Logger::log()

*/
void Logger::debug(const char *message, ...)
{
  if (logLevel > Debug)
    return;
  va_list args;
  va_start(args, message);
  Logger::log(Debug, message, args);
  va_end(args);
}

/*
   Output a info message with a variable amount of parameters
   printf() style, see Logger::log()
*/
void Logger::info(const char *message, ...)
{
  if (logLevel > Info)
    return;
  va_list args;
  va_start(args, message);
  Logger::log(Info, message, args);
  va_end(args);
}

/*
   Output a warning message with a variable amount of parameters
   printf() style, see Logger::log()
*/
void Logger::warn(const char *message, ...)
{
  if (logLevel > Warn)
    return;
  va_list args;
  va_start(args, message);
  Logger::log(Warn, message, args);
  va_end(args);
}

/*
   Output a error message with a variable amount of parameters
   printf() style, see Logger::log()
*/
void Logger::error(const char *message, ...)
{
  if (logLevel > Error)
    return;
  va_list args;
  va_start(args, message);
  Logger::log(Error, message, args);
  va_end(args);
}

/*
   Output a comnsole message with a variable amount of parameters
   printf() style, see Logger::logMessage()
*/
void Logger::console(const char *message, ...)
{
  va_list args;
  va_start(args, message);
  Logger::logMessage(message, args);
  va_end(args);
}


void Logger::setSerialLoglevel(LogLevel level)
{
  serialLogLevel = level;
  logLevel = getLogLevel(); 
}
void Logger::setOledLoglevel(LogLevel level)
{
  oledLogLevel = level;
  logLevel = getLogLevel();
}
void Logger::setSdLoglevel(LogLevel level)
{
  sdLogLevel = level;
  logLevel = getLogLevel();
}

/*
   Retrieve the current log level.
*/
Logger::LogLevel Logger::getSerialLogLevel()
{
  return serialLogLevel;
}
Logger::LogLevel Logger::getSdLogLevel()
{
  return sdLogLevel;
}
Logger::LogLevel Logger::getOledLogLevel()
{
  return oledLogLevel;
}

/**
 * @brief Returns the lowest log level from the serial, SD, and OLED log levels.
 *
 * This is useful for determining the overall log level of the system.
 *
 * @return The lowest log level from serialLogLevel, sdLogLevel, and oledLogLevel
 */

Logger::LogLevel Logger::getLogLevel(){
  return min(min(serialLogLevel, sdLogLevel), oledLogLevel);
}

/*
   Return a timestamp when the last log entry was made.
*/
uint32_t Logger::getLastLogTime()
{
  return lastLogTime;
}

/*
   Returns if debug log level is enabled. This can be used in time critical
   situations to prevent unnecessary string concatenation (if the message won't
   be logged in the end).

   Example:
   if (Logger::isDebug()) {
      Logger::debug("current time: %d", millis());
   }
*/
boolean Logger::isDebug()
{
  return logLevel == Debug;
}

/*
   Output a log message (called by debug(), info(), warn(), error(), console())

   Supports printf() like syntax:

   %% - outputs a '%' character
   %s - prints the next parameter as string
   %d - prints the next parameter as decimal
   %f - prints the next parameter as double float
   %x - prints the next parameter as hex value
   %X - prints the next parameter as hex value with '0x' added before
   %b - prints the next parameter as binary value
   %B - prints the next parameter as binary value with '0b' added before
   %l - prints the next parameter as long
   %c - prints the next parameter as a character
   %t - prints the next parameter as boolean ('T' or 'F')
   %T - prints the next parameter as boolean ('true' or 'false')
*/
void Logger::log(LogLevel level, const char *format, va_list args)
{

  lastLogTime = millis();
  // bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "%lu", lastLogTime);
  // bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, " - ");

  bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "%s::", getFormattedTime().c_str());
  bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "UT:%s::", millisToText(lastLogTime));

  switch (level)
  {
  case Off:
    break;
  case Debug:
    bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "DEBUG");
    break;
  case Info:
    bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "INFO");
    break;
  case Warn:
    bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "WARNING");
    break;
  case Error:
    bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "ERROR");
    break;
  }
  bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, ": ");

  Serial.println("Logbuffer test 1:");
  Serial.println(logBuffer);

  logMessage(format, args);

  printLogs(level); // Outputs the buffer to each relevant device
}

/**
 * @brief Returns a char buffer showing days, hours, minutes and seconds from the given milliseconds.
 *
 * @param milliseconds The time in milliseconds
 * @return A char buffer showing days, hours, minutes and seconds as "xD:xH:xM:xS"
 */
char *Logger::millisToText(unsigned long milliseconds)
{
  static char buffer[25];

  unsigned long days = milliseconds / (24 * 60 * 60 * 1000);
  milliseconds %= (24 * 60 * 60 * 1000);
  unsigned long hours = milliseconds / (60 * 60 * 1000);
  milliseconds %= (60 * 60 * 1000);
  unsigned long minutes = milliseconds / (60 * 1000);
  milliseconds %= (60 * 1000);
  unsigned long seconds = milliseconds / 1000;
  if(days > 0) {
    snprintf(buffer, sizeof(buffer), "%luD:%luH:%luM:%luS", days, hours, minutes, seconds);
  } else if(hours > 0) {
    snprintf(buffer, sizeof(buffer), "%luH:%luM:%luS", hours, minutes, seconds);
  } else if(minutes > 0) {
    snprintf(buffer, sizeof(buffer), "%luM:%luS", minutes, seconds);
  } else {
    snprintf(buffer, sizeof(buffer), "%luS", seconds);
  }

  return buffer;
}

/*
   Output a log message (called by log(), console())

   Supports printf() like syntax:

   %% - outputs a '%' character
   %s - prints the next parameter as string
   %d - prints the next parameter as decimal
   %f - prints the next parameter as double float
   %x - prints the next parameter as hex value
   %X - prints the next parameter as hex value with '0x' added before
   %b - prints the next parameter as binary value
   %B - prints the next parameter as binary value with '0b' added before
   %l - prints the next parameter as long
   %c - prints the next parameter as a character
   %t - prints the next parameter as boolean ('T' or 'F')
   %T - prints the next parameter as boolean ('true' or 'false')
*/
void Logger::logMessage(const char *format, va_list args)
{
  for (; *format != 0; ++format)
  {
    if (*format == '%')
    {
      ++format;
      if (*format == '\0')
        break;
      if (*format == '%')
      {
        bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "%c", *format);
        continue;
      }
      if (*format == 's')
      {
        char *s = (char *)va_arg(args, int);
        bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, s);
        continue;
      }
      if (*format == 'd' || *format == 'i')
      {
        bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "%d", va_arg(args, int));
        continue;
      }
      if (*format == 'f')
      {
        bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "%03f", va_arg(args, double));
        continue;
      }
      if (*format == 'z')
      {
        bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "%f", va_arg(args, double));
        continue;
      }
      if (*format == 'x')
      {
        bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "%#x", va_arg(args, int));
        continue;
      }
      if (*format == 'X')
      {
        bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "0x");
        bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "%#x", va_arg(args, int));
        continue;
      }
      if (*format == 'b')
      {
        bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "%#x", va_arg(args, int));
        continue;
      }
      if (*format == 'B')
      {
        bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "0x");
        bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "%#x", va_arg(args, int));
        continue;
      }
      if (*format == 'l')
      {
        bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "%ld", va_arg(args, long));
        continue;
      }

      if (*format == 'c')
      {
        bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "%d", va_arg(args, int));
        continue;
      }
      if (*format == 't')
      {
        if (va_arg(args, int) == 1)
        {
          bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "T");
        }
        else
        {
          bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "F");
        }
        continue;
      }
      if (*format == 'T')
      {
        if (va_arg(args, int) == 1)
        {
          bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "TRUE");
        }
        else
        {
          bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "FALSE");
        }
        continue;
      }
    }
    bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "%c", *format);
  }
  bufferIndex += snprintf(logBuffer + bufferIndex, sizeof(logBuffer) - bufferIndex, "\n\r");

  
}

void Logger::printLogs(LogLevel level)
{
  if(level >= serialLogLevel) {
    Serial.write(logBuffer, sizeof(logBuffer));
  }
  if (level >= sdLogLevel && sdStatus == SD_OK)
  {
    logtoSD(logBuffer);
  }
  if (level >= oledLogLevel && oledConnected)
  {
    oledPrint(logBuffer);
  }
  memset(logBuffer, 0, sizeof(logBuffer)); // Clear the buffer
  bufferIndex = 0;
}
