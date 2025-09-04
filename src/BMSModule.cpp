/**
 * @file BMSModule.cpp
 * @brief Contains BMSModule class for managing individual Tesla Module controlers.
 * 
 */

#include "config.h"
#include "BMSModule.h"
#include "BMSUtil.h"
#include "Logger.h"

BMSModule::BMSModule()
{
  for (int i = 0; i < 6; i++)
  {
    cellVolt[i] = 0.0f;
    lowestCellVolt[i] = 5.0f;
    highestCellVolt[i] = 0.0f;
  }
  moduleVolt = 0.0f;
  retmoduleVolt = 0.0f;
  temperatures[0] = 0.0f;
  temperatures[1] = 0.0f;
  lowestTemperature = 200.0f;
  highestTemperature = -100.0f;
  lowestModuleVolt = 200.0f;
  highestModuleVolt = 0.0f;
  moduleAddress = 0;
}

/**
 * @brief Clears all module voltage and temperature data.
 *
 * Sets all cell voltage, module voltage, and temperature values to 0.0f.
 */
void BMSModule::clearModule()
{
  for (int i = 0; i < 6; i++)
  {
    cellVolt[i] = 0.0f;
  }
  moduleVolt = 0.0f;
  temperatures[0] = 0.0f;
  temperatures[1] = 0.0f;
}

/*
  Reading the status of the board to identify any flags, will be more useful when implementing a sleep cycle
*/

/**
 * @brief Reads the status of the board to identify any flags, will be more useful when implementing a sleep cycle
 *
 * Reads the Alert Status, Fault Status, COV Faults, CUV Faults, and Balance Time from the module and stores them in the
 * alerts, faults, COVFaults, CUVFaults, and balanceTime variables respectively.
 */
void BMSModule::readStatus()
{
  uint8_t payload[3];
  uint8_t buff[8];
  /*
    Serial.println();
    Serial.print(moduleAddress);
    Serial.print(" - ");
  */
  payload[0] = moduleAddress << 1; //adresss
  payload[1] = REG_DEV_STATUS;//Alert Status start
  payload[2] = 0x01;
  BMSUtil::sendDataWithReply(payload, 3, false, buff, 4);
  /*
    for (int z = 0; z < 4; z++)
    {
    Serial.print(buff[z], HEX);
    Serial.print("-");
    }
  */

  payload[0] = moduleAddress << 1; //adresss
  payload[1] = REG_ALERT_STATUS;//Alert Status start
  payload[2] = 0x04;
  BMSUtil::sendDataWithReply(payload, 3, false, buff, 7);
  alerts = buff[3];
  faults = buff[4];
  COVFaults = buff[5];
  CUVFaults = buff[6];
  payload[0] = moduleAddress << 1; //adresss
  payload[1] = REG_BAL_TIME;//Alert Status start
  payload[2] = 0x01;
  BMSUtil::sendDataWithReply(payload, 3, false, buff, 4);
  /*
    Serial.print(" | ");
    for (int z = 0; z < 4; z++)
    {
    Serial.print(buff[z], HEX);
    Serial.print("-");
    }
  */
}

/**
 * @brief Gets the faults from the BMS module.
 *
 * @return The faults from the BMS module.
 */
uint8_t BMSModule::getFaults()
{
  return faults;
}

/**
 * @brief Gets the alerts from the BMS module.
 *
 * @return The alerts from the BMS module.
 */
uint8_t BMSModule::getAlerts()
{
  return alerts;
}

/**
 * @brief Gets the cells which have exceeded the overvoltage threshold.
 *
 * @return The cells which have exceeded the overvoltage threshold.
 */
uint8_t BMSModule::getCOVCells()
{
  return COVFaults;
}

/**
 * @brief Gets the cells which have exceeded the undervoltage threshold.
 *
 * @return The cells which have exceeded the undervoltage threshold.
 */
uint8_t BMSModule::getCUVCells()
{
  return CUVFaults;
}

/*
  Reading the setpoints, after a reset the default tesla setpoints are loaded
  Default response : 0x10, 0x80, 0x31, 0x81, 0x08, 0x81, 0x66, 0xff
*/
/*
  void BMSModule::readSetpoint()
  {
  uint8_t payload[3];
  uint8_t buff[12];
  payload[0] = moduleAddress << 1; //adresss
  payload[1] = 0x40;//Alert Status start
  payload[2] = 0x08;//two registers
  sendData(payload, 3, false);
  delay(2);
  getReply(buff);

  OVolt = 2.0+ (0.05* buff[5]);
  UVolt = 0.7 + (0.1* buff[7]);
  Tset = 35 + (5 * (buff[9] >> 4));
  } */

/**
 * @brief Stop balancing on this module.
 *
 * This function sends a command to the module to stop balancing. It does
 * not return any data.
 */
void BMSModule::stopBalance()
{
  uint8_t buff[8];
  uint8_t payload[4];
  payload[0] = moduleAddress << 1;
  payload[1] = REG_BAL_CTRL;
  payload[2] = 0; //write balance state to register
  BMSUtil::sendDataWithReply(payload, 3, true, buff, 4);
  delay(2);
}

/**
 * @brief Reads the voltage and temperature values from the module.
 *
 * This function reads the voltage and temperature values from the module and stores them in the appropriate variables.
 * It also updates the highest and lowest values for the module voltage and each cell voltage.
 *
 * @return true if the data was read successfully, false otherwise
 */
bool BMSModule::readModuleValues()
{
  uint8_t payload[4];
  uint8_t buff[50];
  uint8_t calcCRC;
  bool retVal = false;
  int retLen;
  float tempCalc;
  float tempTemp;


  payload[0] = moduleAddress << 1;
  delay(2);

  payload[0] = moduleAddress << 1;

  readStatus();
  Logger::debug("Module %i   alerts=%X   faults=%X   COV=%X   CUV=%X", moduleAddress, alerts, faults, COVFaults, CUVFaults);

  payload[1] = REG_ADC_CTRL;
  payload[2] = 0b00111101; //ADC Auto mode, read every ADC input we can (Both Temps, Pack, 6 cells)
  BMSUtil::sendDataWithReply(payload, 3, true, buff, 3);

  payload[1] = REG_IO_CTRL;
  payload[2] = 0b00000011; //enable temperature measurement VSS pins
  BMSUtil::sendDataWithReply(payload, 3, true, buff, 3);

  payload[1] = REG_ADC_CONV; //start all ADC conversions
  payload[2] = 1;
  BMSUtil::sendDataWithReply(payload, 3, true, buff, 3);

  payload[1] = REG_GPAI; //start reading registers at the module voltage registers
  payload[2] = 0x12; //read 18 bytes (Each value takes 2 - ModuleV, CellV1-6, Temp1, Temp2)
  retLen = BMSUtil::sendDataWithReply(payload, 3, false, buff, 22);

  calcCRC = BMSUtil::genCRC(buff, retLen - 1);
  Logger::debug("Sent CRC: %x     Calculated CRC: %x", buff[21], calcCRC);

  //18 data bytes, address, command, length, and CRC = 22 bytes returned
  //Also validate CRC to ensure we didn't get garbage data.
  if ( (retLen == 22) && (buff[21] == calcCRC) )
  {
    if (buff[0] == (moduleAddress << 1) && buff[1] == REG_GPAI && buff[2] == 0x12) //Also ensure this is actually the reply to our intended query
    {
      //payload is 2 bytes gpai, 2 bytes for each of 6 cell voltages, 2 bytes for each of two temperatures (18 bytes of data)
      retmoduleVolt = (buff[3] * 256 + buff[4]) * 0.0020346293922562f;///0.002034609f;
      if (retmoduleVolt > highestModuleVolt) highestModuleVolt = retmoduleVolt;
      if (retmoduleVolt < lowestModuleVolt) lowestModuleVolt = retmoduleVolt;
      for (int i = 0; i < 6; i++)
      {
        cellVolt[i] = (buff[5 + (i * 2)] * 256 + buff[6 + (i * 2)]) * 0.000381493f;
        if (lowestCellVolt[i] > cellVolt[i] && cellVolt[i] >= IgnoreCell) lowestCellVolt[i] = cellVolt[i];
        if (highestCellVolt[i] < cellVolt[i]) highestCellVolt[i] = cellVolt[i];
      }

      ////use added up cells and not reported module voltage////////
      moduleVolt = 0;
      for (int i = 0; i < 6; i++)
      {
        moduleVolt = moduleVolt + cellVolt[i];
      }

      //Now using steinhart/hart equation for temperatures. We'll see if it is better than old code.
      tempTemp = (1.78f / ((buff[17] * 256 + buff[18] + 2) / 33046.0f) - 3.57f);
      tempTemp *= 1000.0f;
      tempCalc =  1.0f / (0.0007610373573f + (0.0002728524832 * logf(tempTemp)) + (powf(logf(tempTemp), 3) * 0.0000001022822735f));

      temperatures[0] = tempCalc - 273.15f;

      tempTemp = 1.78f / ((buff[19] * 256 + buff[20] + 9) / 33068.0f) - 3.57f;
      tempTemp *= 1000.0f;
      tempCalc = 1.0f / (0.0007610373573f + (0.0002728524832 * logf(tempTemp)) + (powf(logf(tempTemp), 3) * 0.0000001022822735f));
      temperatures[1] = tempCalc - 273.15f;

      if (getLowTemp() < lowestTemperature) lowestTemperature = getLowTemp();
      if (getHighTemp() > highestTemperature) highestTemperature = getHighTemp();

      Logger::debug("Got voltage and temperature readings");
      retVal = true;
    }
  }
  else
  {
    Logger::error("Invalid module response received for module %i  len: %i   crc: %i   calc: %i",
                  moduleAddress, retLen, buff[21], calcCRC);
  }

  //turning the temperature wires off here seems to cause weird temperature glitches
  // payload[1] = REG_IO_CTRL;
  // payload[2] = 0b00000000; //turn off temperature measurement pins
  // BMSUtil::sendData(payload, 3, true);
  // delay(3);
  // BMSUtil::getReply(buff, 50);    //TODO: we're not validating the reply here. Perhaps check to see if a valid reply came back

  return retVal;
}

/**
 * @brief Retrieves the voltage of a specific cell in the module
 *
 * @param cell cell number, valid values are 0-5
 * @return The voltage of the specified cell in Volts
 */
float BMSModule::getCellVoltage(int cell)
{
  if (cell < 0 || cell > 5) return 0.0f;
  return cellVolt[cell];
}

/**
 * @brief Gets the lowest cell voltage of the module
 *
 * @return The lowest cell voltage in Volts, ignoring any cell voltage that is less than IgnoreCell
 */
float BMSModule::getLowCellV()
{
  float lowVal = 10.0f;
  for (int i = 0; i < 6; i++) if (cellVolt[i] < lowVal && cellVolt[i] > IgnoreCell) lowVal = cellVolt[i];
  return lowVal;
}

/**
 * @brief Gets the highest cell voltage of the module
 *
 * @return The highest cell voltage in Volts, ignoring any cell voltage that is greater than 4.5V
 */
float BMSModule::getHighCellV()
{
  float hiVal = 0.0f;
  for (int i = 0; i < 6; i++) if (cellVolt[i] > hiVal && cellVolt[i] < 4.5) hiVal = cellVolt[i];
  return hiVal;
}

/**
 * @brief Calculates the average cell voltage of the module
 *
 * This function loops over all 6 cell voltage values and averages the ones that
 * are within the valid range (IgnoreCell to 60.0V). It keeps track of the number
 * of valid cells and if the number of valid cells changes, it increments a miss
 * counter. If the number of valid cells is the same for 3 consecutive reads, it
 * resets the miss counter and sets the number of valid cells to the current value.
 *
 * @return The average cell voltage of the module in Volts
 */
float BMSModule::getAverageV()
{
  int x = 0;
  float avgVal = 0.0f;
  // TODO - Should this be hard coded to 6?
  for (int i = 0; i < 6; i++)
  {
    if (cellVolt[i] > IgnoreCell && cellVolt[i] < 60.0)
    {
      x++;
      avgVal += cellVolt[i];
    }
  }
  
  if (scells != x)
  {
    if (smiss > 2)
    {
      scells = x;
    }
    else
    {
      smiss++;
    }
  }
  else
  {
    scells = x;
    smiss = 0;
  }
  
  avgVal /= x;
  return avgVal;
}

/**
 * @brief Gets the number of cells in the module that are within the valid voltage range
 *
 * This function returns the number of cells in the module that are within the valid
 * voltage range (i.e. above the IgnoreCell value and below 60.0V). This value is
 * calculated and cached by the getAverageV() function.
 *
 * @return The number of cells in the module that are within the valid voltage range
 */
int BMSModule::getscells()
{
  return scells;
}

/**
 * @brief Returns the highest module voltage that has been seen
 *
 * @return The highest module voltage in Volts
 */
float BMSModule::getHighestModuleVolt()
{
  return highestModuleVolt;
}

/**
 * @brief Returns the lowest module voltage that has been seen
 *
 * @return The lowest module voltage in Volts
 */
float BMSModule::getLowestModuleVolt()
{
  return lowestModuleVolt;
}

/**
 * @brief Returns the highest cell voltage of the specified cell that has been seen
 *
 * @param cell cell number, valid values are 0-5
 * @return The highest cell voltage of the specified cell in Volts
 */
float BMSModule::getHighestCellVolt(int cell)
{
  if (cell < 0 || cell > 5) return 0.0f;
  return highestCellVolt[cell];
}

/**
 * @brief Returns the lowest cell voltage of the specified cell that has been seen
 *
 * @param cell cell number, valid values are 0-5
 * @return The lowest cell voltage of the specified cell in Volts
 */
float BMSModule::getLowestCellVolt(int cell)
{
  if (cell < 0 || cell > 5) return 0.0f;
  return lowestCellVolt[cell];
}

/**
 * @brief Returns the highest temperature reading from the module
 *
 * @return The highest temperature in degrees Celsius
 */
float BMSModule::getHighestTemp()
{
  return highestTemperature;
}

/**
 * @brief Returns the lowest temperature reading from the module
 *
 * @return The lowest temperature in degrees Celsius
 */
float BMSModule::getLowestTemp()
{
  return lowestTemperature;
}

/**
 * @brief Returns the lowest temperature of the module
 *
 * If sensor is 0, returns the lowest of the two temperature readings. If sensor is 1 or 2, returns the
 * temperature reading from the specified sensor.
 *
 * @return The lowest temperature in degrees Celsius
 */
float BMSModule::getLowTemp()
{
  if (sensor == 0)
  {
    return (temperatures[0] < temperatures[1]) ? temperatures[0] : temperatures[1];
  }
  else
  {
    return temperatures[sensor - 1];
  }
}

/**
 * @brief Returns the highest temperature reading from the module
 *
 * If sensor is 0, returns the highest of the two temperature readings. If sensor is 1 or 2, returns the
 * temperature reading from the specified sensor.
 *
 * @return The highest temperature in degrees Celsius
 */
float BMSModule::getHighTemp()
{
  if (sensor == 0)
  {
    return (temperatures[0] < temperatures[1]) ? temperatures[1] : temperatures[0];
  }
  else
  {
    return temperatures[sensor - 1];
  }

}

/**
 * @brief Calculates the average temperature of the module
 *
 * If sensor is 0, calculates the average of the two temperature readings. If sensor is 1 or 2, returns the
 * temperature reading from the specified sensor.
 *
 * @return The average temperature in degrees Celsius
 */
float BMSModule::getAvgTemp()
{
  if (sensor == 0)
  {
    return (temperatures[0] + temperatures[1]) * 0.5f;
  }
  else
  {
    return temperatures[sensor - 1];
  }
}

/**
 * @brief Retrieves the module voltage of the module
 *
 * @return The module voltage in Volts
 */
float BMSModule::getModuleVoltage()
{
  return moduleVolt;
}

/**
 * @brief Retrieves the temperature of a specific sensor in the module
 *
 * @param temp Temperature sensor ID. Valid values are 0 or 1.
 * @return The temperature in degrees Celsius
 */
float BMSModule::getTemperature(int temp)
{
  if (temp < 0 || temp > 1) return 0.0f;
  return temperatures[temp];
}

/**
 * @brief Sets the address of the module
 *
 * @param newAddr The new address of the module. Valid values are 1-62.
 */
void BMSModule::setAddress(int newAddr)
{
  if (newAddr < 0 || newAddr > MAX_MODULE_ADDR) return;
  moduleAddress = newAddr;
}

/**
 * @brief Gets the address of the module
 *
 * @return The address of the module (1-62)
 */
int BMSModule::getAddress()
{
  return moduleAddress;
}

/**
 * @brief Determines if the module is existing (i.e. if it was found during the findBoards() process)
 *
 * @return true if the module exists, false otherwise
 */
bool BMSModule::isExisting()
{
  return exists;
}

/**
 * @brief Sets the temperature sensor to read from the module.
 *
 * @param tempsensor The number of the temperature sensor to read from (0 or 1)
 */
void BMSModule::settempsensor(int tempsensor)
{
  sensor = tempsensor;
}

/**
 * @brief Sets the existence of the module
 *
 * @param ex true if the module exists, false otherwise
 */
void BMSModule::setExists(bool ex)
{
  exists = ex;
}

/**
 * @brief Sets the voltage level below which a cell is ignored during balancing.
 *
 * This is useful for cells that are not connected to the BMS (e.g. if you have
 * a 12S BMS and you are using 10S or 14S batteries). You can set the ignore
 * voltage to a value that is below the lowest cell voltage of the unused
 * cells, so that the BMS does not try to balance the unused cells.
 *
 * @param Ignore The voltage level (in Volts) below which the cell is ignored.
 */
void BMSModule::setIgnoreCell(float Ignore)
{
  IgnoreCell = Ignore;
}
