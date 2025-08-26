#pragma once

#include <Arduino.h>

// Set to the proper port for your USB connection - SerialUSB on Due (Native) or Serial for Due (Programming) or Teensy
#define SERIAL_CONSOLE Serial

// Define this to be the serial port the Tesla BMS modules are connected to.
// On the Due you need to use a USART port (Serial1, Serial2, Serial3) and update the call to
// serialSpecialInit if not Serial1
// Serial3 for teensy
#define SERIALBMS Serial2
#define SERIAL_AUX Serial7

#define EEPROM_VERSION 0x14 // update any time EEPROM struct below is changed.
#define EEPROM_PAGE 0

typedef struct
{
  uint8_t version;
  uint8_t checksum;
  uint32_t canSpeed;
  uint8_t batteryID; // which battery ID should this board associate as on the CAN bus
  uint8_t logLevel;
  float OverVSetpoint;
  float UnderVSetpoint;
  float DischHys;
  float ChargeVsetpoint;
  float DischVsetpoint;
  float ChargeHys;
  float StoreVsetpoint;
  float WarnOff;
  float OverTSetpoint;
  float UnderTSetpoint;
  uint16_t triptime;
  float ChargeTSetpoint;
  float DisTSetpoint;
  float WarnToff;
  float CellGap;
  uint8_t IgnoreTemp;
  float IgnoreVolt;
  float balanceVoltage;
  float balanceHyst;
  int Scells;
  int Pstrings;
  int CAP;
  uint16_t chargecurrentmax;
  uint16_t chargecurrent2max;
  uint16_t chargecurrentend;
  uint16_t discurrentmax;
  int socvolt[4];
  int invertcur;
  int cursens;
  int curcan;
  int voltsoc;
  int Pretime;
  int conthold;
  int Precurrent;
  float convhigh;
  float convlow;
  int32_t changecur;
  uint16_t offset1;
  uint16_t offset2;
  int balanceDuty;
  int ESSmode;
  int gaugelow;
  int gaugehigh;
  int ncur;
  int chargertype;
  uint16_t chargerspd;
  uint16_t UnderDur;
  uint16_t CurDead;
  float DisTaper;
  uint8_t ChargerDirect; // bool
  uint8_t ExpMess;       // bool
  uint8_t SerialCan;     // bool
  uint16_t PulseCh;
  uint16_t PulseChDur;
  uint16_t PulseDi;
  uint16_t PulseDiDur;
  uint8_t tripcont;
  int chargereff;
  int chargerACv;
} EEPROMSettings;
