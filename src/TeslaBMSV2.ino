
/*
  Copyright (c) 2019 Simp ECO Engineering
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

// local files
#include "pinouts.h"
#include "globals.h"
#include "Settings.h"
#include "indicators.h"
#include "BMSModuleManager.h"
#include "config.h"
#include "SerialConsole.h"
#include "Logger.h"
#include "SerialMenu.h"
#include "CurrentSensing.h"
#include "PhysicalGauges.h"

// Libraries
#include <Arduino.h>
#include <ADC.h> //https://github.com/pedvide/ADC
#include <EEPROM.h>
#include <FlexCAN_T4.h> //https://github.com/collin80/FlexCAN_Library
#include <SPI.h>
#include <Filters.h>                  //https://github.com/JonHub/Filters
#include <Serial_CAN_Module_Teensy.h> //https://github.com/tomdebree/Serial_CAN_Teensy

// T4 Additions
#include <Watchdog_t4.h>
#include <imxrt.h>
#include <CrashReport.h>

/////Version Identifier/////////
int firmver = 230719; // Year Month Day
const char *COMPILE_DATE = __DATE__;
const char *COMPILE_TIME = __TIME__;

WDT_T4<WDT1> watchdog;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

Serial_CAN can;
BMSModuleManager bms;
SerialConsole console; // TODO - Appears unused?
EEPROMSettings settings;

ADC *adc = new ADC(); // adc object

CAN_message_t msg;
CAN_message_t inMsg;

// Prototypes
void setBMSstatus(int newstatus, const char *message = nullptr);
void setBMSstatus(int newstatus, int newError = 0, const char *message = nullptr);
const char *getBMSStatusString(int status);
const char *getBMSErrorString(int error);

void setup()
{
  indicatorsSetup();
  // ------------- LED Indicators -------------
  pinMode(PIN_LED_BUILTIN, OUTPUT);
  pinMode(PIN_HEARTBEAT_LED, OUTPUT);
  pinMode(PIN_ERROR_LED, OUTPUT);
  pinMode(PIN_BUZZER_CONTROL, OUTPUT);

  digitalWrite(PIN_LED_BUILTIN, HIGH);
  digitalWrite(PIN_HEARTBEAT_LED, HIGH);
  digitalWrite(PIN_ERROR_LED, HIGH);
  digitalWrite(PIN_BUZZER_CONTROL, HIGH);
  // ------------- Start Tesla Serial BUS -------------
  SERIALBMS.begin(612500); // Tesla serial bus
  delay(2000);             // just for easy debugging. It takes a few seconds for USB to come up properly on most OS's

  // ------------- Pin Mode Assignments -------------
  // pinMode(ACUR1, INPUT);//Not required for Analogue Pins
  // pinMode(ACUR2, INPUT);//Not required for Analogue Pins
  pinMode(PIN_IN1, INPUT_PULLUP);
  pinMode(PIN_IN2, INPUT_PULLUP);
  pinMode(PIN_IN3, INPUT_PULLUP);
  pinMode(PIN_IN4, INPUT_PULLUP);
  pinMode(PIN_OUT1, OUTPUT); // Positive contactor
  pinMode(PIN_OUT2, OUTPUT); // precharge
  pinMode(PIN_OUT3, OUTPUT); // charge relay
  pinMode(PIN_OUT4, OUTPUT); // Negative contactor
  pinMode(PIN_OUT5, OUTPUT); // pwm driver output
  pinMode(PIN_OUT6, OUTPUT); // pwm driver output
  pinMode(PIN_OUT7, OUTPUT); // pwm driver output
  pinMode(PIN_OUT8, OUTPUT); // pwm driver output

  // ------------- PWM Output Configuration -------------
  analogWriteFrequency(PIN_OUT5, pwmfreq);
  analogWriteFrequency(PIN_OUT6, pwmfreq);
  analogWriteFrequency(PIN_OUT7, pwmfreq);
  analogWriteFrequency(PIN_OUT8, pwmfreq);

  // ------------- LEDS and Buzzer off-------------
  digitalWrite(PIN_LED_BUILTIN, LOW);
  digitalWrite(PIN_HEARTBEAT_LED, LOW);
  digitalWrite(PIN_ERROR_LED, LOW);
  digitalWrite(PIN_BUZZER_CONTROL, LOW);

  // ------------- EEPROM Setting Retreival -------------
  EEPROM.get(0, settings);
  if (settings.version != EEPROM_VERSION)
  {
    loadSettings();
  }

  // ------------- CAN Setup -------------
  Can1.begin();
  Can1.setBaudRate(settings.canSpeed);
  // Can1.setMaxMB(16);
  // Can1.enableFIFO();
  // Can1.enableFIFOInterrupt();
  // Can1.onReceive(canRead);
  // Can1.mailboxStatus();

  // Filters for T3.1 - FlexCAN_T4 should be unfiltered by default
  // CAN_filter_t allPassFilter; // Enables extended addresses
  // allPassFilter.id = 0;
  // allPassFilter.ext = 1;
  // allPassFilter.rtr = 0;

  // for (int filterNum = 4; filterNum < 16; filterNum++)
  // {
  //   Can1.setFilter(allPassFilter, filterNum);
  // }

  // if using enable pins on a transceiver they need to be set on

  adc->adc0->setAveraging(16);  // set number of averages
  adc->adc0->setResolution(16); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED);
  adc->adc0->startContinuous(PIN_ACUR_1);

  SERIAL_CONSOLE.begin(115200);
  SERIAL_CONSOLE.println("Starting up!");
  SERIAL_CONSOLE.println("SimpBMS V2 Tesla");

  Serial2.begin(115200); // display and can adpater canbus

  // Teensy 4.x
  if (CrashReport)
  {
    // print info (hope Serial Monitor windows is open)
    // TODO - Log to SD card
    Serial.print(CrashReport);
  }

  //  Enable WDT T4.x
  WDT_timings_t configewm;
  configewm.timeout = 2000;
  configewm.pin = 21;
  watchdog.begin(configewm);
  delay(100);
  watchdog.feed();
  /* window mode test */
  delay(100); /* <-- not keeping this here would cause resets by Callback */

  // VE.begin(19200); //Victron VE direct bus

  SERIAL_CONSOLE.println("Started serial interface to BMS.");

  /*
    EEPROM.get(0, settings);
    if (settings.version != EEPROM_VERSION)
    {
      loadSettings();
    }
  */

  bms.renumberBoardIDs();

  Logger::setLoglevel(Logger::Debug); // Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4

  lastUpdate = 0;
  bms.findBoards();
  digitalWrite(PIN_LED_BUILTIN, HIGH);
  // bmsstatus = BMS_STATUS_ERROR;
  setBMSstatus(BMS_STATUS_ERROR, "Initial state on setup");
  bmsError = ERROR_CAN; // TODO - refine error state
  bms.setPstrings(settings.Pstrings);
  bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);

  // SOC recovery//

  SOC = (EEPROM.read(1000));
  if (settings.voltsoc == 1)
  {
    SOCmem = 0;
  }
  else
  {
    if (SOC > 100)
    {
      SOCmem = 0;
    }
    else if (SOC > 1)
    {
      // SOCmem = 1;
    }
  }

  SERIAL_CONSOLE.println("Recovery SOC: ");
  SERIAL_CONSOLE.print(SOC);

  ////Calculate fixed numbers (??)
  pwmcurmin = (pwmcurmid / 50 * pwmcurmax * -1);
  ////
  bms.clearFaults();

  /// precharge timer kickers
  Pretimer = millis();
  Pretimer1 = millis();

  // setup interrupts
  // RISING/HIGH/CHANGE/LOW/FALLING
  attachInterrupt(PIN_IN4, isrCP, CHANGE); // attach BUTTON 1 interrupt handler [ pin# 21 ]

  // TODO Low/High Voltage Interrupt disabled for T4
  // PMC_LVDSC1 = PMC_LVDSC1_LVDV(1);                    // enable hi v
  // PMC_LVDSC2 = PMC_LVDSC2_LVWIE | PMC_LVDSC2_LVWV(3); // 2.92-3.08v
  // attachInterruptVector(IRQ_LOW_VOLTAGE, low_voltage_isr);
  // NVIC_ENABLE_IRQ(IRQ_LOW_VOLTAGE);

  // Blink Heartbeat LED to indicate setup is complete
  digitalWrite(PIN_HEARTBEAT_LED, HIGH); // Turn off heartbeat LED
  delay(500);                            // Wait for 1 second
  digitalWrite(PIN_HEARTBEAT_LED, LOW);  // Turn on heartbeat LED

  // bmsstatus = BMS_STATUS_BOOT;
  setBMSstatus(BMS_STATUS_BOOT, "Setup complete");
  bmsError = ERROR_NONE;
}

void loop()
{

  indicatorsLoop(); // Call the indicators loop to handle LED and buzzer state

  // TODO - CANx.available not implemented for T4 - need to read CAN here
  // On message recieve.
  // while (CAN1.available())
  // {
  //   canread();
  // }

  // Check if serial menu is requested
  if (SERIAL_CONSOLE.available() > 0)
  {
    menu();
  }

  // Serial.printf("Loop: If output check. Outputcheck: %d, bmsStatus: %d\n", outputcheck, bmsstatus);
  if (outputcheck != 1)
  {
    contactorControl();
    if (settings.ESSmode == 1)
    {
      if (settings.ChargerDirect == 1)
      {
        OutputEnable = 1;
      }
      else
      {
        if (digitalRead(PIN_IN2) == HIGH || CanOnReq == true)
        {
          OutputEnable = 1;
          // Serial.println(CanOnReq);
        }
        else
        {
          OutputEnable = 0;
        }
      }
      if (bmsstatus != BMS_STATUS_ERROR && bmsstatus != BMS_STATUS_BOOT && OutputEnable == 1)
      {
        contctrl = contctrl | 4; // turn on negative contactor
        if (settings.tripcont != 0)
        {
          if (bms.getLowCellVolt() > settings.UnderVSetpoint && bms.getHighCellVolt() < settings.OverVSetpoint)
          {
            if (digitalRead(PIN_OUT2) == LOW && digitalRead(PIN_OUT4) == LOW)
            {
              mainconttimer = millis();
              digitalWrite(PIN_OUT4, HIGH); // Precharge start
              Serial.println();
              Serial.println("Precharge!!!");
              Serial.printf("Conditions: Output2: %d, output4: %d, bmsStatus: %d\n", digitalRead(PIN_OUT2), digitalRead(PIN_OUT4), bmsstatus);
              Serial.println();
              Serial.println(mainconttimer);
              Serial.println();
            }
            if (mainconttimer + settings.Pretime < millis() && digitalRead(PIN_OUT2) == LOW && abs(currentact) < settings.Precurrent)
            {
              digitalWrite(PIN_OUT2, HIGH); // turn on contactor
              contctrl = contctrl | 2;      // turn on contactor
              Serial.println();
              Serial.println("Main On!!!");
              Serial.println();
              mainconttimer = millis() + settings.Pretime;
            }
            if (mainconttimer + settings.Pretime + 1000 < millis())
            {
              digitalWrite(PIN_OUT4, LOW); // ensure precharge is low
            }
          }
          else
          {
            digitalWrite(PIN_OUT4, LOW); // ensure precharge is low
            mainconttimer = 0;
          }
        }
        if (digitalRead(PIN_IN1) == LOW) // Key OFF
        {
          if (storagemode == 1)
          {
            storagemode = 0;
          }
        }
        else
        {
          if (storagemode == 0)
          {
            storagemode = 1;
          }
        }
        if (bms.getHighCellVolt() > settings.balanceVoltage && bms.getHighCellVolt() > bms.getLowCellVolt() + settings.balanceHyst)
        {
          balancecells = 1;
        }
        else
        {
          balancecells = 0;
        }

        // Pretimer + settings.Pretime > millis();

        if (storagemode == 1)
        {
          if (bms.getHighCellVolt() > settings.StoreVsetpoint || chargecurrent == 0)
          {
            digitalWrite(PIN_OUT3, LOW); // turn off charger
            // contctrl = contctrl & 253;
            // Pretimer = millis();
            Charged = 1;
            SOCcharged(2);
          }
          else
          {
            if (Charged == 1)
            {
              if (bms.getHighCellVolt() < (settings.StoreVsetpoint - settings.ChargeHys))
              {
                Charged = 0;
                digitalWrite(PIN_OUT3, HIGH); // turn on charger
                /*
                  if (Pretimer + settings.Pretime < millis())
                  {
                  contctrl = contctrl | 2;
                  Pretimer = 0;
                  }
                */
              }
            }
            else
            {
              digitalWrite(PIN_OUT3, HIGH); // turn on charger
              /*
                if (Pretimer + settings.Pretime < millis())
                {
                contctrl = contctrl | 2;
                Pretimer = 0;
                }
              */
            }
          }
        }
        else
        {
          if (bms.getHighCellVolt() > settings.OverVSetpoint || bms.getHighCellVolt() > settings.ChargeVsetpoint || chargecurrent == 0)
          {
            if ((millis() - overtriptimer) > settings.triptime)
            {
              if (digitalRead(PIN_OUT3) == 1)
              {
                Serial.println();
                Serial.println("Over Voltage Trip");
                digitalWrite(PIN_OUT3, LOW); // turn off charger
                // contctrl = contctrl & 253;
                // Pretimer = millis();
                Charged = 1;
                SOCcharged(2);
              }
            }
          }
          else
          {
            overtriptimer = millis();
            if (Charged == 1)
            {

              if (bms.getHighCellVolt() < (settings.ChargeVsetpoint - settings.ChargeHys))
              {
                if (digitalRead(PIN_OUT3) == 0)
                {
                  Serial.println();
                  Serial.println("Reset Over Voltage Trip Not Charged");
                  Charged = 0;
                  digitalWrite(PIN_OUT3, HIGH); // turn on charger
                }
                /*
                  if (Pretimer + settings.Pretime < millis())
                  {
                  // Serial.println();
                  //Serial.print(Pretimer);
                  contctrl = contctrl | 2;
                  }*/
              }
            }
            else
            {
              if (digitalRead(PIN_OUT3) == 0)
              {
                Serial.println();
                Serial.println("Reset Over Voltage Trip Not Charged");
                digitalWrite(PIN_OUT3, HIGH); // turn on charger
              }
              /*
                if (Pretimer + settings.Pretime < millis())
                {
                // Serial.println();
                //Serial.print(Pretimer);
                contctrl = contctrl | 2;
                }*/
            }
          }
        }

        if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getLowCellVolt() < settings.DischVsetpoint)
        {
          if (digitalRead(PIN_OUT1) == 1)
          {

            if ((millis() - undertriptimer) > settings.triptime)
            {
              Serial.println();
              Serial.println("Under Voltage Trip");
              digitalWrite(PIN_OUT1, LOW); // turn off discharge
              // contctrl = contctrl & 254;
              // Pretimer1 = millis();
            }
          }
        }
        else
        {
          undertriptimer = millis();

          if (bms.getLowCellVolt() > settings.DischVsetpoint + settings.DischHys)
          {
            if (digitalRead(PIN_OUT1) == 0)
            {
              Serial.println();
              Serial.println("Reset Under Voltage Trip");
              digitalWrite(PIN_OUT1, HIGH); // turn on discharge
            }
            /*
              if (Pretimer1 + settings.Pretime < millis())
              {
              contctrl = contctrl | 1;
              }*/
          }
        }

        if (SOCset == 1)
        {
          if (settings.tripcont == 0)
          {
            if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getHighCellVolt() > settings.OverVSetpoint || bms.getHighTemperature() > settings.OverTSetpoint)
            {
              digitalWrite(PIN_OUT2, HIGH); // trip breaker
              // bmsstatus = BMS_STATUS_ERROR;
              setBMSstatus(BMS_STATUS_ERROR, "Voltage or Temperature fault in ESS mode with tripcont=0");
              bmsError = ERROR_VOLTAGE; // TODO - refine error state
              // TODO - break out the above if statement to assign correct error
            }
            else
            {
              digitalWrite(PIN_OUT2, LOW); // trip breaker
            }
          }
          else
          {
            if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getHighCellVolt() > settings.OverVSetpoint || bms.getHighTemperature() > settings.OverTSetpoint)
            {
              digitalWrite(PIN_OUT2, LOW); // turn off contactor
              contctrl = contctrl & 253;   // turn off contactor
              digitalWrite(PIN_OUT4, LOW); // ensure precharge is low
              // bmsstatus = BMS_STATUS_ERROR;
              setBMSstatus(BMS_STATUS_ERROR, "Voltage or Temperature fault in ESS mode with tripcont=1");
              bmsError = ERROR_VOLTAGE; // TODO - refine error state
            }
          }
        }
      }
      else
      {
        // digitalWrite(OUT2, HIGH);//trip breaker
        Discharge = 0;
        digitalWrite(PIN_OUT4, LOW);
        digitalWrite(PIN_OUT3, LOW); // turn off charger
        digitalWrite(PIN_OUT2, LOW);
        digitalWrite(PIN_OUT1, LOW); // turn off discharge
        contctrl = 0;                // turn off out 5 and 6

        if (SOCset == 1)
        {
          if (settings.tripcont == 0)
          {

            digitalWrite(PIN_OUT2, HIGH); // trip breaker
          }
          else
          {
            digitalWrite(PIN_OUT2, LOW); // turn off contactor
            digitalWrite(PIN_OUT4, LOW); // ensure precharge is low
          }

          if (bms.getLowCellVolt() > settings.UnderVSetpoint && bms.getHighCellVolt() < settings.OverVSetpoint && bms.getHighTemperature() < settings.OverTSetpoint && cellspresent == bms.seriescells() && cellspresent == (settings.Scells * settings.Pstrings))
          {
            // bmsstatus = BMS_STATUS_READY;
            setBMSstatus(BMS_STATUS_READY, "Voltage and Temperature normal in ESS mode");
            bmsError = ERROR_NONE;
          }
        }
      }
      // pwmcomms();
    }
    else
    {
      switch (bmsstatus)
      {
      case (BMS_STATUS_BOOT):
        Discharge = 0;
        digitalWrite(PIN_OUT4, LOW);
        digitalWrite(PIN_OUT3, LOW); // turn off charger
        digitalWrite(PIN_OUT2, LOW);
        digitalWrite(PIN_OUT1, LOW); // turn off discharge
        contctrl = 0;
        // bmsstatus = BMS_STATUS_READY;
        setBMSstatus(BMS_STATUS_READY, "Boot complete");
        break;

      case (BMS_STATUS_READY):
        Discharge = 0;
        digitalWrite(PIN_OUT4, LOW);
        digitalWrite(PIN_OUT3, LOW); // turn off charger
        digitalWrite(PIN_OUT2, LOW);
        digitalWrite(PIN_OUT1, LOW); // turn off discharge
        contctrl = 0;                // turn off out 5 and 6
        accurlim = 0;
        if (bms.getHighCellVolt() > settings.balanceVoltage && bms.getHighCellVolt() > bms.getLowCellVolt() + settings.balanceHyst)
        {
          // bms.balanceCells();
          balancecells = 1;
        }
        else
        {
          balancecells = 0;
        }
        if (digitalRead(PIN_IN3) == HIGH && (bms.getHighCellVolt() < (settings.ChargeVsetpoint - settings.ChargeHys)) && bms.getHighTemperature() < (settings.OverTSetpoint - settings.WarnToff)) // detect AC present for charging and check not balancing
        {
          if (settings.ChargerDirect == 1)
          {
            // bmsstatus = BMS_STATUS_CHARGE;
            setBMSstatus(BMS_STATUS_CHARGE, "AC detected in READY state with ChargerDirect=1");
          }
          else
          {
            // bmsstatus = BMS_STATUS_PRECHARGE;
            setBMSstatus(BMS_STATUS_PRECHARGE, "AC detected in READY state with ChargerDirect=0");
            Pretimer = millis();
          }
        }
        if (digitalRead(PIN_IN1) == HIGH && bms.getLowCellVolt() > settings.DischVsetpoint) // detect Key ON
        {
          // bmsstatus = BMS_STATUS_PRECHARGE;
          setBMSstatus(BMS_STATUS_PRECHARGE, "Key ON detected in READY state");
          Pretimer = millis();
        }

        break;

      case (BMS_STATUS_PRECHARGE):
        Discharge = 0;
        Prechargecon();
        break;

      case (BMS_STATUS_DRIVE):
        Discharge = 1;
        accurlim = 0;
        if (digitalRead(PIN_IN1) == LOW) // Key OFF
        {
          // bmsstatus = BMS_STATUS_READY;
          setBMSstatus(BMS_STATUS_READY, "Key OFF detected in DRIVE state");
        }
        if (digitalRead(PIN_IN3) == HIGH && (bms.getHighCellVolt() < (settings.ChargeVsetpoint - settings.ChargeHys)) && bms.getHighTemperature() < (settings.OverTSetpoint - settings.WarnToff)) // detect AC present for charging and check not balancing
        {
          // bmsstatus = BMS_STATUS_CHARGE;
          setBMSstatus(BMS_STATUS_CHARGE, "AC detected in DRIVE state");
        }

        break;

      case (BMS_STATUS_CHARGE):
        if (settings.ChargerDirect > 0)
        {
          Discharge = 0;
          digitalWrite(PIN_OUT4, LOW);
          digitalWrite(PIN_OUT2, LOW);
          digitalWrite(PIN_OUT1, LOW); // turn off discharge
          contctrl = 0;                // turn off out 5 and 6
        }
        Discharge = 0;
        if (digitalRead(PIN_IN2) == HIGH)
        {
          chargecurrentlimit = true;
        }
        else
        {
          chargecurrentlimit = false;
        }
        digitalWrite(PIN_OUT3, HIGH); // enable charger
        if (bms.getHighCellVolt() > settings.balanceVoltage)
        {
          // bms.balanceCells();
          balancecells = 1;
        }
        else
        {
          balancecells = 0;
        }
        if (bms.getHighCellVolt() > settings.ChargeVsetpoint || bms.getHighTemperature() > settings.OverTSetpoint)
        {
          if (bms.getAvgCellVolt() > (settings.ChargeVsetpoint - settings.ChargeHys))
          {
            SOCcharged(2);
          }
          else
          {
            SOCcharged(1);
          }
          digitalWrite(PIN_OUT3, LOW); // turn off charger
          // bmsstatus = BMS_STATUS_READY;
          setBMSstatus(BMS_STATUS_READY, "Charge or Temperature setpoint reached in CHARGE state");
        }
        if (digitalRead(PIN_IN3) == LOW) // detect AC not present for charging
        {
          // bmsstatus = BMS_STATUS_READY;
          setBMSstatus(BMS_STATUS_READY, "AC not detected in CHARGE state");
        }
        break;

      case (BMS_STATUS_ERROR):
        Discharge = 0;
        digitalWrite(PIN_OUT4, LOW);
        digitalWrite(PIN_OUT3, LOW); // turn off charger
        digitalWrite(PIN_OUT2, LOW);
        digitalWrite(PIN_OUT1, LOW); // turn off discharge
        contctrl = 0;                // turn off out 5 and 6
        /*
                  if (digitalRead(IN3) == HIGH) //detect AC present for charging
                  {
                    bmsstatus = Charge;
                  }
        */
        if (bms.getLowCellVolt() >= settings.UnderVSetpoint && bms.getHighCellVolt() <= settings.OverVSetpoint && digitalRead(PIN_IN1) == LOW)
        {
          // bmsstatus = BMS_STATUS_READY;
          setBMSstatus(BMS_STATUS_READY, "Voltage and Key OFF normal in ERROR state");
        }

        break;
      }
    }
    if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL || settings.cursens == CURR_SENSE_ANALOGUE_GUESSING)
    {
      getcurrent();
    }
  }

  if (millis() > nextLoopTime)
  {
    nextLoopTime = millis() + 500; // Every 500ms

    // Update BMS data
    bms.getAllVoltTemp();

    // Cell Voltage  check
    if (settings.ESSmode == 1) // ESS Mode
    {
      // Check if Lowest cell or highest cell is below the Undervoltage setpoint. (Why lowest and highest??)
      if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getHighCellVolt() < settings.UnderVSetpoint)
      {
        if (undertriptimer > millis()) // check is last time not undervoltage is longer than UnderDur ago
        {
          // bmsstatus = BMS_STATUS_ERROR;
          setBMSstatus(BMS_STATUS_ERROR, "Undervoltage detected in ESS mode");
          bmsError = ERROR_VOLTAGE; // TODO - refine error state
        }
      }
      else
      {
        undertriptimer = millis() + settings.triptime;
      }

      if (bms.getLowCellVolt() > settings.OverVSetpoint || bms.getHighCellVolt() > settings.OverVSetpoint)
      {
        if (overtriptimer > millis()) // check is last time not undervoltage is longer thatn UnderDur ago
        {
          // bmsstatus = BMS_STATUS_ERROR;
          setBMSstatus(BMS_STATUS_ERROR, "Overvoltage detected in ESS mode");
          bmsError = ERROR_VOLTAGE; // TODO - refine error state
        }
      }
      else
      {
        overtriptimer = millis() + settings.triptime;
      }
    }
    else // In 'vehicle' mode
    {
      if (bms.getLowCellVolt() < settings.UnderVSetpoint)
      {
        if (UnderTimer < millis()) // check is last time not undervoltage is longer thatn UnderDur ago
        {
          // bmsstatus = BMS_STATUS_ERROR;
          setBMSstatus(BMS_STATUS_ERROR, "Undervoltage detected in vehicle mode");
          bmsError = ERROR_VOLTAGE; // TODO - refine error state
        }
      }
      else
      {
        UnderTimer = millis() + settings.triptime;
      }

      if (bms.getHighCellVolt() < settings.UnderVSetpoint || bms.getHighTemperature() > settings.OverTSetpoint)
      {
        // bmsstatus = BMS_STATUS_ERROR;
        setBMSstatus(BMS_STATUS_ERROR, "Undervoltage or Overtemperature detected in vehicle mode");
        bmsError = ERROR_VOLTAGE; // TODO - refine error state
      }

      if (bms.getHighCellVolt() > settings.OverVSetpoint)
      {
        if (OverTime < millis()) // check is last time not undervoltage is longer thatn UnderDur ago
        {
          bmsstatus = BMS_STATUS_ERROR;
          setBMSstatus(BMS_STATUS_ERROR, "Overvoltage detected in vehicle mode");
          bmsError = ERROR_VOLTAGE; // TODO - refine error state
        }
      }
      else
      {
        OverTime = millis() + settings.triptime;
      }
    }

    balancing();

    if (debug != 0)
    {
      printbmsstat();
      bms.printPackDetails(debugdigits);
    }
    if (CSVdebug != 0)
    {
      bms.printAllCSV(millis(), currentact, SOC, delim);
    }
    if (inputcheck != 0)
    {
      inputdebug();
    }

    if (outputcheck != 0)
    {
      outputdebug();
    }
    else
    {
      gaugeUpdate();
    }
    updateSOC();
    currentlimit();

    if (settings.ESSmode == 1 && settings.ChargerDirect == 0 && CanOnRev == true)
    {
      if ((millis() - CanOntimeout) > 5000)
      {
        Serial.println();
        Serial.println("0x309 Can On Request Missing");
        CanOnReq = false;
      }
    }

    if (cellspresent == 0 && SOCset == 1)
    {
      cellspresent = bms.seriescells();
      bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
    }
    else
    {
      if (cellspresent != bms.seriescells() || cellspresent != (settings.Scells * settings.Pstrings)) // detect a fault in cells detected
      {
        if (debug != 0)
        {
          SERIAL_CONSOLE.println("  ");
          SERIAL_CONSOLE.print("   !!! Series Cells Fault !!!");
          SERIAL_CONSOLE.println("  ");
          bmsstatus = BMS_STATUS_ERROR;
          setBMSstatus(BMS_STATUS_ERROR, "Series Cells count fault detected");
          bmsError = ERROR_VOLTAGE; // TODO - refine error state
        }
      }
    }
    alarmupdate();
    if (CSVdebug != 1)
    {
      dashupdate();
    }
    resetwdog();
  }

  if (millis() - cleartime > 5000)
  {
    bms.clearmodules();
    cleartime = millis();
  }

  if (millis() - looptime1 > settings.chargerspd)
  {
    looptime1 = millis();
    VEcan();
    if (settings.ESSmode == 1)
    {
      chargercomms();
    }
    else
    {
      if (bmsstatus == BMS_STATUS_CHARGE)
      {
        chargercomms();
      }
    }
  }
}

void setBMSstatus(int newstatus, const char *message = nullptr)
{
  bmsstatus = newstatus;
  if (message)
  {
    Logger::debug("BMS Status set to %s: %s", getBMSStatusString(newstatus), message);
  }
  else
  {
    Logger::debug("BMS Status set to %s", getBMSStatusString(newstatus));
  }
}

void setBMSstatus(int newstatus, int newError = 0, const char *message = nullptr)
{
  setBMSstatus(newstatus, message);

  if (newError != 0)
  {
    bmsError = newError;
    Logger::debug("BMS Error set to %d", newError);
  }
}

const char *getBMSStatusString(int status)
{
  switch (status)
  {
  case BMS_STATUS_BOOT:
    return "Boot";
  case BMS_STATUS_READY:
    return "Ready";
  case BMS_STATUS_PRECHARGE:
    return "Precharge";
  case BMS_STATUS_DRIVE:
    return "Drive";
  case BMS_STATUS_CHARGE:
    return "Charge";
  case BMS_STATUS_ERROR:
    return "Error";
  default:
    return "Unknown";
  }
}

// a function to recieve the error state and return a string for display
const char *getBMSErrorString(int error)
{
  switch (error)
  {
  case ERROR_NONE:
    return "No Error";
  case ERROR_CAN:
    return "CAN Error";
  case ERROR_VOLTAGE:
    return "Voltage Error";
  case ERROR_UNDER_VOLTAGE:
    return "Undervoltage Error";
  case ERROR_OVER_VOLTAGE:
    return "Overvoltage Error";
  case ERROR_OVER_TEMPERATURE:
    return "Over Temperature Error";
  case ERROR_UNDER_TEMPERATURE:
    return "Under Temperature Error";
  case ERROR_CURRENT_READING:
    return "Current Sensor Error";
  default:
    return "Unknown Error";
  }
}

/**
 * @brief Updates the alarm and warning status arrays based on current BMS readings and settings.
 *
 * This function checks the current cell voltages and temperatures against the configured setpoints
 * in the settings structure. It sets the appropriate bits in the `alarm` and `warning` arrays to
 * indicate overvoltage, undervoltage, overtemperature, undertemperature, and cell imbalance conditions.
 * These arrays are used for CAN communication and system status reporting.
 *
 * Alarm bits:
 * - alarm[0]:
 *   - 0x04: High cell voltage exceeds OverVSetpoint
 *   - 0x10: Low cell voltage below UnderVSetpoint
 *   - 0x40: High temperature exceeds OverTSetpoint
 * - alarm[1]:
 *   - 0x01: Low temperature below UnderTSetpoint
 * - alarm[3]:
 *   - 0x01: Cell voltage difference exceeds CellGap
 *
 * Warning bits:
 * - warning[0]:
 *   - 0x04: High cell voltage exceeds (OverVSetpoint - WarnOff)
 *   - 0x10: Low cell voltage below (UnderVSetpoint + WarnOff)
 *   - 0x40: High temperature exceeds (OverTSetpoint - WarnToff)
 * - warning[1]:
 *   - 0x01: Low temperature below (UnderTSetpoint + WarnToff)
 *
 * The function resets all alarm and warning bits before evaluating the current status.
 *
 * @note This function should be called periodically to ensure alarms and warnings are up-to-date.
 */
// TODO - appears to never be called?
void alarmupdate()
{
  alarm[0] = 0x00;
  if (settings.OverVSetpoint < bms.getHighCellVolt())
  {
    alarm[0] = 0x04;
  }
  if (bms.getLowCellVolt() < settings.UnderVSetpoint)
  {
    alarm[0] |= 0x10;
  }
  if (bms.getHighTemperature() > settings.OverTSetpoint)
  {
    alarm[0] |= 0x40;
  }
  alarm[1] = 0;
  if (bms.getLowTemperature() < settings.UnderTSetpoint)
  {
    alarm[1] = 0x01;
  }
  alarm[3] = 0;
  if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap)
  {
    alarm[3] = 0x01;
  }

  /// warnings///
  warning[0] = 0;

  if (bms.getHighCellVolt() > (settings.OverVSetpoint - settings.WarnOff))
  {
    warning[0] = 0x04;
  }
  if (bms.getLowCellVolt() < (settings.UnderVSetpoint + settings.WarnOff))
  {
    warning[0] |= 0x10;
  }

  if (bms.getHighTemperature() > (settings.OverTSetpoint - settings.WarnToff))
  {
    warning[0] |= 0x40;
  }
  warning[1] = 0;
  if (bms.getLowTemperature() < (settings.UnderTSetpoint + settings.WarnToff))
  {
    warning[1] = 0x01;
  }
}

void printbmsstat()
{
  Serial.print("\033[H\033[J");
  // SERIAL_CONSOLE.println();
  // SERIAL_CONSOLE.println();
  SERIAL_CONSOLE.println(testcount);
  SERIAL_CONSOLE.print("BMS Status : ");
  if (settings.ESSmode == 1)
  {
    SERIAL_CONSOLE.print("ESS Mode ");

    if (bms.getLowCellVolt() < settings.UnderVSetpoint)
    {
      SERIAL_CONSOLE.print(": UnderVoltage ");
    }
    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      SERIAL_CONSOLE.print(": OverVoltage ");
    }
    if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap)
    {
      SERIAL_CONSOLE.print(": Cell Imbalance ");
    }
    if (bms.getAvgTemperature() > settings.OverTSetpoint)
    {
      SERIAL_CONSOLE.print(": Over Temp ");
    }
    if (bms.getAvgTemperature() < settings.UnderTSetpoint)
    {
      SERIAL_CONSOLE.print(": Under Temp ");
    }
    if (storagemode == 1)
    {
      if (bms.getLowCellVolt() > settings.StoreVsetpoint)
      {
        SERIAL_CONSOLE.print(": OverVoltage Storage ");
        SERIAL_CONSOLE.print(": UNhappy:");
      }
      else
      {
        SERIAL_CONSOLE.print(": Happy ");
      }
    }
    else
    {
      if (bms.getLowCellVolt() > settings.UnderVSetpoint && bms.getHighCellVolt() < settings.OverVSetpoint)
      {

        if (bmsstatus == BMS_STATUS_ERROR)
        {
          SERIAL_CONSOLE.print(": UNhappy:");
        }
        else
        {
          SERIAL_CONSOLE.print(": Happy ");
        }
      }
    }
  }
  else
  {
    SERIAL_CONSOLE.print(bmsstatus);
    switch (bmsstatus)
    {
    case (BMS_STATUS_BOOT):
      SERIAL_CONSOLE.print(" Boot ");
      break;

    case (BMS_STATUS_READY):
      SERIAL_CONSOLE.print(" Ready ");
      break;

    case (BMS_STATUS_PRECHARGE):
      SERIAL_CONSOLE.print(" Precharge ");
      break;

    case (BMS_STATUS_DRIVE):
      SERIAL_CONSOLE.print(" Drive ");
      break;

    case (BMS_STATUS_CHARGE):
      SERIAL_CONSOLE.print(" Charge ");
      break;

    case (BMS_STATUS_ERROR):
      SERIAL_CONSOLE.print(" Error ");
      break;
    }
  }
  SERIAL_CONSOLE.print("  ");
  if (digitalRead(PIN_IN3) == HIGH)
  {
    SERIAL_CONSOLE.print("| AC Present |");
  }
  if (digitalRead(PIN_IN1) == HIGH)
  {
    SERIAL_CONSOLE.print("| Key ON |");
  }
  if (balancecells == 1)
  {
    SERIAL_CONSOLE.print("|Balancing Active");
  }
  SERIAL_CONSOLE.print("  ");
  SERIAL_CONSOLE.print(cellspresent);
  SERIAL_CONSOLE.println();
  SERIAL_CONSOLE.print("Out:");
  SERIAL_CONSOLE.print(digitalRead(PIN_OUT1));
  SERIAL_CONSOLE.print(digitalRead(PIN_OUT2));
  SERIAL_CONSOLE.print(digitalRead(PIN_OUT3));
  SERIAL_CONSOLE.print(digitalRead(PIN_OUT4));
  SERIAL_CONSOLE.print(" Cont:");
  if ((contstat & 1) == 1)
  {
    SERIAL_CONSOLE.print("1");
  }
  else
  {
    SERIAL_CONSOLE.print("0");
  }
  if ((contstat & 2) == 2)
  {
    SERIAL_CONSOLE.print("1");
  }
  else
  {
    SERIAL_CONSOLE.print("0");
  }
  if ((contstat & 4) == 4)
  {
    SERIAL_CONSOLE.print("1");
  }
  else
  {
    SERIAL_CONSOLE.print("0");
  }
  if ((contstat & 8) == 8)
  {
    SERIAL_CONSOLE.print("1");
  }
  else
  {
    SERIAL_CONSOLE.print("0");
  }
  SERIAL_CONSOLE.print(" In:");
  SERIAL_CONSOLE.print(digitalRead(PIN_IN1));
  SERIAL_CONSOLE.print(digitalRead(PIN_IN2));
  SERIAL_CONSOLE.print(digitalRead(PIN_IN3));
  SERIAL_CONSOLE.print(digitalRead(PIN_IN4));

  SERIAL_CONSOLE.print(" Charge Current Limit : ");
  SERIAL_CONSOLE.print(chargecurrent * 0.1, 0);
  SERIAL_CONSOLE.print(" A DisCharge Current Limit : ");
  SERIAL_CONSOLE.print(discurrent * 0.1, 0);
  SERIAL_CONSOLE.print(" A");

  if (bmsstatus == BMS_STATUS_CHARGE || accurlim > 0)
  {
    Serial.print("  CP AC Current Limit: ");
    Serial.print(accurlim);
    Serial.print(" A");
  }

  if (bmsstatus == BMS_STATUS_CHARGE && CPdebug == 1)
  {
    Serial.print("A  CP Dur: ");
    Serial.print(duration);
    Serial.print("  Charge Power : ");
    Serial.print(chargerpower);
    if (chargecurrentlimit == false)
    {
      SERIAL_CONSOLE.print("  No Charge Current Limit");
    }
    else
    {
      SERIAL_CONSOLE.print("  Charge Current Limit Active");
    }
  }
}

void updateSOC()
{
  if (SOCreset == 1)
  {
    SOC = map(uint16_t(bms.getLowCellVolt() * 1000), settings.socvolt[0], settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);
    ampsecond = (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778;
    SOCreset = 0;
  }

  if (SOCset == 0 && SOCmem == 0)
  {
    if (millis() > 5000)
    {
      SOC = map(uint16_t(bms.getLowCellVolt() * 1000), settings.socvolt[0], settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);

      ampsecond = (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778;
      SOCset = 1;
      if (debug != 0)
      {
        SERIAL_CONSOLE.println("  ");
        SERIAL_CONSOLE.println("//////////////////////////////////////// SOC SET ////////////////////////////////////////");
      }
      if (settings.ESSmode == 1)
      {
        // bmsstatus = BMS_STATUS_READY;
        setBMSstatus(BMS_STATUS_READY, "SOC initialized from voltage after 5 seconds in ESS mode");
      }
    }
  }

  if (SOCset == 0 && SOCmem == 1)
  {
    ampsecond = (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778;
    if (millis() > 5000)
    {
      SOCset = 1;
      if (debug != 0)
      {
        SERIAL_CONSOLE.println("  ");
        SERIAL_CONSOLE.println("//////////////////////////////////////// SOC SET ////////////////////////////////////////");
      }
      if (settings.ESSmode == 1)
      {
        // bmsstatus = BMS_STATUS_READY;
        setBMSstatus(BMS_STATUS_READY, "SOC initialized from memory after 5 seconds in ESS mode");
      }
    }
  }

  SOC = ((ampsecond * 0.27777777777778) / (settings.CAP * settings.Pstrings * 1000)) * 100;

  if (settings.voltsoc == 1 || settings.cursens == 0)
  {
    SOC = map(uint16_t(bms.getLowCellVolt() * 1000), settings.socvolt[0], settings.socvolt[2], settings.socvolt[1], settings.socvolt[3]);

    ampsecond = (SOC * settings.CAP * settings.Pstrings * 10) / 0.27777777777778;
  }

  if (SOC >= 100)
  {
    ampsecond = (settings.CAP * settings.Pstrings * 1000) / 0.27777777777778; // reset to full, dependant on given capacity. Need to improve with auto correction for capcity.
    SOC = 100;
  }

  if (SOC < 0)
  {
    SOC = 0; // reset SOC this way the can messages remain in range for other devices. Ampseconds will keep counting.
  }

  if (debug != 0)
  {
    if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL)
    {
      if (sensor == 1)
      {
        SERIAL_CONSOLE.print("Low Range ");
      }
      else
      {
        SERIAL_CONSOLE.print("High Range");
      }
    }
    if (settings.cursens == CURR_SENSE_ANALOGUE_GUESSING)
    {
      SERIAL_CONSOLE.print("Analogue Single ");
    }
    if (settings.cursens == CURR_SENSE_CANBUS)
    {
      SERIAL_CONSOLE.print("CANbus ");
    }
    SERIAL_CONSOLE.print("  ");
    SERIAL_CONSOLE.print(currentact);
    SERIAL_CONSOLE.print("mA");
    SERIAL_CONSOLE.print("  ");
    SERIAL_CONSOLE.print(SOC);
    SERIAL_CONSOLE.print("% SOC ");
    SERIAL_CONSOLE.print(ampsecond * 0.27777777777778, 2);
    SERIAL_CONSOLE.print("mAh");
  }
}

void SOCcharged(int y)
{
  if (y == 1)
  {
    SOC = 95;
    ampsecond = (settings.CAP * settings.Pstrings * 1000) / 0.27777777777778; // reset to full, dependant on given capacity. Need to improve with auto correction for capcity.
  }
  if (y == 2)
  {
    SOC = 100;
    ampsecond = (settings.CAP * settings.Pstrings * 1000) / 0.27777777777778; // reset to full, dependant on given capacity. Need to improve with auto correction for capcity.
  }
}

void Prechargecon()
{
  if (digitalRead(PIN_IN1) == HIGH || digitalRead(PIN_IN3) == HIGH) // detect Key ON or AC present
  {
    digitalWrite(PIN_OUT4, HIGH); // Negative Contactor Close
    contctrl = 2;
    if (Pretimer + settings.Pretime > millis() || currentact > settings.Precurrent)
    {
      digitalWrite(PIN_OUT2, HIGH); // precharge
    }
    else // close main contactor
    {
      digitalWrite(PIN_OUT1, HIGH); // Positive Contactor Close
      contctrl = 3;
      if (settings.ChargerDirect == 1)
      {
        // bmsstatus = BMS_STATUS_DRIVE;
        setBMSstatus(BMS_STATUS_DRIVE, "ChargerDirect=1 so going to DRIVE state after precharge");
      }
      else
      {
        if (digitalRead(PIN_IN3) == HIGH)
        {
          // bmsstatus = BMS_STATUS_CHARGE;
          setBMSstatus(BMS_STATUS_CHARGE, "AC detected so going to CHARGE state after precharge");
        }
        if (digitalRead(PIN_IN1) == HIGH)
        {
          // bmsstatus = BMS_STATUS_DRIVE;
          setBMSstatus(BMS_STATUS_DRIVE, "Key ON detected so going to DRIVE state after precharge");
        }
      }
      digitalWrite(PIN_OUT2, LOW);
    }
  }
  else
  {
    digitalWrite(PIN_OUT1, LOW);
    digitalWrite(PIN_OUT2, LOW);
    digitalWrite(PIN_OUT4, LOW);
    // bmsstatus = BMS_STATUS_READY;
    setBMSstatus(BMS_STATUS_READY, "Key OFF and AC not detected during PRECHARGE state");
    contctrl = 0;
  }
}

// Contactor Control Function
void contactorControl()
{
  if (contctrl != contstat) // check for contactor request change
  {
    if ((contctrl & 1) == 0)
    {
      analogWrite(PIN_OUT5, 0);
      contstat = contstat & 254;
    }
    if ((contctrl & 2) == 0)
    {
      analogWrite(PIN_OUT6, 0);
      contstat = contstat & 253;
    }
    if ((contctrl & 4) == 0)
    {
      analogWrite(PIN_OUT7, 0);
      contstat = contstat & 251;
    }

    if ((contctrl & 1) == 1)
    {
      if ((contstat & 1) != 1)
      {
        if (conttimer1 == 0)
        {
          analogWrite(PIN_OUT5, 255);
          conttimer1 = millis() + pulltime;
        }
        if (conttimer1 < millis())
        {
          analogWrite(PIN_OUT5, settings.conthold);
          contstat = contstat | 1;
          conttimer1 = 0;
        }
      }
    }

    if ((contctrl & 2) == 2)
    {
      if ((contstat & 2) != 2)
      {
        if (conttimer2 == 0)
        {
          if (debug != 0)
          {
            Serial.println();
            Serial.println("pull in OUT6");
          }
          analogWrite(PIN_OUT6, 255);
          conttimer2 = millis() + pulltime;
        }
        if (conttimer2 < millis())
        {
          analogWrite(PIN_OUT6, settings.conthold);
          contstat = contstat | 2;
          conttimer2 = 0;
        }
      }
    }
    if ((contctrl & 4) == 4)
    {
      if ((contstat & 4) != 4)
      {
        if (conttimer3 == 0)
        {
          if (debug != 0)
          {
            Serial.println();
            Serial.println("pull in OUT7");
          }
          analogWrite(PIN_OUT7, 255);
          conttimer3 = millis() + pulltime;
        }
        if (conttimer3 < millis())
        {
          analogWrite(PIN_OUT7, settings.conthold);
          contstat = contstat | 4;
          conttimer3 = 0;
        }
      }
    }
    /*
       SERIAL_CONSOLE.print(conttimer);
       SERIAL_CONSOLE.print("  ");
       SERIAL_CONSOLE.print(contctrl);
       SERIAL_CONSOLE.print("  ");
       SERIAL_CONSOLE.print(contstat);
       SERIAL_CONSOLE.println("  ");
    */
  }
  if (contctrl == 0)
  {
    analogWrite(PIN_OUT5, 0);
    analogWrite(PIN_OUT6, 0);
  }
}

void VEcan() // communication with Victron system over CAN
{
  msg.id = 0x351;
  msg.len = 8;
  if (storagemode == 0)
  {
    msg.buf[0] = lowByte(uint16_t((settings.ChargeVsetpoint * settings.Scells) * 10));
    msg.buf[1] = highByte(uint16_t((settings.ChargeVsetpoint * settings.Scells) * 10));
  }
  else
  {
    msg.buf[0] = lowByte(uint16_t((settings.StoreVsetpoint * settings.Scells) * 10));
    msg.buf[1] = highByte(uint16_t((settings.StoreVsetpoint * settings.Scells) * 10));
  }
  msg.buf[2] = lowByte(chargecurrent);
  msg.buf[3] = highByte(chargecurrent);
  msg.buf[4] = lowByte(discurrent);
  msg.buf[5] = highByte(discurrent);
  msg.buf[6] = lowByte(uint16_t((settings.DischVsetpoint * settings.Scells) * 10));
  msg.buf[7] = highByte(uint16_t((settings.DischVsetpoint * settings.Scells) * 10));
  Can1.write(msg);

  msg.id = 0x355;
  msg.len = 8;
  msg.buf[0] = lowByte(SOC);
  msg.buf[1] = highByte(SOC);
  msg.buf[2] = lowByte(SOH);
  msg.buf[3] = highByte(SOH);
  msg.buf[4] = lowByte(SOC * 10);
  msg.buf[5] = highByte(SOC * 10);
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  Can1.write(msg);

  msg.id = 0x356;
  msg.len = 8;

  if (settings.chargertype == VictronHV || settings.SerialCan == 1)
  {
    msg.buf[0] = lowByte(uint16_t(bms.getPackVoltage() * 10));
    msg.buf[1] = highByte(uint16_t(bms.getPackVoltage() * 10));
  }
  else
  {
    msg.buf[0] = lowByte(uint16_t(bms.getPackVoltage() * 100));
    msg.buf[1] = highByte(uint16_t(bms.getPackVoltage() * 100));
  }

  msg.buf[2] = lowByte(long(currentact / 100));
  msg.buf[3] = highByte(long(currentact / 100));
  msg.buf[4] = lowByte(int16_t(bms.getAvgTemperature() * 10));
  msg.buf[5] = highByte(int16_t(bms.getAvgTemperature() * 10));
  msg.buf[6] = 0;
  msg.buf[7] = 0;
  Can1.write(msg);

  delay(2);
  msg.id = 0x35A;
  msg.len = 8;
  msg.buf[0] = alarm[0];   // High temp  Low Voltage | High Voltage
  msg.buf[1] = alarm[1];   // High Discharge Current | Low Temperature
  msg.buf[2] = alarm[2];   // Internal Failure | High Charge current
  msg.buf[3] = alarm[3];   // Cell Imbalance
  msg.buf[4] = warning[0]; // High temp  Low Voltage | High Voltage
  msg.buf[5] = warning[1]; // High Discharge Current | Low Temperature
  msg.buf[6] = warning[2]; // Internal Failure | High Charge current
  msg.buf[7] = warning[3]; // Cell Imbalance
  Can1.write(msg);

  msg.id = 0x35E;
  msg.len = 8;
  msg.buf[0] = bmsname[0];
  msg.buf[1] = bmsname[1];
  msg.buf[2] = bmsname[2];
  msg.buf[3] = bmsname[3];
  msg.buf[4] = bmsname[4];
  msg.buf[5] = bmsname[5];
  msg.buf[6] = bmsname[6];
  msg.buf[7] = bmsname[7];
  Can1.write(msg);

  delay(2);
  msg.id = 0x370;
  msg.len = 8;
  msg.buf[0] = bmsmanu[0];
  msg.buf[1] = bmsmanu[1];
  msg.buf[2] = bmsmanu[2];
  msg.buf[3] = bmsmanu[3];
  msg.buf[4] = bmsmanu[4];
  msg.buf[5] = bmsmanu[5];
  msg.buf[6] = bmsmanu[6];
  msg.buf[7] = bmsmanu[7];
  Can1.write(msg);

  delay(2);
  msg.id = 0x373;
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(bms.getLowCellVolt() * 1000));
  msg.buf[1] = highByte(uint16_t(bms.getLowCellVolt() * 1000));
  msg.buf[2] = lowByte(uint16_t(bms.getHighCellVolt() * 1000));
  msg.buf[3] = highByte(uint16_t(bms.getHighCellVolt() * 1000));
  msg.buf[4] = lowByte(uint16_t(bms.getLowTemperature() + 273.15));
  msg.buf[5] = highByte(uint16_t(bms.getLowTemperature() + 273.15));
  msg.buf[6] = lowByte(uint16_t(bms.getHighTemperature() + 273.15));
  msg.buf[7] = highByte(uint16_t(bms.getHighTemperature() + 273.15));
  Can1.write(msg);

  delay(2);
  msg.id = 0x379; // Installed capacity
  msg.len = 8;
  msg.buf[0] = lowByte(uint16_t(settings.Pstrings * settings.CAP));
  msg.buf[1] = highByte(uint16_t(settings.Pstrings * settings.CAP));
  msg.buf[2] = contstat; // contactor state
  msg.buf[3] = (digitalRead(PIN_OUT1) | (digitalRead(PIN_OUT2) << 1) | (digitalRead(PIN_OUT3) << 2) | (digitalRead(PIN_OUT4) << 3));
  msg.buf[4] = bmsstatus;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  Can1.write(msg);
  /*
      delay(2);
    msg.id  = 0x378; //Installed capacity
    msg.len = 2;
    //energy in 100wh/unit
    msg.buf[0] =
    msg.buf[1] =
    msg.buf[2] =
    msg.buf[3] =
    //energy out 100wh/unit
    msg.buf[4] =
    msg.buf[5] =
    msg.buf[6] =
    msg.buf[7] =
  */
  delay(2);

  msg.id = 0x372;
  msg.len = 8;
  msg.buf[0] = lowByte(bms.getNumModules());
  msg.buf[1] = highByte(bms.getNumModules());
  msg.buf[2] = 0x00;
  msg.buf[3] = 0x00;
  msg.buf[4] = 0x00;
  msg.buf[5] = 0x00;
  msg.buf[6] = 0x00;
  msg.buf[7] = 0x00;
  Can1.write(msg);
}

int pgnFromCANId(int canId)
{
  if ((canId & 0x10000000) == 0x10000000)
  {
    return (canId & 0x03FFFF00) >> 8;
  }
  else
  {
    return canId; // not sure if this is really right?
  }
}

bool canRead()
{
  if (Can1.read(inMsg))
  {
    // Read data: len = data length, buf = data byte(s)
    if (settings.cursens == CURR_SENSE_CANBUS)
    {
      if (settings.curcan == 1)
      {
        switch (inMsg.id)
        {
        case 0x3c0:
          CAB300();
          break;

        case 0x3c1:
          CAB300();
          break;

        case 0x3c2:
          CAB300();
          break;

        default:
          break;
        }
      }
      if (settings.curcan == 2)
      {
        switch (inMsg.id)
        {
        case 0x3c0:
          CAB500();
          break;

        case 0x3c1:
          CAB500();
          break;

        case 0x3c2:
          CAB500();
          break;

        default:
          break;
        }
      }
      if (settings.curcan == 3)
      {
        switch (inMsg.id)
        {
        case 0x521: //
          CANmilliamps = (long)((inMsg.buf[2] << 24) | (inMsg.buf[3] << 16) | (inMsg.buf[4] << 8) | (inMsg.buf[5]));
          if (settings.cursens == CURR_SENSE_CANBUS)
          {
            RawCur = CANmilliamps;
            getcurrent();
          }
          break;
        case 0x3C3: // Jaguar Ipace ISA shunt current reading
          CANmilliamps = inMsg.buf[5] + (inMsg.buf[4] << 8) + (inMsg.buf[3] << 16) + (inMsg.buf[2] << 24);
          if (settings.cursens == CURR_SENSE_CANBUS)
          {
            RawCur = CANmilliamps;
            getcurrent();
          }
          break;

        case 0x522: //
          voltage1 = (long)((inMsg.buf[2] << 24) | (inMsg.buf[3] << 16) | (inMsg.buf[4] << 8) | (inMsg.buf[5]));
          break;
        case 0x523: //
          voltage2 = (long)((inMsg.buf[2] << 24) | (inMsg.buf[3] << 16) | (inMsg.buf[4] << 8) | (inMsg.buf[5]));
          break;
        default:
          break;
        }
      }
      if (settings.curcan == 4)
      {
        if (pgnFromCANId(inMsg.id) == 0x1F214 && inMsg.buf[0] == 0) // Check PGN and only use the first packet of each sequence
        {
          handleVictronLynx();
        }
      }
    }

    if (inMsg.id == 0x309)
    {
      Rx309();
    }

    if (debug == 1)
    {
      if (candebug == 1)
      {
        Serial.print(millis());
        if ((inMsg.id & 0x80000000) == 0x80000000) // Determine if ID is standard (11 bits) or extended (29 bits)
          sprintf(msgString, "Extended ID : 0x%.8lX  DLC : % 1d  Data : ", (inMsg.id & 0x1FFFFFFF), inMsg.len);
        else
          sprintf(msgString, ", 0x%.3lX, false, % 1d", inMsg.id, inMsg.len);

        Serial.print(msgString);

        if ((inMsg.id & 0x40000000) == 0x40000000)
        { // Determine if message is a remote request frame.
          sprintf(msgString, " REMOTE REQUEST FRAME");
          Serial.print(msgString);
        }
        else
        {
          for (byte i = 0; i < inMsg.len; i++)
          {
            sprintf(msgString, ", 0x%.2X", inMsg.buf[i]);
            Serial.print(msgString);
          }
        }

        Serial.println();
      }
    }
  }
  else
  {
    // No Message
    return 0;
  }
  // returns true as it's read a message, now check again.
  return 1;
}

void Rx309()
{
  if (SOCset == 1)
  {
    if (inMsg.buf[0] & 0x01)
    {
      CanOnReq = true;
      CanOnRev = true;
      CanOntimeout = millis();
    }
    else
    {
      CanOnReq = false;
      CanOnRev = true;
      CanOntimeout = millis();
    }
  }
}

void CAB300()
{
  for (int i = 0; i < 4; i++)
  {
    inbox = (inbox << 8) | inMsg.buf[i];
  }
  // CANmilliamps = inbox;
  if (inbox > 0x80000000)
  {
    CANmilliamps = inbox - 0x80000000;
  }
  else
  {
    CANmilliamps = (0x80000000 - inbox) * -1;
  }
  if (settings.cursens == CURR_SENSE_CANBUS)
  {
    RawCur = CANmilliamps;
    getcurrent();
  }
  if (candebug == 1)
  {
    Serial.println();
    Serial.print(CANmilliamps);
    Serial.print("mA ");
  }
}

void CAB500()
{
  inbox = 0;
  for (int i = 1; i < 4; i++)
  {
    inbox = (inbox << 8) | inMsg.buf[i];
  }
  CANmilliamps = inbox;
  if (candebug == 1)
  {
    Serial.println();
    Serial.print(CANmilliamps, HEX);
  }
  if (CANmilliamps > 0x800000)
  {
    CANmilliamps -= 0x800000;
  }
  else
  {
    CANmilliamps = (0x800000 - CANmilliamps) * -1;
  }
  if (settings.cursens == CURR_SENSE_CANBUS)
  {
    RawCur = CANmilliamps;
    getcurrent();
  }
  if (candebug == 1)
  {
    Serial.println();
    Serial.print(CANmilliamps);
    Serial.print("mA ");
  }
}

void handleVictronLynx()
{
  if (inMsg.buf[4] == 0xff && inMsg.buf[3] == 0xff)
    return;
  int16_t current = (int)inMsg.buf[4] << 8; // in 0.1A increments
  current |= inMsg.buf[3];
  CANmilliamps = current * 100;
  if (settings.cursens == CURR_SENSE_CANBUS)
  {
    RawCur = CANmilliamps;
    getcurrent();
  }
  if (candebug == 1)
  {
    Serial.println();
    Serial.print(CANmilliamps);
    Serial.print("mA ");
  }
}

void currentlimit()
{
  if (bmsstatus == BMS_STATUS_ERROR)
  {
    discurrent = 0;
    chargecurrent = 0;
  }
  /*
    settings.PulseCh = 600; //Peak Charge current in 0.1A
    settings.PulseChDur = 5000; //Ms of discharge pulse derating
    settings.PulseDi = 600; //Peak Charge current in 0.1A
    settings.PulseDiDur = 5000; //Ms of discharge pulse derating
  */
  else
  {

    /// Start at no derating///
    discurrent = settings.discurrentmax;

    if (chargecurrentlimit == false)
    {
      chargecurrent = settings.chargecurrentmax;
    }
    else
    {
      chargecurrent = settings.chargecurrent2max;
    }

    ///////All hard limits to into zeros
    if (bms.getLowTemperature() < settings.UnderTSetpoint)
    {
      // discurrent = 0; Request Daniel
      chargecurrent = 0;
    }
    if (bms.getHighTemperature() > settings.OverTSetpoint)
    {
      discurrent = 0;
      chargecurrent = 0;
    }
    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      chargecurrent = 0;
    }
    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      chargecurrent = 0;
    }
    if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getLowCellVolt() < settings.DischVsetpoint)
    {
      discurrent = 0;
    }

    // Modifying discharge current///

    if (discurrent > 0)
    {
      // Temperature based///

      if (bms.getHighTemperature() > settings.DisTSetpoint)
      {
        discurrent = discurrent - map(bms.getHighTemperature(), settings.DisTSetpoint, settings.OverTSetpoint, 0, settings.discurrentmax);
      }
      // Voltagee based///
      if (bms.getLowCellVolt() < (settings.DischVsetpoint + settings.DisTaper))
      {
        discurrent = discurrent - map(bms.getLowCellVolt(), settings.DischVsetpoint, (settings.DischVsetpoint + settings.DisTaper), settings.discurrentmax, 0);
      }
    }

    // Modifying Charge current///

    if (chargecurrent > 0)
    {
      if (chargecurrentlimit == false)
      {
        // Temperature based///
        if (bms.getLowTemperature() < settings.ChargeTSetpoint)
        {
          chargecurrent = chargecurrent - map(bms.getLowTemperature(), settings.UnderTSetpoint, settings.ChargeTSetpoint, settings.chargecurrentmax, 0);
        }
        // Voltagee based///
        if (storagemode == 1)
        {
          if (bms.getHighCellVolt() > (settings.StoreVsetpoint - settings.ChargeHys))
          {
            chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.StoreVsetpoint - settings.ChargeHys), settings.StoreVsetpoint, settings.chargecurrentend, settings.chargecurrentmax);
          }
        }
        else
        {
          if (bms.getHighCellVolt() > (settings.ChargeVsetpoint - settings.ChargeHys))
          {
            chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.ChargeVsetpoint - settings.ChargeHys), settings.ChargeVsetpoint, 0, (settings.chargecurrentmax - settings.chargecurrentend));
          }
        }
      }
      else
      {
        // Temperature based///
        if (bms.getLowTemperature() < settings.ChargeTSetpoint)
        {
          chargecurrent = chargecurrent - map(bms.getLowTemperature(), settings.UnderTSetpoint, settings.ChargeTSetpoint, settings.chargecurrent2max, 0);
        }
        // Voltagee based///
        if (storagemode == 1)
        {
          if (bms.getHighCellVolt() > (settings.StoreVsetpoint - settings.ChargeHys))
          {
            chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.StoreVsetpoint - settings.ChargeHys), settings.StoreVsetpoint, settings.chargecurrentend, settings.chargecurrent2max);
          }
        }
        else
        {
          if (bms.getHighCellVolt() > (settings.ChargeVsetpoint - settings.ChargeHys))
          {
            chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.ChargeVsetpoint - settings.ChargeHys), settings.ChargeVsetpoint, 0, (settings.chargecurrent2max - settings.chargecurrentend));
          }
        }
      }
    }
  }
  /// No negative currents///

  if (discurrent < 0)
  {
    discurrent = 0;
  }
  if (chargecurrent < 0)
  {
    chargecurrent = 0;
  }

  // Charge current derate for Control Pilot AC limit

  if (accurlim > 0)
  {
    chargerpower = accurlim * settings.chargerACv * settings.chargereff * 0.01;
    tempchargecurrent = (chargerpower * 10) / (bms.getAvgCellVolt() * settings.Scells);

    if (chargecurrent > tempchargecurrent)
    {
      chargecurrent = tempchargecurrent;
    }
  }
}

void inputdebug()
{
  Serial.println();
  Serial.print("Input : ");
  if (digitalRead(PIN_IN1))
  {
    Serial.print("1 ON  ");
  }
  else
  {
    Serial.print("1 OFF ");
  }
  if (digitalRead(PIN_IN3))
  {
    Serial.print("2 ON  ");
  }
  else
  {
    Serial.print("2 OFF ");
  }
  if (digitalRead(PIN_IN3))
  {
    Serial.print("3 ON  ");
  }
  else
  {
    Serial.print("3 OFF ");
  }
  if (digitalRead(PIN_IN4))
  {
    Serial.print("4 ON  ");
  }
  else
  {
    Serial.print("4 OFF ");
  }
  Serial.println();
}

void outputdebug()
{
  if (outputstate < 5)
  {
    digitalWrite(PIN_OUT1, HIGH);
    digitalWrite(PIN_OUT2, HIGH);
    digitalWrite(PIN_OUT3, HIGH);
    digitalWrite(PIN_OUT4, HIGH);
    analogWrite(PIN_OUT5, 255);
    analogWrite(PIN_OUT6, 255);
    analogWrite(PIN_OUT7, 255);
    analogWrite(PIN_OUT8, 255);
    outputstate++;
  }
  else
  {
    digitalWrite(PIN_OUT1, LOW);
    digitalWrite(PIN_OUT2, LOW);
    digitalWrite(PIN_OUT3, LOW);
    digitalWrite(PIN_OUT4, LOW);
    analogWrite(PIN_OUT5, 0);
    analogWrite(PIN_OUT6, 0);
    analogWrite(PIN_OUT7, 0);
    analogWrite(PIN_OUT8, 0);
    outputstate++;
  }
  if (outputstate > 10)
  {
    outputstate = 0;
  }
}

void resetwdog()
{
  // noInterrupts();                                     //   No - reset WDT
  // WDOG_REFRESH = 0xA602; Teensy 3.1
  // WDOG_REFRESH = 0xB480;
  // interrupts();
  watchdog.feed();
}

void pwmcomms()
{
  int p = 0;
  p = map((currentact * 0.001), pwmcurmin, pwmcurmax, 50, 255);
  analogWrite(PIN_OUT7, p);
  /*
    Serial.println();
    Serial.print(p*100/255);
    Serial.print(" OUT8 ");
  */

  if (bms.getLowCellVolt() < settings.UnderVSetpoint)
  {
    analogWrite(PIN_OUT7, 255); // 12V to 10V converter 1.5V
  }
  else
  {
    p = map(SOC, 0, 100, 220, 50);
    analogWrite(PIN_OUT8, p); // 2V to 10V converter 1.5-10V
  }
  /*
    Serial.println();
    Serial.print(p);
    Serial.print(" OUT7 ");
  */
}

void dashupdate()
{
  Serial2.write("stat.txt=");
  Serial2.write(0x22);
  if (settings.ESSmode == 1)
  {
    switch (bmsstatus)
    {
    case (BMS_STATUS_BOOT):
      Serial2.print(" Active ");
      break;
    case (BMS_STATUS_ERROR):
      Serial2.print(" Error ");
      break;
    }
  }
  else
  {
    switch (bmsstatus)
    {
    case (BMS_STATUS_BOOT):
      Serial2.print(" Boot ");
      break;

    case (BMS_STATUS_READY):
      Serial2.print(" Ready ");
      break;

    case (BMS_STATUS_PRECHARGE):
      Serial2.print(" Precharge ");
      break;

    case (BMS_STATUS_DRIVE):
      Serial2.print(" Drive ");
      break;

    case (BMS_STATUS_CHARGE):
      Serial2.print(" Charge ");
      break;

    case (BMS_STATUS_ERROR):
      Serial2.print(" Error ");
      break;
    }
  }
  Serial2.write(0x22);
  Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("soc.val=");
  Serial2.print(SOC);
  Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("soc1.val=");
  Serial2.print(SOC);
  Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("current.val=");
  Serial2.print(currentact / 100, 0);
  Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("temp.val=");
  Serial2.print(bms.getAvgTemperature(), 0);
  Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("templow.val=");
  Serial2.print(bms.getLowTemperature(), 0);
  Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("temphigh.val=");
  Serial2.print(bms.getHighTemperature(), 0);
  Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("volt.val=");
  Serial2.print(bms.getPackVoltage() * 10, 0);
  Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("lowcell.val=");
  Serial2.print(bms.getLowCellVolt() * 1000, 0);
  Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("highcell.val=");
  Serial2.print(bms.getHighCellVolt() * 1000, 0);
  Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("firm.val=");
  Serial2.print(firmver);
  Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("celldelta.val=");
  Serial2.print((bms.getHighCellVolt() - bms.getLowCellVolt()) * 1000, 0);
  Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("cellbal.val=");
  Serial2.print(bms.getBalancing());
  Serial2.write(0xff); // We always have to send this three lines after each command sent to the nextion display.
  Serial2.write(0xff);
  Serial2.write(0xff);
}

void balancing()
{
  if (balancecells == 1)
  {
    if (debug == 1)
    {
      bms.balanceCells(settings.balanceDuty, 0);
    }
    else
    {
      bms.balanceCells(settings.balanceDuty, 0);
    }
  }
  else
  {
    bms.StopBalancing();
  }
}

void chargercomms()
{

  if (settings.chargertype == Elcon)
  {
    msg.id = 0x1806E5F4; // broadcast to all Elteks
    msg.len = 8;
    msg.flags.extended = 1;
    msg.buf[0] = highByte(uint16_t(settings.ChargeVsetpoint * settings.Scells * 10));
    msg.buf[1] = lowByte(uint16_t(settings.ChargeVsetpoint * settings.Scells * 10));
    msg.buf[2] = highByte(chargecurrent / ncharger);
    msg.buf[3] = lowByte(chargecurrent / ncharger);
    msg.buf[4] = 0x00;
    msg.buf[5] = 0x00;
    msg.buf[6] = 0x00;
    msg.buf[7] = 0x00;

    Can1.write(msg);
    msg.flags.extended = 0;
  }

  if (settings.chargertype == Eltek)
  {
    msg.id = 0x2FF; // broadcast to all Elteks
    msg.len = 7;
    msg.buf[0] = 0x01;
    msg.buf[1] = lowByte(1000);
    msg.buf[2] = highByte(1000);
    msg.buf[3] = lowByte(uint16_t(settings.ChargeVsetpoint * settings.Scells * 10));
    msg.buf[4] = highByte(uint16_t(settings.ChargeVsetpoint * settings.Scells * 10));
    msg.buf[5] = lowByte(chargecurrent / ncharger);
    msg.buf[6] = highByte(chargecurrent / ncharger);

    Can1.write(msg);
  }
  if (settings.chargertype == BrusaNLG5)
  {
    msg.id = chargerid1;
    msg.len = 7;
    msg.buf[0] = 0x80;
    /*
      if (chargertoggle == 0)
      {
      msg.buf[0] = 0x80;
      chargertoggle++;
      }
      else
      {
      msg.buf[0] = 0xC0;
      chargertoggle = 0;
      }
    */
    if (digitalRead(PIN_IN2) == LOW) // Gen OFF
    {
      msg.buf[1] = highByte(maxac1 * 10);
      msg.buf[2] = lowByte(maxac1 * 10);
    }
    else
    {
      msg.buf[1] = highByte(maxac2 * 10);
      msg.buf[2] = lowByte(maxac2 * 10);
    }
    msg.buf[5] = highByte(chargecurrent / ncharger);
    msg.buf[6] = lowByte(chargecurrent / ncharger);
    msg.buf[3] = highByte(uint16_t(((settings.ChargeVsetpoint * settings.Scells) - chargerendbulk) * 10));
    msg.buf[4] = lowByte(uint16_t(((settings.ChargeVsetpoint * settings.Scells) - chargerendbulk) * 10));
    Can1.write(msg);

    delay(2);

    msg.id = chargerid2;
    msg.len = 7;
    msg.buf[0] = 0x80;
    if (digitalRead(PIN_IN2) == LOW) // Gen OFF
    {
      msg.buf[1] = highByte(maxac1 * 10);
      msg.buf[2] = lowByte(maxac1 * 10);
    }
    else
    {
      msg.buf[1] = highByte(maxac2 * 10);
      msg.buf[2] = lowByte(maxac2 * 10);
    }
    msg.buf[3] = highByte(uint16_t(((settings.ChargeVsetpoint * settings.Scells) - chargerend) * 10));
    msg.buf[4] = lowByte(uint16_t(((settings.ChargeVsetpoint * settings.Scells) - chargerend) * 10));
    msg.buf[5] = highByte(chargecurrent / ncharger);
    msg.buf[6] = lowByte(chargecurrent / ncharger);
    Can1.write(msg);
  }
  if (settings.chargertype == ChevyVolt)
  {
    msg.id = 0x30E;
    msg.len = 1;
    msg.buf[0] = 0x02; // only HV charging , 0x03 hv and 12V charging
    Can1.write(msg);

    msg.id = 0x304;
    msg.len = 4;
    msg.buf[0] = 0x40; // fixed
    if ((chargecurrent * 2) > 255)
    {
      msg.buf[1] = 255;
    }
    else
    {
      msg.buf[1] = (chargecurrent * 2);
    }
    if ((settings.ChargeVsetpoint * settings.Scells) > 200)
    {
      msg.buf[2] = highByte(uint16_t((settings.ChargeVsetpoint * settings.Scells) * 2));
      msg.buf[3] = lowByte(uint16_t((settings.ChargeVsetpoint * settings.Scells) * 2));
    }
    else
    {
      msg.buf[2] = highByte(400);
      msg.buf[3] = lowByte(400);
    }
    Can1.write(msg);
  }

  if (settings.chargertype == Coda)
  {
    msg.id = 0x050;
    msg.len = 8;
    msg.buf[0] = 0x00;
    msg.buf[1] = 0xDC;
    if ((settings.ChargeVsetpoint * settings.Scells) > 200)
    {
      msg.buf[2] = highByte(uint16_t((settings.ChargeVsetpoint * settings.Scells) * 10));
      msg.buf[3] = lowByte(uint16_t((settings.ChargeVsetpoint * settings.Scells) * 10));
    }
    else
    {
      msg.buf[2] = highByte(400);
      msg.buf[3] = lowByte(400);
    }
    msg.buf[4] = 0x00;
    if ((settings.ChargeVsetpoint * settings.Scells) * chargecurrent < 3300)
    {
      msg.buf[5] = highByte(uint16_t(((settings.ChargeVsetpoint * settings.Scells) * chargecurrent) / 240));
      msg.buf[6] = highByte(uint16_t(((settings.ChargeVsetpoint * settings.Scells) * chargecurrent) / 240));
    }
    else // 15 A AC limit
    {
      msg.buf[5] = 0x00;
      msg.buf[6] = 0x96;
    }
    msg.buf[7] = 0x01; // HV charging
    Can1.write(msg);
  }
}

void isrCP()
{
  if (digitalRead(PIN_IN4) == LOW)
  {
    duration = micros() - pilottimer;
    pilottimer = micros();
  }
  else
  {
    accurlim = ((duration - (micros() - pilottimer + 35)) * 60) / duration; // pilottimer + "xx" optocoupler decade ms
  }
} // ******** end of isr CP ********

// void low_voltage_isr(void)
// {
//   EEPROM.update(1000, uint8_t(SOC));

//   PMC_LVDSC2 |= PMC_LVDSC2_LVWACK; // clear if we can
//   PMC_LVDSC1 |= PMC_LVDSC1_LVDACK;
// }
