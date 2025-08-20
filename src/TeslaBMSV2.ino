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
#include "Application_Settings.h"
#include "indicators.h"
#include "BMSModuleManager.h"
#include "config.h"
#include "SerialConsole.h"
#include "Logger.h"

// Libraries
#include <Arduino.h>
#include <ADC.h> //https://github.com/pedvide/ADC
#include <EEPROM.h>
#include <FlexCAN_T4.h> //https://github.com/collin80/FlexCAN_Library
#include <SPI.h>
#include <Filters.h> //https://github.com/JonHub/Filters
#include <Serial_CAN_Module_Teensy.h> //https://github.com/tomdebree/Serial_CAN_Teensy

// T4 Additions
#include <Watchdog_t4.h>
#include <imxrt.h>
#include <CrashReport.h>



/////Version Identifier/////////
int firmver = 230719; // Year Month Day
const char* COMPILE_DATE = __DATE__;
const char* COMPILE_TIME = __TIME__;

WDT_T4<WDT1> watchdog;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

Serial_CAN can;
BMSModuleManager bms;
SerialConsole console;
EEPROMSettings settings;

// Curent filter//
float filterFrequency = 5.0;
FilterOnePole lowpassFilter(LOWPASS, filterFrequency);

ADC *adc = new ADC(); // adc object

CAN_message_t msg;
CAN_message_t inMsg;



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
  pinMode(PIN_IN1, INPUT);
  pinMode(PIN_IN2, INPUT);
  pinMode(PIN_IN3, INPUT);
  pinMode(PIN_IN4, INPUT);
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

  SERIALCONSOLE.begin(115200);
  SERIALCONSOLE.println("Starting up!");
  SERIALCONSOLE.println("SimpBMS V2 Tesla");

  Serial2.begin(115200); // display and can adpater canbus
  
  // Teensy 3.1
  // // Display reason the Teensy was last reset
  // Serial.println();
  // Serial.println("Reason for last Reset: ");

  // if (RCM_SRS1 & RCM_SRS1_SACKERR)   Serial.println("Stop Mode Acknowledge Error Reset");
  // if (RCM_SRS1 & RCM_SRS1_MDM_AP)    Serial.println("MDM-AP Reset");
  // if (RCM_SRS1 & RCM_SRS1_SW)        Serial.println("Software Reset");                   // reboot with SCB_AIRCR = 0x05FA0004
  // if (RCM_SRS1 & RCM_SRS1_LOCKUP)    Serial.println("Core Lockup Event Reset");
  // if (RCM_SRS0 & RCM_SRS0_POR)       Serial.println("Power-on Reset");                   // removed / applied power
  // if (RCM_SRS0 & RCM_SRS0_PIN)       Serial.println("External Pin Reset");               // Reboot with software download
  // if (RCM_SRS0 & RCM_SRS0_WDOG)      Serial.println("Watchdog(COP) Reset");              // WDT timed out
  // if (RCM_SRS0 & RCM_SRS0_LOC)       Serial.println("Loss of External Clock Reset");
  // if (RCM_SRS0 & RCM_SRS0_LOL)       Serial.println("Loss of Lock in PLL Reset");
  // if (RCM_SRS0 & RCM_SRS0_LVD)       Serial.println("Low-voltage Detect Reset");
  // Serial.println();
  // ///////////////////

  // Teensy 4.x
  if (CrashReport)
  {
    /* print info (hope Serial Monitor windows is open) */
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

  SERIALCONSOLE.println("Started serial interface to BMS.");

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

  SERIALCONSOLE.println("Recovery SOC: ");
  SERIALCONSOLE.print(SOC);

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
  delay(500);                          // Wait for 1 second    
  digitalWrite(PIN_HEARTBEAT_LED, LOW); // Turn on heartbeat LED

  bmsstatus = BMS_STATUS_BOOT;
}

void loop()
{

  indicatorsLoop(); // Call the indicators loop to handle LED and buzzer state

  // On message recieve.
  while (canRead())
  {
  }

  // Check if serial menu is requested
  if (SERIALCONSOLE.available() > 0)
  {
    menu();
  }

  
  if (outputcheck != 1)
  {
    contactoControl();
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
              Serial.println(mainconttimer);
              Serial.println();
            }
            if (mainconttimer + settings.Pretime < millis() && digitalRead(PIN_OUT2) == LOW && abs(currentact) < settings.Precurrent)
            {
              digitalWrite(PIN_OUT2, HIGH); // turn on contactor
              contctrl = contctrl | 2;  // turn on contactor
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
              bmsstatus = BMS_STATUS_ERROR;
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
              digitalWrite(PIN_OUT2, LOW);   // turn off contactor
              contctrl = contctrl & 253; // turn off contactor
              digitalWrite(PIN_OUT4, LOW);   // ensure precharge is low
              bmsstatus = BMS_STATUS_ERROR;
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
        contctrl = 0;            // turn off out 5 and 6

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
            bmsstatus = BMS_STATUS_READY;
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
        bmsstatus = BMS_STATUS_READY;
        break;

      case (BMS_STATUS_READY):
        Discharge = 0;
        digitalWrite(PIN_OUT4, LOW);
        digitalWrite(PIN_OUT3, LOW); // turn off charger
        digitalWrite(PIN_OUT2, LOW);
        digitalWrite(PIN_OUT1, LOW); // turn off discharge
        contctrl = 0;            // turn off out 5 and 6
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
            bmsstatus = BMS_STATUS_CHARGE;
          }
          else
          {
            bmsstatus = BMS_STATUS_PRECHARGE;
            Pretimer = millis();
          }
        }
        if (digitalRead(PIN_IN1) == HIGH && bms.getLowCellVolt() > settings.DischVsetpoint) // detect Key ON
        {
          bmsstatus = BMS_STATUS_PRECHARGE;
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
          bmsstatus = BMS_STATUS_READY;
        }
        if (digitalRead(PIN_IN3) == HIGH && (bms.getHighCellVolt() < (settings.ChargeVsetpoint - settings.ChargeHys)) && bms.getHighTemperature() < (settings.OverTSetpoint - settings.WarnToff)) // detect AC present for charging and check not balancing
        {
          bmsstatus = BMS_STATUS_CHARGE;
        }

        break;

      case (BMS_STATUS_CHARGE):
        if (settings.ChargerDirect > 0)
        {
          Discharge = 0;
          digitalWrite(PIN_OUT4, LOW);
          digitalWrite(PIN_OUT2, LOW);
          digitalWrite(PIN_OUT1, LOW); // turn off discharge
          contctrl = 0;            // turn off out 5 and 6
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
          bmsstatus = BMS_STATUS_READY;
        }
        if (digitalRead(PIN_IN3) == LOW) // detect AC not present for charging
        {
          bmsstatus = BMS_STATUS_READY;
        }
        break;

      case (BMS_STATUS_ERROR):
        Discharge = 0;
        digitalWrite(PIN_OUT4, LOW);
        digitalWrite(PIN_OUT3, LOW); // turn off charger
        digitalWrite(PIN_OUT2, LOW);
        digitalWrite(PIN_OUT1, LOW); // turn off discharge
        contctrl = 0;            // turn off out 5 and 6
        /*
                  if (digitalRead(IN3) == HIGH) //detect AC present for charging
                  {
                    bmsstatus = Charge;
                  }
        */
        if (bms.getLowCellVolt() >= settings.UnderVSetpoint && bms.getHighCellVolt() <= settings.OverVSetpoint && digitalRead(PIN_IN1) == LOW)
        {
          bmsstatus = BMS_STATUS_READY;
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
          bmsstatus = BMS_STATUS_ERROR;
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
          bmsstatus = BMS_STATUS_ERROR;
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
          bmsstatus = BMS_STATUS_ERROR;
          bmsError = ERROR_VOLTAGE; // TODO - refine error state
        }
      }
      else
      {
        UnderTimer = millis() + settings.triptime;
      }

      if (bms.getHighCellVolt() < settings.UnderVSetpoint || bms.getHighTemperature() > settings.OverTSetpoint)
      {
        bmsstatus = BMS_STATUS_ERROR;
        bmsError = ERROR_VOLTAGE; // TODO - refine error state
      }

      if (bms.getHighCellVolt() > settings.OverVSetpoint)
      {
        if (OverTime < millis()) // check is last time not undervoltage is longer thatn UnderDur ago
        {
          bmsstatus = BMS_STATUS_ERROR;
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
      gaugeupdate();
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
          SERIALCONSOLE.println("  ");
          SERIALCONSOLE.print("   !!! Series Cells Fault !!!");
          SERIALCONSOLE.println("  ");
          bmsstatus = BMS_STATUS_ERROR;
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

void loadSettings()
{
  Logger::console("Resetting to factory defaults");
  settings.version = EEPROM_VERSION;
  settings.checksum = DEFAULT_CHECKSUM;
  settings.canSpeed = DEFAULT_CAN_SPEED;
  settings.batteryID = DEFAULT_BATTERY_ID;
  settings.OverVSetpoint = DEFAULT_OVERV_SETPOINT;
  settings.UnderVSetpoint = DEFAULT_UNDERV_SETPOINT;
  settings.ChargeVsetpoint = DEFAULT_CHARGEV_SETPOINT;
  settings.ChargeHys = DEFAULT_CHARGE_HYS;
  settings.WarnOff = DEFAULT_WARN_OFF;
  settings.DischVsetpoint = DEFAULT_DISCHV_SETPOINT;
  settings.DischHys = DEFAULT_DISCH_HYS;
  settings.CellGap = DEFAULT_CELL_GAP;
  settings.OverTSetpoint = DEFAULT_OVERT_SETPOINT;
  settings.UnderTSetpoint = DEFAULT_UNDERT_SETPOINT;
  settings.ChargeTSetpoint = DEFAULT_CHARGET_SETPOINT;
  settings.triptime = DEFAULT_TRIPTIME;
  settings.DisTSetpoint = DEFAULT_DIST_SETPOINT;
  settings.WarnToff = DEFAULT_WARN_TOFF;
  settings.IgnoreTemp = DEFAULT_IGNORE_TEMP;
  settings.IgnoreVolt = DEFAULT_IGNORE_VOLT;
  settings.balanceVoltage = DEFAULT_BALANCE_VOLTAGE;
  settings.balanceHyst = DEFAULT_BALANCE_HYST;
  settings.balanceDuty = DEFAULT_BALANCE_DUTY;
  settings.logLevel = DEFAULT_LOG_LEVEL;
  settings.CAP = DEFAULT_CAP;
  settings.Pstrings = DEFAULT_PSTRINGS;
  settings.Scells = DEFAULT_SCELLS;
  settings.StoreVsetpoint = DEFAULT_STOREV_SETPOINT;
  settings.discurrentmax = DEFAULT_DISCURRENTMAX;
  settings.DisTaper = DEFAULT_DISTAPER;
  settings.chargecurrentmax = DEFAULT_CHARGECURRENTMAX;
  settings.chargecurrent2max = DEFAULT_CHARGECURRENT2MAX;
  settings.chargecurrentend = DEFAULT_CHARGECURRENTEND;
  settings.PulseCh = DEFAULT_PULSECH;
  settings.PulseChDur = DEFAULT_PULSECHDUR;
  settings.PulseDi = DEFAULT_PULSEDI;
  settings.PulseDiDur = DEFAULT_PULSEDIDUR;
  settings.socvolt[0] = DEFAULT_SOCVOLT_0;
  settings.socvolt[1] = DEFAULT_SOCVOLT_1;
  settings.socvolt[2] = DEFAULT_SOCVOLT_2;
  settings.socvolt[3] = DEFAULT_SOCVOLT_3;
  settings.invertcur = DEFAULT_INVERTCUR;
  settings.cursens = DEFAULT_CURSENS;
  settings.curcan = DEFAULT_CURCAN;
  settings.voltsoc = DEFAULT_VOLTSOC;
  settings.Pretime = DEFAULT_PRETIME;
  settings.conthold = DEFAULT_CONTHOLD;
  settings.Precurrent = DEFAULT_PRECURRENT;
  settings.convhigh = DEFAULT_CONVHIGH;
  settings.convlow = DEFAULT_CONVLOW;
  settings.offset1 = DEFAULT_OFFSET1;
  settings.offset2 = DEFAULT_OFFSET2;
  settings.changecur = DEFAULT_CHANGECUR;
  settings.gaugelow = DEFAULT_GAUGELOW;
  settings.gaugehigh = DEFAULT_GAUGEHIGH;
  settings.ESSmode = DEFAULT_ESSMODE;
  settings.ncur = DEFAULT_NCUR;
  settings.chargertype = DEFAULT_CHARGERTYPE;
  settings.chargerspd = DEFAULT_CHARGERSPD;
  settings.chargereff = DEFAULT_CHARGEREFF;
  settings.chargerACv = DEFAULT_CHARGERACV;
  settings.UnderDur = DEFAULT_UNDERDUR;
  settings.CurDead = DEFAULT_CURDEAD;
  settings.ExpMess = DEFAULT_EXPMESS;
  settings.SerialCan = DEFAULT_SERIALCAN;
  settings.tripcont = DEFAULT_TRIPCONT;
  settings.ChargerDirect = DEFAULT_CHARGERDIRECT;
}

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

void gaugeupdate()
{
  if (gaugedebug == 1)
  {
    SOCtest = SOCtest + 10;
    if (SOCtest > 1000)
    {
      SOCtest = 0;
    }
    analogWrite(PIN_OUT8, map(SOCtest * 0.1, 0, 100, settings.gaugelow, settings.gaugehigh));
    if (debug != 0)
    {
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("SOC : ");
      SERIALCONSOLE.print(SOCtest * 0.1);
      SERIALCONSOLE.print("  fuel pwm : ");
      SERIALCONSOLE.print(map(SOCtest * 0.1, 0, 100, settings.gaugelow, settings.gaugehigh));
      SERIALCONSOLE.println("  ");
    }
  }
  if (gaugedebug == 2)
  {
    SOCtest = 0;
    analogWrite(PIN_OUT8, map(SOCtest * 0.1, 0, 100, settings.gaugelow, settings.gaugehigh));
  }
  if (gaugedebug == 3)
  {
    SOCtest = 1000;
    analogWrite(PIN_OUT8, map(SOCtest * 0.1, 0, 100, settings.gaugelow, settings.gaugehigh));
  }
  if (gaugedebug == 0)
  {
    analogWrite(PIN_OUT8, map(SOC, 0, 100, settings.gaugelow, settings.gaugehigh));
  }
}

void printbmsstat()
{
  SERIALCONSOLE.println();
  SERIALCONSOLE.println();
  SERIALCONSOLE.println(testcount);
  SERIALCONSOLE.print("BMS Status : ");
  if (settings.ESSmode == 1)
  {
    SERIALCONSOLE.print("ESS Mode ");

    if (bms.getLowCellVolt() < settings.UnderVSetpoint)
    {
      SERIALCONSOLE.print(": UnderVoltage ");
    }
    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      SERIALCONSOLE.print(": OverVoltage ");
    }
    if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap)
    {
      SERIALCONSOLE.print(": Cell Imbalance ");
    }
    if (bms.getAvgTemperature() > settings.OverTSetpoint)
    {
      SERIALCONSOLE.print(": Over Temp ");
    }
    if (bms.getAvgTemperature() < settings.UnderTSetpoint)
    {
      SERIALCONSOLE.print(": Under Temp ");
    }
    if (storagemode == 1)
    {
      if (bms.getLowCellVolt() > settings.StoreVsetpoint)
      {
        SERIALCONSOLE.print(": OverVoltage Storage ");
        SERIALCONSOLE.print(": UNhappy:");
      }
      else
      {
        SERIALCONSOLE.print(": Happy ");
      }
    }
    else
    {
      if (bms.getLowCellVolt() > settings.UnderVSetpoint && bms.getHighCellVolt() < settings.OverVSetpoint)
      {

        if (bmsstatus == BMS_STATUS_ERROR)
        {
          SERIALCONSOLE.print(": UNhappy:");
        }
        else
        {
          SERIALCONSOLE.print(": Happy ");
        }
      }
    }
  }
  else
  {
    SERIALCONSOLE.print(bmsstatus);
    switch (bmsstatus)
    {
    case (BMS_STATUS_BOOT):
      SERIALCONSOLE.print(" Boot ");
      break;

    case (BMS_STATUS_READY):
      SERIALCONSOLE.print(" Ready ");
      break;

    case (BMS_STATUS_PRECHARGE):
      SERIALCONSOLE.print(" Precharge ");
      break;

    case (BMS_STATUS_DRIVE):
      SERIALCONSOLE.print(" Drive ");
      break;

    case (BMS_STATUS_CHARGE):
      SERIALCONSOLE.print(" Charge ");
      break;

    case (BMS_STATUS_ERROR):
      SERIALCONSOLE.print(" Error ");
      break;
    }
  }
  SERIALCONSOLE.print("  ");
  if (digitalRead(PIN_IN3) == HIGH)
  {
    SERIALCONSOLE.print("| AC Present |");
  }
  if (digitalRead(PIN_IN1) == HIGH)
  {
    SERIALCONSOLE.print("| Key ON |");
  }
  if (balancecells == 1)
  {
    SERIALCONSOLE.print("|Balancing Active");
  }
  SERIALCONSOLE.print("  ");
  SERIALCONSOLE.print(cellspresent);
  SERIALCONSOLE.println();
  SERIALCONSOLE.print("Out:");
  SERIALCONSOLE.print(digitalRead(PIN_OUT1));
  SERIALCONSOLE.print(digitalRead(PIN_OUT2));
  SERIALCONSOLE.print(digitalRead(PIN_OUT3));
  SERIALCONSOLE.print(digitalRead(PIN_OUT4));
  SERIALCONSOLE.print(" Cont:");
  if ((contstat & 1) == 1)
  {
    SERIALCONSOLE.print("1");
  }
  else
  {
    SERIALCONSOLE.print("0");
  }
  if ((contstat & 2) == 2)
  {
    SERIALCONSOLE.print("1");
  }
  else
  {
    SERIALCONSOLE.print("0");
  }
  if ((contstat & 4) == 4)
  {
    SERIALCONSOLE.print("1");
  }
  else
  {
    SERIALCONSOLE.print("0");
  }
  if ((contstat & 8) == 8)
  {
    SERIALCONSOLE.print("1");
  }
  else
  {
    SERIALCONSOLE.print("0");
  }
  SERIALCONSOLE.print(" In:");
  SERIALCONSOLE.print(digitalRead(PIN_IN1));
  SERIALCONSOLE.print(digitalRead(PIN_IN2));
  SERIALCONSOLE.print(digitalRead(PIN_IN3));
  SERIALCONSOLE.print(digitalRead(PIN_IN4));

  SERIALCONSOLE.print(" Charge Current Limit : ");
  SERIALCONSOLE.print(chargecurrent * 0.1, 0);
  SERIALCONSOLE.print(" A DisCharge Current Limit : ");
  SERIALCONSOLE.print(discurrent * 0.1, 0);
  SERIALCONSOLE.print(" A");

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
      SERIALCONSOLE.print("  No Charge Current Limit");
    }
    else
    {
      SERIALCONSOLE.print("  Charge Current Limit Active");
    }
  }
}

void getcurrent()
{
  if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL || settings.cursens == CURR_SENSE_ANALOGUE_GUESSING)
  {
    if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL)
    {
      if (currentact < settings.changecur && currentact > (settings.changecur * -1))
      {
        sensor = 1;
        adc->adc0->startContinuous(PIN_ACUR_1);
      }
      else
      {
        sensor = 2;
        adc->adc0->startContinuous(PIN_ACUR_2);
      }
    }
    else
    {
      sensor = 1;
      adc->adc0->startContinuous(PIN_ACUR_1);
    }
    if (sensor == 1)
    {
      if (debugCur)
      {
        SERIALCONSOLE.println();
        if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL)
        {
          SERIALCONSOLE.print("Low Range: ");
        }
        else
        {
          SERIALCONSOLE.print("Single In: ");
        }
        SERIALCONSOLE.print("Value ADC0: ");
      }
      value = (uint16_t)adc->adc0->analogReadContinuous(); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
      if (debugCur)
      {
        SERIALCONSOLE.print(value * 3300 / adc->adc0->getMaxValue()); //- settings.offset1)
        SERIALCONSOLE.print(" ");
        SERIALCONSOLE.print(settings.offset1);
      }
      RawCur = int16_t((value * 3300 / adc->adc0->getMaxValue()) - settings.offset1) / (settings.convlow * 0.0000066);

      if (abs((int16_t(value * 3300 / adc->adc0->getMaxValue()) - settings.offset1)) < settings.CurDead)
      {
        RawCur = 0;
      }
      if (debugCur != 0)
      {
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print(int16_t(value * 3300 / adc->adc0->getMaxValue()) - settings.offset1);
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print(RawCur);
        SERIALCONSOLE.print(" mA");
        SERIALCONSOLE.print("  ");
      }
    }
    else
    {
      if (debugCur != 0)
      {
        SERIALCONSOLE.println();
        SERIALCONSOLE.print("High Range: ");
        SERIALCONSOLE.print("Value ADC0: ");
      }
      value = (uint16_t)adc->adc0->analogReadContinuous(); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
      if (debugCur != 0)
      {
        SERIALCONSOLE.print(value * 3300 / adc->adc0->getMaxValue()); //- settings.offset2)
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print(settings.offset2);
      }
      RawCur = int16_t((value * 3300 / adc->adc0->getMaxValue()) - settings.offset2) / (settings.convhigh * 0.0000066);
      if (value < 100 || value > (adc->adc0->getMaxValue() - 100))
      {
        RawCur = 0;
      }
      if (debugCur != 0)
      {
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print((float(value * 3300 / adc->adc0->getMaxValue()) - settings.offset2));
        SERIALCONSOLE.print("  ");
        SERIALCONSOLE.print(RawCur);
        SERIALCONSOLE.print("mA");
        SERIALCONSOLE.print("  ");
      }
    }
  }

  if (settings.invertcur == 1)
  {
    RawCur = RawCur * -1;
  }

  lowpassFilter.input(RawCur);
  if (debugCur != 0)
  {
    SERIALCONSOLE.print(lowpassFilter.output());
    SERIALCONSOLE.print(" | ");
    SERIALCONSOLE.print(settings.changecur);
    SERIALCONSOLE.print(" | ");
  }

  currentact = lowpassFilter.output();

  if (debugCur != 0)
  {
    SERIALCONSOLE.print(currentact);
    SERIALCONSOLE.print("mA  ");
  }

  if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL)
  {
    if (sensor == 1)
    {
      if (currentact > 500 || currentact < -500)
      {
        ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
        lasttime = millis();
      }
      else
      {
        lasttime = millis();
      }
    }
    if (sensor == 2)
    {
      if (currentact > settings.changecur || currentact < (settings.changecur * -1))
      {
        ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
        lasttime = millis();
      }
      else
      {
        lasttime = millis();
      }
    }
  }
  else
  {
    if (currentact > 500 || currentact < -500)
    {
      ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
      lasttime = millis();
    }
    else
    {
      lasttime = millis();
    }
  }
  currentact = settings.ncur * currentact;
  RawCur = 0;
  /*
    AverageCurrentTotal = AverageCurrentTotal - RunningAverageBuffer[NextRunningAverage];

    RunningAverageBuffer[NextRunningAverage] = currentact;

    if (debugCur != 0)
    {
      SERIALCONSOLE.print(" | ");
      SERIALCONSOLE.print(AverageCurrentTotal);
      SERIALCONSOLE.print(" | ");
      SERIALCONSOLE.print(RunningAverageBuffer[NextRunningAverage]);
      SERIALCONSOLE.print(" | ");
    }
    AverageCurrentTotal = AverageCurrentTotal + RunningAverageBuffer[NextRunningAverage];
    if (debugCur != 0)
    {
      SERIALCONSOLE.print(" | ");
      SERIALCONSOLE.print(AverageCurrentTotal);
      SERIALCONSOLE.print(" | ");
    }

    NextRunningAverage = NextRunningAverage + 1;

    if (NextRunningAverage > RunningAverageCount)
    {
      NextRunningAverage = 0;
    }

    AverageCurrent = AverageCurrentTotal / (RunningAverageCount + 1);

    if (debugCur != 0)
    {
      SERIALCONSOLE.print(AverageCurrent);
      SERIALCONSOLE.print(" | ");
      SERIALCONSOLE.print(AverageCurrentTotal);
      SERIALCONSOLE.print(" | ");
      SERIALCONSOLE.print(NextRunningAverage);
    }
  */
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
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println("//////////////////////////////////////// SOC SET ////////////////////////////////////////");
      }
      if (settings.ESSmode == 1)
      {
        bmsstatus = BMS_STATUS_READY;
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
        SERIALCONSOLE.println("  ");
        SERIALCONSOLE.println("//////////////////////////////////////// SOC SET ////////////////////////////////////////");
      }
      if (settings.ESSmode == 1)
      {
        bmsstatus = BMS_STATUS_READY;
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
        SERIALCONSOLE.print("Low Range ");
      }
      else
      {
        SERIALCONSOLE.print("High Range");
      }
    }
    if (settings.cursens == CURR_SENSE_ANALOGUE_GUESSING)
    {
      SERIALCONSOLE.print("Analogue Single ");
    }
    if (settings.cursens == CURR_SENSE_CANBUS)
    {
      SERIALCONSOLE.print("CANbus ");
    }
    SERIALCONSOLE.print("  ");
    SERIALCONSOLE.print(currentact);
    SERIALCONSOLE.print("mA");
    SERIALCONSOLE.print("  ");
    SERIALCONSOLE.print(SOC);
    SERIALCONSOLE.print("% SOC ");
    SERIALCONSOLE.print(ampsecond * 0.27777777777778, 2);
    SERIALCONSOLE.print("mAh");
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
        bmsstatus = BMS_STATUS_DRIVE;
      }
      else
      {
        if (digitalRead(PIN_IN3) == HIGH)
        {
          bmsstatus = BMS_STATUS_CHARGE;
        }
        if (digitalRead(PIN_IN1) == HIGH)
        {
          bmsstatus = BMS_STATUS_DRIVE;
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
    bmsstatus = BMS_STATUS_READY;
    contctrl = 0;
  }
}

// Contactor Control Function
void contactoControl()
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
       SERIALCONSOLE.print(conttimer);
       SERIALCONSOLE.print("  ");
       SERIALCONSOLE.print(contctrl);
       SERIALCONSOLE.print("  ");
       SERIALCONSOLE.print(contstat);
       SERIALCONSOLE.println("  ");
    */
  }
  if (contctrl == 0)
  {
    analogWrite(PIN_OUT5, 0);
    analogWrite(PIN_OUT6, 0);
  }
}

void calcur()
{
  adc->startContinuous(PIN_ACUR_1, ADC_0);
  sensor = 1;
  x = 0;
  SERIALCONSOLE.print(" Calibrating Current Offset ::::: ");
  while (x < 20)
  {
    settings.offset1 = settings.offset1 + ((uint16_t)adc->adc0->analogReadContinuous() * 3300 / adc->adc0->getMaxValue());
    SERIALCONSOLE.print(".");
    delay(100);
    x++;
  }
  settings.offset1 = settings.offset1 / 21;
  SERIALCONSOLE.print(settings.offset1);
  SERIALCONSOLE.print(" current offset 1 calibrated ");
  SERIALCONSOLE.println("  ");
  x = 0;
  adc->adc0->startContinuous(PIN_ACUR_2);
  sensor = 2;
  SERIALCONSOLE.print(" Calibrating Current Offset ::::: ");
  while (x < 20)
  {
    settings.offset2 = settings.offset2 + ((uint16_t)adc->adc0->analogReadContinuous() * 3300 / adc->adc0->getMaxValue());
    SERIALCONSOLE.print(".");
    delay(100);
    x++;
  }
  settings.offset2 = settings.offset2 / 21;
  SERIALCONSOLE.print(settings.offset2);
  SERIALCONSOLE.print(" current offset 2 calibrated ");
  SERIALCONSOLE.println("  ");
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

// Settings menu
void menu()
{

  incomingByte = Serial.read(); // read the incoming byte:
  if (menuload == 4)
  {
    switch (incomingByte)
    {

    case '1':
      menuload = 1;
      candebug = !candebug;
      incomingByte = 'd';
      break;

    case '2':
      menuload = 1;
      debugCur = !debugCur;
      incomingByte = 'd';
      break;

    case '3':
      menuload = 1;
      outputcheck = !outputcheck;
      if (outputcheck == 0)
      {
        contctrl = 0;
        digitalWrite(PIN_OUT1, LOW);
        digitalWrite(PIN_OUT2, LOW);
        digitalWrite(PIN_OUT3, LOW);
        digitalWrite(PIN_OUT4, LOW);
      }
      incomingByte = 'd';
      break;

    case '4':
      menuload = 1;
      inputcheck = !inputcheck;
      incomingByte = 'd';
      break;

    case '5':
      menuload = 1;
      settings.ESSmode = !settings.ESSmode;
      incomingByte = 'd';
      break;

    case '6':
      menuload = 1;
      cellspresent = bms.seriescells();
      incomingByte = 'd';
      break;

    case '7':
      menuload = 1;
      gaugedebug = !gaugedebug;
      incomingByte = 'd';
      break;

    case '8':
      menuload = 1;
      CSVdebug = !CSVdebug;
      incomingByte = 'd';
      break;

    case '9':
      menuload = 1;
      if (Serial.available() > 0)
      {
        debugdigits = Serial.parseInt();
      }
      if (debugdigits > 4)
      {
        debugdigits = 2;
      }
      incomingByte = 'd';
      break;

    case '0':
      menuload = 1;
      CPdebug = !CPdebug;
      incomingByte = 'd';
      break;

    case 'd':
      menuload = 1;
      if (Serial.available() > 0)
      {
        delim = Serial.parseInt();
      }
      if (delim > 1)
      {
        delim = 0;
      }
      incomingByte = 'd';
      break;

    case 113: // q for quite menu

      menuload = 0;
      incomingByte = 115;
      break;

    default:
      // if nothing else matches, do the default
      // default is optional
      break;
    }
  }

  if (menuload == 9)
  {
    if (settings.ExpMess > 1)
    {
      settings.ExpMess = 0;
    }
    switch (incomingByte)
    {

    case '1':
      menuload = 1;
      settings.ExpMess = !settings.ExpMess;
      incomingByte = 'x';
      break;
    case 113: // q to go back to main menu
      menuload = 0;
      incomingByte = 115;
      break;
    }
  }

  if (menuload == 2)
  {
    switch (incomingByte)
    {

    case 99: // c for calibrate zero offset

      calcur();
      break;

    case '1':
      menuload = 1;
      settings.invertcur = !settings.invertcur;
      incomingByte = 'c';
      break;

    case '2':
      menuload = 1;
      settings.voltsoc = !settings.voltsoc;
      incomingByte = 'c';
      break;

    case '3':
      menuload = 1;
      if (Serial.available() > 0)
      {
        settings.ncur = Serial.parseInt();
      }
      menuload = 1;
      incomingByte = 'c';
      break;

    case '8':
      menuload = 1;
      if (Serial.available() > 0)
      {
        settings.changecur = Serial.parseInt();
      }
      menuload = 1;
      incomingByte = 'c';
      break;

    case '4':
      menuload = 1;
      if (Serial.available() > 0)
      {
        settings.convlow = Serial.parseInt();
      }
      incomingByte = 'c';
      break;

    case '5':
      menuload = 1;
      if (Serial.available() > 0)
      {
        settings.convhigh = Serial.parseInt();
      }
      incomingByte = 'c';
      break;

    case '6':
      menuload = 1;
      if (Serial.available() > 0)
      {
        settings.CurDead = Serial.parseInt();
      }
      incomingByte = 'c';
      break;

    case 113: // q for quite menu

      menuload = 0;
      incomingByte = 115;
      break;

    case 115: // s for switch sensor
      settings.cursens++;
      if (settings.cursens > 3)
      {
        settings.cursens = 0;
      }
      /*
        if (settings.cursens == Analoguedual)
        {
          settings.cursens = Canbus;
          SERIALCONSOLE.println("  ");
          SERIALCONSOLE.print(" CANbus Current Sensor ");
          SERIALCONSOLE.println("  ");
        }
        else
        {
          settings.cursens = Analoguedual;
          SERIALCONSOLE.println("  ");
          SERIALCONSOLE.print(" Analogue Current Sensor ");
          SERIALCONSOLE.println("  ");
        }
      */
      menuload = 1;
      incomingByte = 'c';
      break;

    case '7': // s for switch sensor
      settings.curcan++;
      if (settings.curcan > CurCanMax)
      {
        settings.curcan = 1;
      }
      menuload = 1;
      incomingByte = 'c';
      break;

    default:
      // if nothing else matches, do the default
      // default is optional
      break;
    }
  }

  if (menuload == 8)
  {
    switch (incomingByte)
    {
    case '1': // e dispaly settings
      if (Serial.available() > 0)
      {
        settings.IgnoreTemp = Serial.parseInt();
      }
      if (settings.IgnoreTemp > 2)
      {
        settings.IgnoreTemp = 0;
      }
      bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
      menuload = 1;
      incomingByte = 'i';
      break;

    case '2':
      if (Serial.available() > 0)
      {
        settings.IgnoreVolt = Serial.parseInt();
        settings.IgnoreVolt = settings.IgnoreVolt * 0.001;
        bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
        // Serial.println(settings.IgnoreVolt);
        menuload = 1;
        incomingByte = 'i';
      }
      break;

    case 113: // q to go back to main menu

      menuload = 0;
      incomingByte = 115;
      break;
    }
  }

  if (menuload == 7)
  {
    switch (incomingByte)
    {
    case '1':
      if (Serial.available() > 0)
      {
        settings.WarnOff = Serial.parseInt();
        settings.WarnOff = settings.WarnOff * 0.001;
        menuload = 1;
        incomingByte = 'a';
      }
      break;

    case '2':
      if (Serial.available() > 0)
      {
        settings.CellGap = Serial.parseInt();
        settings.CellGap = settings.CellGap * 0.001;
        menuload = 1;
        incomingByte = 'a';
      }
      break;

    case '3':
      if (Serial.available() > 0)
      {
        settings.WarnToff = Serial.parseInt();
        menuload = 1;
        incomingByte = 'a';
      }
      break;

    case '4':
      if (Serial.available() > 0)
      {
        settings.triptime = Serial.parseInt();
        menuload = 1;
        incomingByte = 'a';
      }
      break;

    case 113: // q to go back to main menu
      menuload = 0;
      incomingByte = 115;
      break;
    }
  }

  if (menuload == 6) // Charging settings
  {
    switch (incomingByte)
    {

    case 113: // q to go back to main menu

      menuload = 0;
      incomingByte = 115;
      break;

    case '1':
      if (Serial.available() > 0)
      {
        settings.ChargeVsetpoint = Serial.parseInt();
        settings.ChargeVsetpoint = settings.ChargeVsetpoint / 1000;
        menuload = 1;
        incomingByte = 'e';
      }
      break;

    case '2':
      if (Serial.available() > 0)
      {
        settings.ChargeHys = Serial.parseInt();
        settings.ChargeHys = settings.ChargeHys / 1000;
        menuload = 1;
        incomingByte = 'e';
      }
      break;

    case '4':
      if (Serial.available() > 0)
      {
        settings.chargecurrentend = Serial.parseInt() * 10;
        menuload = 1;
        incomingByte = 'e';
      }
      break;

    case '3':
      if (Serial.available() > 0)
      {
        settings.chargecurrentmax = Serial.parseInt() * 10;
        menuload = 1;
        incomingByte = 'e';
      }
      break;

    case 'a':
      if (Serial.available() > 0)
      {
        settings.chargecurrent2max = Serial.parseInt() * 10;
        menuload = 1;
        incomingByte = 'e';
      }
      break;

    case '5': // 1 Over Voltage Setpoint
      settings.chargertype = settings.chargertype + 1;
      if (settings.chargertype > 7)
      {
        settings.chargertype = 0;
      }
      menuload = 1;
      incomingByte = 'e';
      break;

    case '6':
      if (Serial.available() > 0)
      {
        settings.chargerspd = Serial.parseInt();
        menuload = 1;
        incomingByte = 'e';
      }
      break;

    case '7':
      if (Serial.available() > 0)
      {
        settings.canSpeed = Serial.parseInt() * 1000;
        // Can1.end(); //FlexCAN_T4 has no end function TODO
        Can1.begin();
        Can1.setBaudRate(settings.canSpeed);
        menuload = 1;
        incomingByte = 'e';
      }
      break;

    case '8':
      if (settings.ChargerDirect == 1)
      {
        settings.ChargerDirect = 0;
        menuload = 1;
        incomingByte = 'e';
      }
      else
      {
        settings.ChargerDirect = 1;
        menuload = 1;
        incomingByte = 'e';
      }
      break;

    case '9':
      if (Serial.available() > 0)
      {
        settings.ChargeTSetpoint = Serial.parseInt();
        menuload = 1;
        incomingByte = 'e';
      }
      break;

    case 'b':
      if (Serial.available() > 0)
      {
        settings.chargereff = Serial.parseInt();
        menuload = 1;
        incomingByte = 'e';
      }
      break;

    case 'c':
      if (Serial.available() > 0)
      {
        settings.chargerACv = Serial.parseInt();
        menuload = 1;
        incomingByte = 'e';
      }
      break;

    case 'd':
      if (Serial.available() > 0)
      {
        if (settings.SerialCan == 0)
        {
          settings.SerialCan = 1;
        }
        else
        {
          settings.SerialCan = 0;
        }
        menuload = 1;
        incomingByte = 'e';
      }
      break;
    }
  }

  if (menuload == 5)
  {
    switch (incomingByte)
    {
    case '1':
      if (Serial.available() > 0)
      {
        settings.Pretime = Serial.parseInt();
        menuload = 1;
        incomingByte = 'k';
      }
      break;

    case '2':
      if (Serial.available() > 0)
      {
        settings.Precurrent = Serial.parseInt();
        menuload = 1;
        incomingByte = 'k';
      }
      break;

    case '3':
      if (Serial.available() > 0)
      {
        settings.conthold = Serial.parseInt();
        menuload = 1;
        incomingByte = 'k';
      }
      break;

    case '4':
      if (Serial.available() > 0)
      {
        settings.gaugelow = Serial.parseInt();
        gaugedebug = 2;
        gaugeupdate();
        menuload = 1;
        incomingByte = 'k';
      }
      break;

    case '5':
      if (Serial.available() > 0)
      {
        settings.gaugehigh = Serial.parseInt();
        gaugedebug = 3;
        gaugeupdate();
        menuload = 1;
        incomingByte = 'k';
      }
      break;

    case '6':
      settings.tripcont = !settings.tripcont;
      if (settings.tripcont > 1)
      {
        settings.tripcont = 0;
      }
      menuload = 1;
      incomingByte = 'k';
      break;

    case '7':
      if (settings.ChargerDirect == 1)
      {
        settings.ChargerDirect = 0;
      }
      else
      {
        settings.ChargerDirect = 1;
      }
      menuload = 1;
      incomingByte = 'k';
      break;

    case 113: // q to go back to main menu
      gaugedebug = 0;
      menuload = 0;
      incomingByte = 115;
      break;
    }
  }

  if (menuload == 3)
  {
    switch (incomingByte)
    {
    case 113: // q to go back to main menu

      menuload = 0;
      incomingByte = 115;
      break;

    case 'b':
      if (Serial.available() > 0)
      {
        settings.socvolt[0] = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case 'f': // f factory settings
      loadSettings();
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.println(" Coded Settings Loaded ");
      SERIALCONSOLE.println("  ");
      menuload = 1;
      incomingByte = 'b';
      break;

    case 'r': // r for reset
      SOCreset = 1;
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print(" mAh Reset ");
      SERIALCONSOLE.println("  ");
      menuload = 1;
      incomingByte = 'b';
      break;

    case '1': // 1 Over Voltage Setpoint
      if (Serial.available() > 0)
      {
        settings.OverVSetpoint = Serial.parseInt();
        settings.OverVSetpoint = settings.OverVSetpoint / 1000;
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case 'g':
      if (Serial.available() > 0)
      {
        settings.StoreVsetpoint = Serial.parseInt();
        settings.StoreVsetpoint = settings.StoreVsetpoint / 1000;
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case 'h':
      if (Serial.available() > 0)
      {
        settings.DisTaper = Serial.parseInt();
        settings.DisTaper = settings.DisTaper / 1000;
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case 'j':
      if (Serial.available() > 0)
      {
        settings.DisTSetpoint = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case 'c':
      if (Serial.available() > 0)
      {
        settings.socvolt[1] = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case 'd':
      if (Serial.available() > 0)
      {
        settings.socvolt[2] = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case 'e':
      if (Serial.available() > 0)
      {
        settings.socvolt[3] = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '9': // Discharge Voltage Setpoint
      if (Serial.available() > 0)
      {
        settings.DischVsetpoint = Serial.parseInt();
        settings.DischVsetpoint = settings.DischVsetpoint / 1000;
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case 'k': // Discharge Voltage hysteresis
      if (Serial.available() > 0)
      {
        settings.DischHys = Serial.parseInt();
        settings.DischHys = settings.DischHys / 1000;
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '0': // c Pstrings
      if (Serial.available() > 0)
      {
        settings.Pstrings = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
        bms.setPstrings(settings.Pstrings);
      }
      break;

    case 'a': //
      if (Serial.available() > 0)
      {
        settings.Scells = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '2': // 2 Under Voltage Setpoint
      if (Serial.available() > 0)
      {
        settings.UnderVSetpoint = Serial.parseInt();
        settings.UnderVSetpoint = settings.UnderVSetpoint / 1000;
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '3': // 3 Over Temperature Setpoint
      if (Serial.available() > 0)
      {
        settings.OverTSetpoint = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '4': // 4 Udner Temperature Setpoint
      if (Serial.available() > 0)
      {
        settings.UnderTSetpoint = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '5': // 5 Balance Voltage Setpoint
      if (Serial.available() > 0)
      {
        settings.balanceVoltage = Serial.parseInt();
        settings.balanceVoltage = settings.balanceVoltage / 1000;
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '6': // 6 Balance Voltage Hystersis
      if (Serial.available() > 0)
      {
        settings.balanceHyst = Serial.parseInt();
        settings.balanceHyst = settings.balanceHyst / 1000;
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '7': // 7 Battery Capacity inAh
      if (Serial.available() > 0)
      {
        settings.CAP = Serial.parseInt();
        menuload = 1;
        incomingByte = 'b';
      }
      break;

    case '8': // discurrent in A
      if (Serial.available() > 0)
      {
        settings.discurrentmax = Serial.parseInt() * 10;
        menuload = 1;
        incomingByte = 'b';
      }
      break;
    }
  }

  if (menuload == 1)
  {
    switch (incomingByte)
    {
    case 'R': // restart
      CPU_REBOOT;
      break;

    case 'x': // Ignore Value Settings
      while (Serial.available())
      {
        Serial.read();
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Experimental Settings");
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Do not use unless you know what it does!!!!!");
      SERIALCONSOLE.print("1 - Sending Experimental Victron CAN:");
      SERIALCONSOLE.println(settings.ExpMess);

      SERIALCONSOLE.println("q - Go back to menu");
      menuload = 9;
      break;

    case 'i': // Ignore Value Settings
      while (Serial.available())
      {
        Serial.read();
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Ignore Value Settings");
      SERIALCONSOLE.print("1 - Temp Sensor Setting:");
      SERIALCONSOLE.println(settings.IgnoreTemp);
      SERIALCONSOLE.print("2 - Voltage Under Which To Ignore Cells:");
      SERIALCONSOLE.print(settings.IgnoreVolt * 1000, 0);
      SERIALCONSOLE.println("mV");
      SERIALCONSOLE.println("q - Go back to menu");
      menuload = 8;
      break;

    case 'e': // Charging settings
      while (Serial.available())
      {
        Serial.read();
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Charging Settings");
      SERIALCONSOLE.print("1 - Cell Charge Voltage Limit Setpoint: ");
      SERIALCONSOLE.print(settings.ChargeVsetpoint * 1000, 0);
      SERIALCONSOLE.println("mV");
      SERIALCONSOLE.print("2 - Charge Hystersis: ");
      SERIALCONSOLE.print(settings.ChargeHys * 1000, 0);
      SERIALCONSOLE.println("mV");
      if (settings.chargertype > 0)
      {
        SERIALCONSOLE.print("3 - Pack Max Charge Current: ");
        SERIALCONSOLE.print(settings.chargecurrentmax * 0.1);
        SERIALCONSOLE.println("A");
        SERIALCONSOLE.print("4- Pack End of Charge Current: ");
        SERIALCONSOLE.print(settings.chargecurrentend * 0.1);
        SERIALCONSOLE.println("A");
      }
      SERIALCONSOLE.print("5- Charger Type: ");
      switch (settings.chargertype)
      {
      case 0:
        SERIALCONSOLE.print("Relay Control");
        break;
      case 1:
        SERIALCONSOLE.print("Brusa NLG5xx");
        break;
      case 2:
        SERIALCONSOLE.print("Volt Charger");
        break;
      case 3:
        SERIALCONSOLE.print("Eltek Charger");
        break;
      case 4:
        SERIALCONSOLE.print("Elcon Charger");
        break;
      case 5:
        SERIALCONSOLE.print("Victron/SMA");
        break;
      case 6:
        SERIALCONSOLE.print("Coda");
        break;
      case 7:
        SERIALCONSOLE.print("Victron HV Spec");
        break;
      }
      SERIALCONSOLE.println();
      if (settings.chargertype > 0)
      {
        SERIALCONSOLE.print("6- Charger Can Msg Spd: ");
        SERIALCONSOLE.print(settings.chargerspd);
        SERIALCONSOLE.println("mS");
        SERIALCONSOLE.print("7- Can Baudrate: ");
        SERIALCONSOLE.print(settings.canSpeed * 0.001, 0);
        SERIALCONSOLE.println("kbps");
      }
      SERIALCONSOLE.print("8 - Charger HV Connection: ");
      switch (settings.ChargerDirect)
      {
      case 0:
        SERIALCONSOLE.print(" Behind Contactors");
        break;
      case 1:
        SERIALCONSOLE.print("Direct To Battery HV");
        break;
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.print("9 - Charge Current derate Low: ");
      SERIALCONSOLE.print(settings.ChargeTSetpoint);
      SERIALCONSOLE.println(" C");
      if (settings.chargertype > 0)
      {
        SERIALCONSOLE.print("a - Alternate Pack Max Charge Current: ");
        SERIALCONSOLE.print(settings.chargecurrent2max * 0.1);
        SERIALCONSOLE.println("A");
        SERIALCONSOLE.print("b - Charger AC to DC effiecency: ");
        SERIALCONSOLE.print(settings.chargereff);
        SERIALCONSOLE.println("%");
        SERIALCONSOLE.print("c - Charger AC Voltage: ");
        SERIALCONSOLE.print(settings.chargerACv);
        SERIALCONSOLE.println("VAC");
      }

      SERIALCONSOLE.print("d - Standard Can Voltage Scale: ");
      if (settings.SerialCan == 0)
      {
        SERIALCONSOLE.print("0.01");
      }
      else if (settings.SerialCan == 1)
      {
        SERIALCONSOLE.print("0.1");
      }
      SERIALCONSOLE.println();

      SERIALCONSOLE.println("q - Go back to menu");
      menuload = 6;
      break;

    case 'a': // Alarm and Warning settings
      while (Serial.available())
      {
        Serial.read();
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Alarm and Warning Settings Menu");
      SERIALCONSOLE.print("1 - Voltage Warning Offset: ");
      SERIALCONSOLE.print(settings.WarnOff * 1000, 0);
      SERIALCONSOLE.println("mV");
      SERIALCONSOLE.print("2 - Cell Voltage Difference Alarm: ");
      SERIALCONSOLE.print(settings.CellGap * 1000, 0);
      SERIALCONSOLE.println("mV");
      SERIALCONSOLE.print("3 - Temp Warning Offset: ");
      SERIALCONSOLE.print(settings.WarnToff);
      SERIALCONSOLE.println(" C");
      // SERIALCONSOLE.print("4 - Temp Warning delay: ");
      // SERIALCONSOLE.print(settings.UnderDur);
      // SERIALCONSOLE.println(" mS");
      SERIALCONSOLE.print("4 - Over and Under Voltage Delay: ");
      SERIALCONSOLE.print(settings.triptime);
      SERIALCONSOLE.println(" mS");

      menuload = 7;
      break;

    case 'k': // contactor settings
      while (Serial.available())
      {
        Serial.read();
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Contactor and Gauge Settings Menu");
      SERIALCONSOLE.print("1 - PreCharge Timer: ");
      SERIALCONSOLE.print(settings.Pretime);
      SERIALCONSOLE.println("mS");
      SERIALCONSOLE.print("2 - PreCharge Finish Current: ");
      SERIALCONSOLE.print(settings.Precurrent);
      SERIALCONSOLE.println(" mA");
      SERIALCONSOLE.print("3 - PWM contactor Hold 0 - 255 : ");
      SERIALCONSOLE.println(settings.conthold);
      SERIALCONSOLE.print("4 - PWM for Gauge Low 0 - 255 : ");
      SERIALCONSOLE.println(settings.gaugelow);
      SERIALCONSOLE.print("5 - PWM for Gauge High 0 - 255 : ");
      SERIALCONSOLE.println(settings.gaugehigh);
      if (settings.ESSmode == 1)
      {
        SERIALCONSOLE.print("6 - ESS Main Contactor or Trip : ");
        if (settings.tripcont == 0)
        {
          SERIALCONSOLE.println("Trip Shunt");
        }
        else
        {
          SERIALCONSOLE.println("Main Contactor and Precharge");
        }
        SERIALCONSOLE.print("7 - External Battery Enable : ");
        switch (settings.ChargerDirect)
        {
        case 0:
          SERIALCONSOLE.print(" Enable In2");
          break;
        case 1:
          SERIALCONSOLE.print("Auto Start");
          break;
        }
      }

      menuload = 5;
      break;

    case 113:                  // q to go back to main menu
      EEPROM.put(0, settings); // save all change to eeprom
      menuload = 0;
      debug = 1;
      break;
    case 'd': // d for debug settings
      while (Serial.available())
      {
        Serial.read();
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Debug Settings Menu");
      SERIALCONSOLE.println("Toggle on / off");
      SERIALCONSOLE.print("1 - Can Debug : ");
      SERIALCONSOLE.println(candebug);
      SERIALCONSOLE.print("2 - Current Debug : ");
      SERIALCONSOLE.println(debugCur);
      SERIALCONSOLE.print("3 - Output Check : ");
      SERIALCONSOLE.println(outputcheck);
      SERIALCONSOLE.print("4 - Input Check : ");
      SERIALCONSOLE.println(inputcheck);
      SERIALCONSOLE.print("5 - ESS mode : ");
      SERIALCONSOLE.println(settings.ESSmode);
      SERIALCONSOLE.print("6 - Cells Present Reset : ");
      SERIALCONSOLE.println(cellspresent);
      SERIALCONSOLE.print("7 - Gauge Debug : ");
      SERIALCONSOLE.println(gaugedebug);
      SERIALCONSOLE.print("8 - CSV Output : ");
      SERIALCONSOLE.println(CSVdebug);
      SERIALCONSOLE.print("9 - Decimal Places to Show : ");
      SERIALCONSOLE.println(debugdigits);
      SERIALCONSOLE.print("d - CSV Delimiter : ");
      if (delim == 1)
      {
        SERIALCONSOLE.println("Space");
      }
      else
      {
        SERIALCONSOLE.println("Comma");
      }
      SERIALCONSOLE.println("q - Go back to menu");
      menuload = 4;
      break;

    case 99: // c for calibrate zero offset
      while (Serial.available())
      {
        Serial.read();
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Current Sensor Calibration Menu");
      SERIALCONSOLE.println("c - To calibrate sensor offset");
      SERIALCONSOLE.print("s - Current Sensor Type : ");
      switch (settings.cursens)
      {
      case CURR_SENSE_ANALOGUE_DUAL:
        SERIALCONSOLE.println(" Analogue Dual Current Sensor ");
        break;
      case CURR_SENSE_ANALOGUE_GUESSING:
        SERIALCONSOLE.println(" Analogue Single Current Sensor ");
        break;
      case CURR_SENSE_CANBUS:
        SERIALCONSOLE.println(" Canbus Current Sensor ");
        break;
      default:
        SERIALCONSOLE.println("Undefined");
        break;
      }
      SERIALCONSOLE.print("1 - invert current : ");
      SERIALCONSOLE.println(settings.invertcur);
      SERIALCONSOLE.print("2 - Pure Voltage based SOC : ");
      SERIALCONSOLE.println(settings.voltsoc);
      SERIALCONSOLE.print("3 - Current Multiplication : ");
      SERIALCONSOLE.println(settings.ncur);
      if (settings.cursens == CURR_SENSE_ANALOGUE_GUESSING || settings.cursens == CURR_SENSE_ANALOGUE_DUAL)
      {
        SERIALCONSOLE.print("4 - Analogue Low Range Conv : ");
        SERIALCONSOLE.print(settings.convlow * 0.01, 2);
        SERIALCONSOLE.println(" mV / A");
      }
      if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL)
      {
        SERIALCONSOLE.print("5 - Analogue High Range Conv : ");
        SERIALCONSOLE.print(settings.convhigh * 0.01, 2);
        SERIALCONSOLE.println(" mV / A");
      }
      if (settings.cursens == CURR_SENSE_ANALOGUE_GUESSING || settings.cursens == CURR_SENSE_ANALOGUE_DUAL)
      {
        SERIALCONSOLE.print("6 - Current Sensor Deadband : ");
        SERIALCONSOLE.print(settings.CurDead);
        SERIALCONSOLE.println(" mV");
      }
      if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL)
      {

        SERIALCONSOLE.print("8 - Current Channel ChangeOver : ");
        SERIALCONSOLE.print(settings.changecur * 0.001);
        SERIALCONSOLE.println(" A");
      }

      if (settings.cursens == CURR_SENSE_CANBUS)
      {
        SERIALCONSOLE.print("7 - Can Current Sensor : ");
        if (settings.curcan == LemCAB300)
        {
          SERIALCONSOLE.println(" LEM CAB300 / 500 series ");
        }
        else if (settings.curcan == LemCAB500)
        {
          SERIALCONSOLE.println(" LEM CAB500 Special ");
        }
        else if (settings.curcan == IsaScale)
        {
          SERIALCONSOLE.println(" IsaScale IVT - S ");
        }
        else if (settings.curcan == VictronLynx)
        {
          SERIALCONSOLE.println(" Victron Lynx VE.CAN Shunt");
        }
      }
      SERIALCONSOLE.println("q - Go back to menu");
      menuload = 2;
      break;

    case 98: // c for calibrate zero offset
      while (Serial.available())
      {
        Serial.read();
      }
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.println("Battery Settings Menu");
      SERIALCONSOLE.println("r - Reset AH counter");
      SERIALCONSOLE.println("f - Reset to Coded Settings");
      SERIALCONSOLE.println("q - Go back to menu");
      SERIALCONSOLE.println();
      SERIALCONSOLE.println();
      SERIALCONSOLE.print("1 - Cell Over Voltage Setpoint : ");
      SERIALCONSOLE.print(settings.OverVSetpoint * 1000, 0);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("2 - Cell Under Voltage Setpoint : ");
      SERIALCONSOLE.print(settings.UnderVSetpoint * 1000, 0);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("3 - Over Temperature Setpoint : ");
      SERIALCONSOLE.print(settings.OverTSetpoint);
      SERIALCONSOLE.print("C");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("4 - Under Temperature Setpoint : ");
      SERIALCONSOLE.print(settings.UnderTSetpoint);
      SERIALCONSOLE.print("C");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("5 - Cell Balance Voltage Setpoint : ");
      SERIALCONSOLE.print(settings.balanceVoltage * 1000, 0);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("6 - Balance Voltage Hystersis : ");
      SERIALCONSOLE.print(settings.balanceHyst * 1000, 0);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("7 - Ah Battery Capacity : ");
      SERIALCONSOLE.print(settings.CAP);
      SERIALCONSOLE.print("Ah");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("8 - Pack Max Discharge : ");
      SERIALCONSOLE.print(settings.discurrentmax * 0.1);
      SERIALCONSOLE.print("A");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("9 - Cell Discharge Voltage Limit Setpoint : ");
      SERIALCONSOLE.print(settings.DischVsetpoint * 1000, 0);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("0 - Slave strings in parallel : ");
      SERIALCONSOLE.print(settings.Pstrings);
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("a - Cells in Series per String : ");
      SERIALCONSOLE.print(settings.Scells);
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("b - setpoint 1 : ");
      SERIALCONSOLE.print(settings.socvolt[0]);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("c - SOC setpoint 1 : ");
      SERIALCONSOLE.print(settings.socvolt[1]);
      SERIALCONSOLE.print(" % ");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("d - setpoint 2 : ");
      SERIALCONSOLE.print(settings.socvolt[2]);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("e - SOC setpoint 2 : ");
      SERIALCONSOLE.print(settings.socvolt[3]);
      SERIALCONSOLE.print(" % ");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("g - Storage Setpoint : ");
      SERIALCONSOLE.print(settings.StoreVsetpoint * 1000, 0);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("h - Discharge Current Taper Offset : ");
      SERIALCONSOLE.print(settings.DisTaper * 1000, 0);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("j - Discharge Current Temperature Derate : ");
      SERIALCONSOLE.print(settings.DisTSetpoint);
      SERIALCONSOLE.print("C");
      SERIALCONSOLE.println("  ");
      SERIALCONSOLE.print("k - Cell Discharge Voltage Hysteresis : ");
      SERIALCONSOLE.print(settings.DischHys * 1000, 0);
      SERIALCONSOLE.print("mV");
      SERIALCONSOLE.println("  ");

      SERIALCONSOLE.println();
      menuload = 3;
      break;

    default:
      // if nothing else matches, do the default
      // default is optional
      break;
    }
  }

  if (incomingByte == 115 && menuload == 0)
  {
    SERIALCONSOLE.println();
    SERIALCONSOLE.println("MENU");
    SERIALCONSOLE.println("Debugging Paused");
    SERIALCONSOLE.print("Firmware Version : ");
    SERIALCONSOLE.println(firmver);
    SERIALCONSOLE.println("b - Battery Settings");
    SERIALCONSOLE.println("a - Alarm and Warning Settings");
    SERIALCONSOLE.println("e - Charging Settings");
    SERIALCONSOLE.println("c - Current Sensor Calibration");
    SERIALCONSOLE.println("k - Contactor and Gauge Settings");
    SERIALCONSOLE.println("i - Ignore Value Settings");
    SERIALCONSOLE.println("d - Debug Settings");
    SERIALCONSOLE.println("x - Experimental Settings");
    SERIALCONSOLE.println("R - Restart BMS");
    SERIALCONSOLE.println("q - exit menu");
    debug = 0;
    menuload = 1;
  }
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
