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
#include "NextionDisplay.h"
#include "BMSLogic.h"
#include "StatusAndLogging.h"
#include "BMS_SD.h"
#include "BMS_RTC.h"

// Libraries
#include <Arduino.h>
#include <ADC.h> //https://github.com/pedvide/ADC
#include <EEPROM.h>
#include <FlexCAN_T4.h> //https://github.com/collin80/FlexCAN_Library
#include <SPI.h>
#include <Filters.h>                  //https://github.com/JonHub/Filters
#include <Serial_CAN_Module_Teensy.h> //https://github.com/tomdebree/Serial_CAN_Teensy (https://github.com/Longan-Labs/Serial_CAN_Arduino)

// T4 Additions
#include <Watchdog_t4.h>
#include <imxrt.h>
#include <CrashReport.h>

/////Version Identifier/////////
int firmver = 250823; // Year Month Day
const char *COMPILE_DATE = __DATE__;
const char *COMPILE_TIME = __TIME__;

WDT_T4<WDT1> watchdog;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

// These two appear unused
// Serial_CAN can;
// SerialConsole console; // TODO - Appears unused?

BMSModuleManager bms;
EEPROMSettings settings;

ADC *adc = new ADC(); // adc object

CAN_message_t msg;
CAN_message_t inMsg;

// Prototypes
void alarmupdate();
void printbmsstat();
void updateSOC();
void SOCcharged(int y);
void Prechargecon();
void contactorControl();
void VEcan();
int pgnFromCANId(int canId);
bool canRead();
void Rx309();
void CAB300();
void CAB500();
void handleVictronLynx();
void currentlimit();
void inputdebug();
void outputdebug();
void resetwdog();
void pwmcomms();
void balancing();
void chargercomms();
void isrCP();

void setup()
{
  Logger::setLoglevel(Logger::Info); // Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4



  indicatorsSetup();

  analogWrite(PIN_HEARTBEAT_LED, 50);

  // ------------- Start Serial Busses -------------
  SERIALBMS.begin(612500); // Tesla serial bus

  SERIAL_CONSOLE.begin(500000);
  SERIAL_CONSOLE.println("eChook Comms Master BMS - Tesla");
  SERIAL_CONSOLE.println("Starting up!");

  
  SERIAL_AUX.begin(115200); // display and can adpater canbus
  delay(2000);              // just for easy debugging. It takes a few seconds for USB to come up properly on most OS's
  Serial.println("Serial busses started, plus 2 second delay");

  if (CrashReport)
  {
    // TODO - Log to SD card
    Logger::error("Crash detected, printing report:");
    SERIAL_CONSOLE.print(CrashReport);
  }

  // ------------- Pin Setup -------------
  pinSetup();

  // ------------- RTC Setup -------------
  RTCSetup();

  // ------------- SD Card Setup -------------
  SDInit();

  // ------------- EEPROM Setting Retreival -------------

  /**
   * A Note on Settings
   * 
   * They do not appear to be saved to the EEPROM, except when changing settings in the 
   * Serial Menu. There is no automatic saving on the first boot that I can find,
   * as would be implied by testing the settings version vs the eeprom version
   */
  Serial.println("Loading settings from EEPROM");
  EEPROM.get(0, settings);
  if (settings.version != EEPROM_VERSION)
  {
    loadSettings();
  }

  // ------------- CAN Setup -------------
  Serial.println("Starting CAN bus Can1 at " + String(settings.canSpeed) + " baud");
  Can1.begin();
  Can1.setBaudRate(settings.canSpeed);

  Serial.println("Starting CAN bus Can2 at " + String(settings.canSpeed) + " baud");
  Can2.begin();
  Can2.setBaudRate(settings.canSpeed);

  Serial.println("Starting CAN bus Can3 at " + String(settings.canSpeed) + " baud");
  Can3.begin();
  Can3.setBaudRate(settings.canSpeed);

 

  //  Enable WDT T4.x
  WDT_timings_t configewm;
  configewm.timeout = 5; // seconds
  configewm.pin = PIN_ERROR_LED;
  configewm.callback = watchdogCallback;
  watchdog.begin(configewm);
  delay(100);
  watchdog.feed();

  delay(100); /* <-- not keeping this here would cause resets by Callback */

  // VE.begin(19200); //Victron VE direct bus


  moduleSetup();

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

  // // Blink Heartbeat LED to indicate setup is complete
  // digitalWrite(PIN_HEARTBEAT_LED, HIGH); // Turn off heartbeat LED
  // delay(500);                            // Wait for 1 second
  digitalWrite(PIN_HEARTBEAT_LED, LOW); // Turn off heartbeat LED

  // bmsstatus = BMS_STATUS_BOOT;
  if (bmsError == ERROR_NONE)
  {
    setBMSstatus(BMS_STATUS_BOOT, "Setup complete");
    digitalWrite(PIN_LED_BUILTIN, LOW);
    bmsError = ERROR_NONE;
    Logger::info("Setup complete, entering main loop");
  }
  else
  {
    Logger::info("Setup complete with ERRORS, entering main loop");
  }
}

void loop()
{

  resetwdog();

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

  if (modulesConnected)
  {
    // Runs Periodic checks and updates BMS readings every 500ms
    bmsLoop();

    // Serial.printf("Loop: If output check. Outputcheck: %d, bmsStatus: %d\n", outputcheck, bmsstatus);
    outputCheck();
  }
  else
  {
    // Try to reconnect to modules periodically
    static unsigned long nextModuleCheck = millis();
    if (millis() > nextModuleCheck)
    {
      nextModuleCheck += 5000; // every 5 seconds
      moduleSetup();
    }
  }

} // End of loop

void moduleSetup()
{
  Logger::info("Starting Communication with Battery Modules");
  Logger::debug("Renumbering BOARD IDs");
  bms.renumberBoardIDs();

  lastUpdate = 0;
  Logger::debug("Finding BMS boards");
  bms.findBoards();

  bms.setPstrings(settings.Pstrings);
  bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);

  Logger::debug("Number of Modules Found: %i", bms.getNumModules());

  if (bms.getNumModules() == 0)
  {
    Logger::error("No modules found - Check connections to pack");
    setBMSstatus(BMS_STATUS_ERROR, ERROR_BATTERY_COMMS, "No modules found - Check connections to pack");
    modulesConnected = 0;
  } else if (bms.seriescells() != settings.Scells)
  {
    Logger::error("Number of cells in pack does not match settings");
    Logger::error("Detected: %d, Expected: %d", bms.seriescells(), settings.Scells);
    setBMSstatus(BMS_STATUS_ERROR, ERROR_BATTERY_COMMS, "Number of cells in pack does not match settings");
  }
}

/**
 * @brief Sets up all the required pins as inputs or outputs
 *
 * This function sets the pin modes for all the digital inputs and outputs,
 * and sets all the outputs low on boot. It also sets the PWM frequency for
 * the desired pins.
 *
 * @note This function is called by setup() at startup.
 */
void pinSetup()
{
  Serial.println("Setting up pins");
  // ------------- Pin Mode Assignments -------------
  // pinMode(ACUR1, INPUT);//Not required for Analogue Pins
  // pinMode(ACUR2, INPUT);//Not required for Analogue Pins
  pinMode(PIN_IN1, INPUT_PULLDOWN);
  pinMode(PIN_IN2, INPUT_PULLDOWN);
  pinMode(PIN_IN3, INPUT_PULLDOWN);
  pinMode(PIN_IN4, INPUT_PULLDOWN);
  pinMode(PIN_OUT1, OUTPUT); // Positive contactor
  pinMode(PIN_OUT2, OUTPUT); // precharge
  pinMode(PIN_OUT3, OUTPUT); // charge relay
  pinMode(PIN_OUT4, OUTPUT); // Negative contactor
  pinMode(PIN_OUT5, OUTPUT); // pwm driver output
  pinMode(PIN_OUT6, OUTPUT); // pwm driver output
  pinMode(PIN_OUT7, OUTPUT); // pwm driver output
  pinMode(PIN_OUT8, OUTPUT); // pwm driver output

  // ------------- Set all outputs low on boot -------------
  digitalWrite(PIN_OUT1, LOW);
  digitalWrite(PIN_OUT2, LOW);
  digitalWrite(PIN_OUT3, LOW);
  digitalWrite(PIN_OUT4, LOW);
  digitalWrite(PIN_OUT5, LOW);
  digitalWrite(PIN_OUT6, LOW);
  digitalWrite(PIN_OUT7, LOW);
  digitalWrite(PIN_OUT8, LOW);

  // ------------- PWM Output Configuration -------------
  analogWriteFrequency(PIN_OUT5, pwmfreq);
  analogWriteFrequency(PIN_OUT6, pwmfreq);
  analogWriteFrequency(PIN_OUT7, pwmfreq);
  analogWriteFrequency(PIN_OUT8, pwmfreq);

  adc->adc0->setAveraging(16);  // set number of averages
  adc->adc0->setResolution(16); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED);
  adc->adc0->startContinuous(PIN_ACUR_1);
}


/**
 * @brief Updates the State of Charge (SOC) based on current measurements and voltage readings.
 *
 * This function calculates the SOC of the battery pack using either current integration
 * (amp-seconds) or voltage-based estimation, depending on the configuration settings.
 * It also handles initial SOC setting from memory or voltage after a delay, and ensures
 * SOC remains within valid bounds (0-100%).
 *
 * The function updates the `SOC` variable and `ampsecond` variable, which represents
 * the total charge in milliamp-hours. It also provides debug output if enabled.
 *
 * Key features:
 * - Initial SOC setting from memory or voltage after 5 seconds.
 * - Voltage-based SOC estimation if configured.
 * - Current integration for SOC calculation if current sensing is enabled.
 * - Ensures SOC does not exceed 100% or drop below 0%.
 * - Debug output for current, SOC, and amp-seconds.
 */
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

/*
 * @brief Updates the State of Charge (SOC) to a charged state.
 *
 *  This function sets the SOC to either 95% or 100% based on the input parameter.
 *  It also recalculates the ampsecond value based on the battery capacity and number of  strings.
 * @param y An integer indicating the desired charged state:
 *          - If y is 1, SOC is set to 95%.
 */
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

/**
 * @brief Handles the precharge state of the BMS.
 *
 * This function checks the states of the key switch and AC present inputs and controls the contactors
 * accordingly. If the key switch is ON or AC is present, it enables the precharge contactor and then the
 * main contactor. If the charger is set to direct mode, it goes to the drive state after precharge.
 * Otherwise, it goes to the charge or drive state depending on whether AC is present or the key switch is
 * ON.
 *
 * If the inputs are not active, it disables all contactors and sets the BMS status to READY.
 */
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

/**
 * @brief Controls the contactors based on the desired state.
 *
 * This function manages the state of the contactors by comparing the desired
 * control state (`contctrl`) with the current status (`contstat`). It handles
 * the activation and deactivation of contactors with appropriate timing to ensure
 * safe operation.
 *
 * The function uses timers (`conttimer1`, `conttimer2`, `conttimer3`) to manage
 * the pull-in time for each contactor, ensuring that they are energized for a
 * specified duration (`pulltime`) before switching to a holding current defined
 * in the settings.
 *
 * Contactors are controlled via PWM signals on specific output pins:
 * - PIN_OUT5 for contactor 1
 * - PIN_OUT6 for contactor 2
 * - PIN_OUT7 for contactor 3
 *
 * The function also ensures that if no contactors are requested (`contctrl` is 0),
 * all contactors are de-energized.
 */
void contactorControl()
{
  Logger::debug("Entering contactorControl with contctrl=%d, contstat=%d", contctrl, contstat);
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
          // start pull-in timer for contactor 1
          analogWrite(PIN_OUT5, 255);
          conttimer1 = millis() + pulltime;
        }
        if (conttimer1 < millis())
        {
          // switch to holding current after pull-in timer expires
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
          // start pull-in timer for contactor 2
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
          // switch to holding current after pull-in timer expires
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
          // start pull-in timer for contactor 3
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
          // switch to holding current after pull-in timer expires
          analogWrite(PIN_OUT7, settings.conthold);
          contstat = contstat | 4;
          conttimer3 = 0;
        }
      }
    }
  }
  if (contctrl == 0)
  {
    // de-energize all contactors if no contactors are requested
    analogWrite(PIN_OUT5, 0);
    analogWrite(PIN_OUT6, 0);
  }
  Logger::debug("Leaving contactorControl with contctrl=%d, contstat=%d", contctrl, contstat);
}

/**
 * @brief Handles communication with the Victron system over CAN bus.
 *
 * This function constructs and sends several CAN messages to communicate
 * the current status of the BMS to a Victron system. It sends information
 * such as charge and discharge voltage setpoints, current limits, state of
 * charge (SOC), state of health (SOH), pack voltage, current, temperature,
 * alarms, warnings, and identification details.
 *
 * The function uses the `Can1` object to send messages with specific IDs
 * and data formats expected by Victron systems. It also includes brief
 * delays between sending messages to ensure proper timing on the CAN bus.
 *
 * The messages sent include:
 * - 0x351: Charge and discharge setpoints and current limits
 * - 0x355: SOC and SOH information
 * - 0x356: Pack voltage, current, and temperature
 * - 0x35A: Alarm and warning status
 * - 0x35E: BMS name identification
 * - 0x370: BMS manufacturer identification
 * - 0x373: Low cell voltage and temperature information
 */
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

/*
 * @brief Extracts the PGN (Parameter Group Number) from a given CAN ID.
 *
 * This function takes a CAN ID as input and determines whether it is an extended
 * or standard ID. If it is an extended ID, it extracts the PGN by applying a mask
 * and shifting the bits accordingly. If it is a standard ID, it simply returns
 * the CAN ID as is.
 *
 * @param canId The CAN ID from which to extract the PGN.
 * @return The extracted PGN if the CAN ID is extended; otherwise, returns the original CAN ID.
 */
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

/**
 * @brief Reads and processes incoming CAN messages.
 *
 * This function checks for incoming CAN messages using the `Can1` object.
 * If a message is received, it processes the message based on its ID and
 * performs various actions depending on the configuration settings.
 *
 * The function handles messages for current sensing via CAN bus, including
 * support for different current sensors (CAB300, CAB500, Jaguar I-Pace ISA shunt).
 * It also processes a specific message with ID 0x309 to manage contactor requests.
 *
 * If debugging is enabled, the function prints detailed information about
 * the received CAN messages to the serial console.
 *
 * @return true if a message was read and processed; false if no message was available.
 */
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

/**
 * @brief Processes incoming CAN message with ID 0x309 to manage contactor requests.
 *
 * This function checks if the State of Charge (SOC) has been set. If it has,
 * it examines the first byte of the incoming CAN message buffer. If the least
 * significant bit (bit 0) is set, it indicates a request to turn on the contactor,
 * setting `CanOnReq` to true and updating the `CanOntimeout` timestamp. If the bit
 * is not set, it indicates a request to turn off the contactor, setting `CanOnReq`
 * to false while still updating `CanOntimeout`.
 *
 * The function ensures that contactor requests are only processed when the SOC
 * has been initialized.
 */
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

/**
 * @brief Processes CAN messages from a CAB300 current sensor.
 *
 * This function reads a 4-byte current measurement from the incoming CAN message
 * buffer, combines the bytes into a single integer value, and converts it to milliamps.
 * It handles both positive and negative current values based on the sensor's output format.
 * If the current sensing method is set to CAN bus, it updates the raw current value
 * and calls the `getcurrent()` function to process it further.
 *
 * If CAN debugging is enabled, the function prints the received current value in
 * both hexadecimal and decimal formats to the serial console.
 */
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

/**
 * @brief Processes CAN messages from a CAB500 current sensor.
 *
 * This function reads a 3-byte current measurement from the incoming CAN message
 * buffer, combines the bytes into a single integer value, and converts it to milliamps.
 * It handles both positive and negative current values based on the sensor's output format.
 * If the current sensing method is set to CAN bus, it updates the raw current value
 * and calls the `getcurrent()` function to process it further.
 *
 * If CAN debugging is enabled, the function prints the received current value in
 * both hexadecimal and decimal formats to the serial console.
 */
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

/**
 * @brief Handles CAN messages from a Victron Lynx current sensor.
 *
 * This function processes incoming CAN messages specifically formatted for
 * Victron Lynx current sensors. It checks for invalid data (0xffff) and
 * extracts the current value from the message buffer. The current is then
 * converted to milliamps and, if the current sensing method is set to CAN bus,
 * updates the raw current value and calls the `getcurrent()` function.
 *
 * If CAN debugging is enabled, the function prints the received current value
 * in milliamps to the serial console.
 */
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

/**
 * @brief Calculates and sets the charge and discharge current limits based on various conditions.
 *
 * This function evaluates the current status of the Battery Management System (BMS)
 * and adjusts the charge and discharge current limits accordingly. It considers factors
 * such as temperature, voltage, state of charge (SOC), and predefined settings to ensure
 * safe operation of the battery system.
 *
 * The function first checks if the BMS is in an error state, in which case both
 * charge and discharge currents are set to zero. If the BMS is operational, it starts
 * with maximum allowed currents and applies derating based on temperature and voltage
 * thresholds. It also ensures that no negative current values are set.
 *
 * Additionally, if there is an AC current limit from a control pilot, it calculates
 * the maximum allowable charge power and adjusts the charge current accordingly.
 */
void currentlimit()
{
  Logger::debug("Entering Current Limit");
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
  Logger::debug("Exiting Current Limit");
}

/**
 * @brief Debugging function to print the state of input pins.
 *
 * This function reads the state of four input pins (PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4)
 * and prints their status (ON or OFF) to the serial console. It is useful for
 * debugging purposes to verify the state of these inputs during operation.
 */
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

/**
 * @brief Debugging function to toggle output pins in a specific pattern.
 *
 * This function toggles the state of eight output pins (PIN_OUT1 to PIN_OUT8)
 * in a specific pattern for debugging purposes. It uses a static variable
 * `outputstate` to track the current state and changes the pin states
 * accordingly. The function increments the `outputstate` variable and resets
 * it after reaching a certain threshold to create a repeating pattern.
 */
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

/**
 * @brief Resets the watchdog timer to prevent system reset.
 *
 * This function feeds the watchdog timer to prevent it from expiring
 * and causing a system reset. It is typically called periodically
 * within the main loop or critical sections of code to ensure
 * the system remains operational.
 */
void resetwdog()
{
  static unsigned long nextFeed = millis();
  if (millis() > nextFeed)
  {
    // Logger::debug("Feeding watchdog");
    nextFeed = millis() + 1000; // Feed the watchdog every second
    watchdog.feed();
  }
}

// Triggered on watchdog feed?
void watchdogCallback()
{
  // Logger::debug("Watchdog triggered - resetting...");
}

/**
 * @brief Generates PWM signals for communication with external devices.
 *
 * This function generates PWM signals on two output pins (PIN_OUT7 and PIN_OUT8)
 * based on the current state of the battery system. The PWM signal on PIN_OUT7
 * is mapped to the actual current draw, while the signal on PIN_OUT8 is mapped
 * to the state of charge (SOC) of the battery.
 *
 * If the lowest cell voltage drops below a predefined under-voltage setpoint,
 * the function sets the PWM signal on PIN_OUT7 to a fixed value (255) to indicate
 * a critical condition. Otherwise, it adjusts the PWM signal on PIN_OUT8 based
 * on the SOC, providing a voltage range from 2V to 10V.
 *
 * Unclear what system this is intended to communicate with.
 *
 */
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

/**
 * @brief Manages cell balancing based on the balancecells setting.
 *
 * This function checks the `balancecells` setting to determine whether
 * cell balancing should be enabled or disabled. If balancing is enabled
 * (i.e., `balancecells` is set to 1), it calls the `balanceCells` method
 * of the BMS object with the specified balance duty cycle. If balancing
 * is disabled, it stops the balancing process by calling the `StopBalancing`
 * method of the BMS object.
 *
 * The function includes a debug mode that can be activated by setting
 * the `debug` variable to 1, although in this implementation, both
 * debug and non-debug modes perform the same action.
 */
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

/**
 * @brief Communicates with the charger via CAN bus to set charging parameters.
 *
 * This function sends CAN messages to various types of chargers (Elcon, Eltek,
 * Brusa NLG5, Chevy Volt) to set charging parameters such as voltage and current
 * limits. The specific message format and content depend on the type of charger
 * configured in the `settings.chargertype`.
 *
 * For each charger type, the function constructs a CAN message with the appropriate
 * ID, length, and data bytes, then sends the message using the `Can1.write` method.
 * The function also includes some conditional logic to handle different scenarios,
 * such as generator status for the Brusa NLG5 charger.
 */
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

/**
 * @brief Interrupt Service Routine (ISR) for the Control Pilot (CP) signal.
 *
 * This function is triggered on any change (RISING or FALLING edge) of the digital input pin `PIN_IN4`,
 * which is connected to the J1772 Control Pilot signal from an EVSE (Electric Vehicle Supply Equipment).
 * It calculates the duty cycle of the CP's PWM signal to determine the maximum AC current the EVSE can provide.
 *
 * On a FALLING edge (pin goes LOW), it measures the total period of the PWM signal (`duration`)
 * and resets the `pilottimer`.
 *
 * On a RISING edge (pin goes HIGH), it calculates the available AC current limit (`accurlim`) based on the
 * measured period and the pulse width. The formula used is a variation of the J1772 standard calculation,
 * which typically relates duty cycle to available current (Current = Duty Cycle * 0.6A).
 *
 * @note This is an ISR and should execute as quickly as possible. It uses `micros()` for high-resolution timing.
 * @global long duration Stores the period of the CP PWM signal in microseconds.
 * @global long pilottimer Stores the timestamp of the last FALLING edge.
 * @global int accurlim The calculated maximum AC current available from the EVSE, which is used to limit charging power.
 */
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

// Not currently used - intended for low voltage interrupt handling on T3.2, need to reimplement for T4

// void low_voltage_isr(void)
// {
//   EEPROM.update(1000, uint8_t(SOC));

//   PMC_LVDSC2 |= PMC_LVDSC2_LVWACK; // clear if we can
//   PMC_LVDSC1 |= PMC_LVDSC1_LVDACK;
// }
