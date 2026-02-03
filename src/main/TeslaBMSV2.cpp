/**
 * @file TeslaBMSV2.ino
 * @brief Entry Point for the project.
 *
 * Adapted from the original SimpBMS Arduino project
 * with setup() and loop() functions. *
 */

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
#include "config/pinouts.h"
#include "config/globals.h"
#include "config/Settings.h"
#include "display/indicators.h"
#include "battery/BMSModuleManager.h"
#include "config/config.h"
#include "interface/SerialConsole.h"
#include "storage/Logger.h"
#include "interface/SerialMenu.h"
#include "power/CurrentSensing.h"
#include "display/PhysicalGauges.h"
#include "display/NextionDisplay.h"
#include "main/BMSLogicV2.h"
#include "storage/StatusAndLogging.h"
#include "storage/BMS_SD.h"
#include "storage/BMS_RTC.h"
#include "display/BMS_OLED.h"
#include "power/BMS_Contactor.h"



// Libraries
#include <Arduino.h>
#include "TeensyDebug.h"
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

// Comms Modules
#include "comms/CanBusManager.h"
#include "comms/sensors/CAB500.h"
#include "comms/sensors/CAB300.h"
#include "comms/sensors/IsaScale.h"
#include "comms/chargers/Charger.h"
#include "comms/chargers/ElconCharger.h"
#include "comms/chargers/ChevyVoltCharger.h"
#include "comms/chargers/EltekCharger.h"
#include "comms/chargers/BrusaCharger.h"
#include "comms/chargers/CodaCharger.h"
#include "comms/emulators/VictronEmulator.h"
#include <vector>
#include <functional>

/////Version Identifier/////////
int firmver = 250823; // Year Month Day
const char *COMPILE_DATE = __DATE__;
const char *COMPILE_TIME = __TIME__;

WDT_T4<WDT1> watchdog;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

BMS_Contactor positive(POSITIVE, H1, BuiltIn);
BMS_Contactor precharge(PRECHARGE, H2, BuiltIn);
BMS_Contactor charge(CHARGE, H3, BuiltIn);
BMS_Contactor negative(NEGATIVE, H4, BuiltIn);
BMS_Contactor trip(0); // Unconfigured

struct Contactors contactors = {positive, precharge, charge, negative, trip};

// These two appear unused
// Serial_CAN can;
// SerialConsole console; // TODO - Appears unused?

BMSModuleManager bms;
EEPROMSettings settings;

ADC *adc = new ADC(); // adc object

CAN_message_t msg;
CAN_message_t inMsg;

// -----------------------------------------------------------
// Modular CAN Architecture Setup
// -----------------------------------------------------------

// Forward declaration for legacy handler
void Rx309();

// Helper wrappers to create function pointers from FlexCAN_T4 member functions
bool can1Read(CAN_message_t &msg) { return Can1.read(msg); }
int can1Write(const CAN_message_t &msg) { return Can1.write(msg); }

bool can2Read(CAN_message_t &msg) { return Can2.read(msg); }
int can2Write(const CAN_message_t &msg) { return Can2.write(msg); }

bool can3Read(CAN_message_t &msg) { return Can3.read(msg); }
int can3Write(const CAN_message_t &msg) { return Can3.write(msg); }

// One manager per CAN bus
CanBusManager can1Manager(can1Read, can1Write);
CanBusManager can2Manager(can2Read, can2Write);
CanBusManager can3Manager(can3Read, can3Write);

// Active device pointers
CurrentSensor* activeSensor = nullptr;
Charger* activeCharger = nullptr;
VictronEmulator* victronEmulator = nullptr;

// 3. Define Legacy Listener (Adapter)
// Currently: Contactors (0x309)
class LegacyCanListener : public CanListener 
{
public:
    bool onReceive(const CAN_message_t &msg) override 
    {
        // Update global 'inMsg' which legacy functions rely on
        inMsg = msg; 

        // Handle Contactor Request
        if (msg.id == 0x309) {
            Rx309();
            return true;
        }
        
        return false;
    }
};

LegacyCanListener legacyListener;

// -----------------------------------------------------------
// Setup Helper
// -----------------------------------------------------------
void setupCanDevices()
{
    // Lambda to bridge back to legacy processing
    auto updateCallback = [](float amps) {
         extern void processCurrentValue(int32_t rawInput);
         processCurrentValue((int32_t)(amps * 1000.0f));
    };

    // =========================================================
    // PROTOTYPE: Multi-bus demonstration
    // - CAB500 sensor on CAN1
    // - Chargers on CAN1
    // - Victron Emulator output on CAN3
    // =========================================================

    // A. Setup Current Sensor on CAN1
    if (settings.cursens == CURR_SENSE_CANBUS)
    {
        switch(settings.curcan) {
            case 1: // CAB300
                activeSensor = new CAB300(updateCallback);
                can1Manager.registerListener(activeSensor);
                Logger::info("CAB300 sensor on CAN1");
                break;
            case 2: // CAB500
                activeSensor = new CAB500(updateCallback);
                can1Manager.registerListener(activeSensor);
                Logger::info("CAB500 sensor on CAN1");
                break;
            case 3: // IsaScale
                activeSensor = new IsaScaleSensor(updateCallback);
                can1Manager.registerListener(activeSensor);
                Logger::info("IsaScale sensor on CAN1");
                break;
        }
    }

    // B. Setup Charger on CAN1 (default for now)
    auto can1SendCallback = [](const CAN_message_t& m){ can1Write(m); };

    switch(settings.chargertype) {
        case Elcon:
            activeCharger = new ElconCharger(can1SendCallback);
            break;
        case ChevyVolt:
            activeCharger = new ChevyVoltCharger(can1SendCallback);
            break;
        case Eltek:
            activeCharger = new EltekCharger(can1SendCallback);
            break;
        case BrusaNLG5:
            activeCharger = new BrusaCharger(can1SendCallback);
            break;
        case Coda:
            activeCharger = new CodaCharger(can1SendCallback);
            break;
    }

    if (activeCharger) {
        can1Manager.registerListener(activeCharger);
        Logger::info("Charger on CAN1");
    }

    // C. Register Legacy Helper on CAN1
    can1Manager.registerListener(&legacyListener);

    // D. Setup Victron Emulator on CAN3
    auto can3SendCallback = [](const CAN_message_t& m){ can3Write(m); };
    victronEmulator = new VictronEmulator(can3SendCallback, bms, settings);
    Logger::info("Victron Emulator on CAN3");
}

// Prototypes needs to be updated or we just place this before setup()
// Since I am editing the block that contains "LegacyCanListener", I can put this helper here.



// Prototypes
void alarmupdate();
void printbmsstat();
void updateSOC();
void SOCcharged(int y);
void VEcan();
void Rx309();
void inputdebug();
void outputdebug();
void resetwdog();
void pwmcomms();
void balancing();
void chargercomms();
void isrCP();
void watchdogCallback();
void pinSetup();
void moduleSetup();

void setup()
{
  Logger::setSerialLoglevel(Logger::Info); // Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4
  Logger::setSdLoglevel(Logger::Info);     // Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4
  Logger::setOledLoglevel(Logger::Info);   // Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4

  indicatorsSetup();

  analogWrite(PIN_HEARTBEAT_LED, 50);

  // ------------- Start Serial Busses -------------
  SERIALBMS.begin(612500); // Tesla serial bus

  SERIAL_CONSOLE.begin(500000);
  SERIAL_CONSOLE.println(F("eChook Comms Master BMS - Tesla Modules"));
  SERIAL_CONSOLE.println(F("Starting up!"));

  Serial.println("Serial busses started");

  if (CrashReport)
  {
    // TODO - Log to SD card
    Logger::error("Crash detected, printing report:");
    SERIAL_CONSOLE.print(CrashReport);
  }

  // ------------- OLED Setup -------------
  setupOLED();

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

  // -----------------------------------------------------------
  // Register CAN Devices
  // -----------------------------------------------------------
  setupCanDevices();

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
      SOCmem = 1;
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
  attachInterrupt(PIN_EVSE_PILOT, isrCP, CHANGE); // attach BUTTON 1 interrupt handler [ pin# 21 ]

  // TODO Low/High Voltage Interrupt disabled for T4
  // PMC_LVDSC1 = PMC_LVDSC1_LVDV(1);                    // enable hi v
  // PMC_LVDSC2 = PMC_LVDSC2_LVWIE | PMC_LVDSC2_LVWV(3); // 2.92-3.08v
  // attachInterruptVector(IRQ_LOW_VOLTAGE, low_voltage_isr);
  // NVIC_ENABLE_IRQ(IRQ_LOW_VOLTAGE);

  // // Blink Heartbeat LED to indicate setup is complete
  digitalWrite(PIN_HEARTBEAT_LED, HIGH); // Turn off heartbeat LED
  delay(500);                            // Wait for 1 second
  digitalWrite(PIN_HEARTBEAT_LED, LOW);  // Turn off heartbeat LED

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

  // End of setup()
  Logger::info("Setup complete, entering main loop");
  SERIAL_CONSOLE.flush();
}

void loop()
{
  watchdog.feed();
  // Add this temporarily
  digitalWrite(PIN_LED_BUILTIN, !digitalRead(PIN_LED_BUILTIN));

  indicatorsLoop(); // Call the indicators loop to handle LED and buzzer state

  // Poll all CAN Managers
  can1Manager.poll();
  can2Manager.poll();
  can3Manager.poll();

  // Check if serial menu is requested
  if (SERIAL_CONSOLE.available() > 0)
  {
    menu();
  }

  static unsigned long nextContactorCheck = millis();
  if (millis() > nextContactorCheck)
  {
    Logger::debug("Checking contactors");
    nextContactorCheck += 1000;
    Serial.printf("Contactor Current: %f\n", negative.getPinCurrent(1));
    delay(10);
    Serial.printf("Contactor Voltage: %f\n", negative.getVoltage(1));
    Serial.printf("Contactor Temperature: %f\n", negative.getTemperature(1));
  }

  if (modulesConnected)
  {
    Logger::debug("Entering BMS loop");
    delay(500); // Give the USB host a breather
    // Runs Periodic checks and updates BMS readings every 500ms
    bmsLoop();

    Logger::debug("Entering output check");
    outputCheck();
  }
  else
  {
    // Try to reconnect to modules periodically
    static unsigned long nextModuleCheck = millis();
    if (millis() > nextModuleCheck)
    {
      Logger::debug("No modules connected, trying to find them");

      nextModuleCheck += 5000; // every 5 seconds
      moduleSetup();
    }
    if (settings.ESSmode == 1)
    {
      // bmsstatus = BMS_STATUS_READY;
      setBMSstatus(BMS_STATUS_READY, "SOC initialized from memory after 5 seconds in ESS mode");
    }
  }
}

void moduleSetup()
{
  watchdog.feed();
  Logger::info("Starting Communication with Battery Modules");
  Logger::debug("Renumbering BOARD IDs");
  bms.renumberBoardIDs();

  lastUpdate = 0;
  Logger::debug("Finding BMS boards");
  bms.findBoards();

  bms.setPstrings(settings.Pstrings);
  bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);

  bms.getAllVoltTemp();
  bms.getAvgCellVolt();

  Logger::debug("Number of Modules Found: %i", bms.getNumModules());

  if (bms.getNumModules() == 0)
  {
    Logger::error("No modules found - Check connections to pack");
    setBMSstatus(BMS_STATUS_ERROR, ERROR_BATTERY_COMMS, "No modules found - Check connections to pack");
    modulesConnected = false;
  }
  else if (bms.seriescells() != settings.Scells)
  {
    Logger::error("Number of cells in pack does not match settings");
    Logger::error("Detected: %d, Expected: %d", bms.seriescells(), settings.Scells);
    setBMSstatus(BMS_STATUS_ERROR, ERROR_BATTERY_COMMS, "Number of cells in pack does not match settings");
    modulesConnected = false;
  }
  else
  {
    Logger::debug("BMS initialised correctly", bms.getNumModules());
    setBMSstatus(BMS_STATUS_READY, ERROR_NONE, "BMS initialised correctly");
    modulesConnected = true;
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
  pinMode(PIN_IGNITION, INPUT_PULLDOWN);
  pinMode(PIN_IN2, INPUT_PULLDOWN);
  pinMode(PIN_CHARGE, INPUT_PULLDOWN);
  pinMode(PIN_EVSE_PILOT, INPUT_PULLDOWN);
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

  // adc->adc0->setAveraging(16);  // set number of averages
  // adc->adc0->setResolution(16); // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED);
  adc->adc0->startContinuous(PIN_ACUR_1);
}

void updateSOC()
{
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

  if (debugMode != 0)
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
  if (victronEmulator) {
    victronEmulator->update();
  }
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

// currentlimit() implementation moved to src/power/limits.cpp

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
  if (activeCharger) {
      // Send the control message via our modular class
      // We pass the RAW target values. The Charger implementation handles scaling/protocol.

      // Calculate target voltage
      float targetV = settings.ChargeVsetpoint * settings.Scells; 
      
      // Calculate target current
      // Logic: 'chargecurrent' in Limits.cpp is stored in 0.1A units.
      // e.g., 100 = 10.0A.
      // Our Charger Interface expects Amps (float).
      float targetC = (float)chargecurrent / 10.0f;
  
      activeCharger->sendControlMessage(targetV, targetC);
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
  if (digitalRead(PIN_EVSE_PILOT) == LOW)
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
