#pragma once

#include "config.h"

// =====================
// Default BMS Settings
// =====================

// Logging 
#define DEFAULT_LOG_LEVEL 2 // Log level

// Operation Mode
#define DEFAULT_ESSMODE 1 // ESS mode

// Battery Settings
#define DEFAULT_CAP 100     // Capacity (Ah)
#define DEFAULT_PSTRINGS 1  // Parallel strings
#define DEFAULT_SCELLS 12   // Series cells

// CAN
#define DEFAULT_CHECKSUM 2
#define DEFAULT_CAN_SPEED 500000
#define DEFAULT_BATTERY_ID 0x01

// Voltage Setpoints
#define DEFAULT_OVERV_SETPOINT 4.2f   // Overvoltage setpoint
#define DEFAULT_UNDERV_SETPOINT 3.0f  // Undervoltage setpoint
#define DEFAULT_CHARGEV_SETPOINT 4.1f // Charge voltage setpoint
#define DEFAULT_CHARGE_HYS 0.2f       // Charge voltage hysteresis
#define DEFAULT_WARN_OFF 0.1f         // Warning offset
#define DEFAULT_DISCHV_SETPOINT 3.2f  // Discharge voltage setpoint
#define DEFAULT_DISCH_HYS 0.2f        // Discharge voltage hysteresis
#define DEFAULT_CELL_GAP 0.2f         // Cell voltage gap
#define DEFAULT_IGNORE_VOLT 0.5f      // Ignore voltage threshold
#define DEFAULT_STOREV_SETPOINT 3.8f  // Storage voltage setpoint

// Balancing
#define DEFAULT_BALANCE_VOLTAGE 3.9f // Balancing voltage
#define DEFAULT_BALANCE_HYST 0.04f   // Balancing hysteresis
#define DEFAULT_BALANCE_DUTY 50      // Balancing duty cycle

// Current Sensor Settings
#define DEFAULT_NCUR 1           // Number of current sensors
#define DEFAULT_INVERTCUR 0      // Invert current sense
#define DEFAULT_CURSENS 2        // Current sensor type (see CURR_SENSE_* defines)
#define DEFAULT_CURCAN LemCAB300 // CAN current sensor type
#define DEFAULT_CONVHIGH 580     // High Current sensor calibration constant
#define DEFAULT_CONVLOW 6430     // Low Current sensor calibration constant
#define DEFAULT_OFFSET1 1750     // Low Current Sensor reading Offset
#define DEFAULT_OFFSET2 1750     // High Current Sensor reading Offset

// Current Setpoints
#define DEFAULT_DISCURRENTMAX 300     // Max discharge current (A)
#define DEFAULT_DISTAPER 0.3f         // Discharge taper
#define DEFAULT_CHARGECURRENTMAX 300  // Max charge current (A)
#define DEFAULT_CHARGECURRENT2MAX 150 // Max charge current 2 (A)
#define DEFAULT_CHARGECURRENTEND 50   // Charge end current (A)

// Temperature Setpoints
#define DEFAULT_OVERT_SETPOINT 65.0f   // Overtemperature setpoint
#define DEFAULT_UNDERT_SETPOINT -10.0f // Undertemperature setpoint
#define DEFAULT_CHARGET_SETPOINT 0.0f  // Charge temperature setpoint
#define DEFAULT_IGNORE_TEMP 0          // Ignore temperature flag
#define DEFAULT_DIST_SETPOINT 40.0f    // Discharge temperature setpoint

// Timing and Trip
#define DEFAULT_TRIPTIME 500   // Trip time (ms)
#define DEFAULT_WARN_TOFF 5.0f // Warning turn-off time (s)

// Pulse Charging/Discharging
#define DEFAULT_PULSECH 600     // Pulse charge current (A)
#define DEFAULT_PULSECHDUR 5000 // Pulse charge duration (ms)
#define DEFAULT_PULSEDI 600     // Pulse discharge current (A)
#define DEFAULT_PULSEDIDUR 5000 // Pulse discharge duration (ms)

// State of Charge Voltages
#define DEFAULT_SOCVOLT_0 3100 // SOC voltage point 0 (mV)
#define DEFAULT_SOCVOLT_1 10   // SOC percentage point 0
#define DEFAULT_SOCVOLT_2 4100 // SOC voltage point 1 (mV)
#define DEFAULT_SOCVOLT_3 90   // SOC percentage point 1

// Charger Settings
#define DEFAULT_CHARGERTYPE 2  // Charger type
#define DEFAULT_CHARGERSPD 100 // Charger speed
#define DEFAULT_CHARGEREFF 85  // Charger efficiency (%)
#define DEFAULT_CHARGERACV 240 // Charger AC voltage
#define DEFAULT_CHARGERDIRECT 0 // Charger direct flag

// Contactor Settings
#define DEFAULT_PRETIME 5000     // Precharge time (ms)
#define DEFAULT_PRECURRENT 1000  // Precharge current (A). Current must be below this threshold to close the precharge contactor
#define DEFAULT_CONTHOLD 50      // Contactor hold current (A)

// Miscellaneous Settings
#define DEFAULT_VOLTSOC 0        // Voltage SOC
#define DEFAULT_CHANGECUR 20000  // Change current threshold
#define DEFAULT_GAUGELOW 50      // Gauge low value
#define DEFAULT_GAUGEHIGH 255    // Gauge high value



#define DEFAULT_UNDERDUR 5000    // Undervoltage duration (ms)
#define DEFAULT_CURDEAD 5        // Current deadband

// Flags - do not change
#define DEFAULT_EXPMESS 0        // Export messages flag
#define DEFAULT_SERIALCAN 0      // Serial CAN flag
#define DEFAULT_TRIPCONT 1       // Trip contactor flag


// Function Prototypes
void loadSettings();
