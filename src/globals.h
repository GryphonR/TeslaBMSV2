#pragma once
#include <Arduino.h>

// bms status values
#define BMS_STATUS_BOOT 0
#define BMS_STATUS_READY 1
#define BMS_STATUS_DRIVE 2
#define BMS_STATUS_CHARGE 3
#define BMS_STATUS_PRECHARGE 4
#define BMS_STATUS_ERROR 5

// Current sensor values
#define CURR_SENSE_UNDEFINED 0
#define CURR_SENSE_ANALOGUE_DUAL 1
#define CURR_SENSE_CANBUS 2
#define CURR_SENSE_ANALOGUE_GUESSING 3

// Can current sensor values
#define LemCAB300 1
#define IsaScale 3
#define VictronLynx 4
#define LemCAB500 2
#define CurCanMax 4 // max value

// Charger Types
#define NoCharger 0
#define BrusaNLG5 1
#define ChevyVolt 2
#define Eltek 3
#define Elcon 4
#define Victron 5
#define Coda 6
#define VictronHV 7

#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))
#define CPU_REBOOT WRITE_RESTART(0x5FA0004)

extern int Discharge;

// variables for output control
extern int pulltime;
extern int contctrl, contstat;
extern unsigned long conttimer1, conttimer2, conttimer3, Pretimer, Pretimer1, overtriptimer, undertriptimer, mainconttimer;
extern uint16_t pwmfreq;
extern int pwmcurmax;
extern int pwmcurmid;
extern int16_t pwmcurmin;

extern bool OutputEnable;
extern bool CanOnReq;
extern bool CanOnRev;

// variables for VE can
extern uint16_t chargevoltage;
extern uint16_t chargecurrent, tempchargecurrent;
extern uint16_t disvoltage;
extern uint16_t discurrent;
extern int batvcal;

extern uint16_t SOH;

extern unsigned char alarm[4];
extern unsigned char warning[4];
extern unsigned char mes[8];
extern unsigned char bmsname[8];
extern unsigned char bmsmanu[8];
extern long unsigned int rxId;
extern unsigned char len;
extern byte rxBuf[8];
extern char msgString[128];
extern uint32_t inbox;
extern signed long CANmilliamps;
extern signed long voltage1, voltage2, voltage3;

// variables for current calculation
extern uint16_t value;
extern float currentact, RawCur;
extern float ampsecond;
extern unsigned long lasttime;
extern unsigned long nextLoopTime, looptime1, UnderTimer, OverTime, cleartime, baltimer, CanOntimeout;
extern int currentsense;
extern int sensor;

// Variables for SOC calc
extern int SOC;
extern int SOCset;
extern int SOCtest;
extern int SOCmem;
extern int SOCreset;

// charger variables
extern int maxac1;
extern int maxac2;
extern int chargerid1;
extern int chargerid2;
extern float chargerendbulk;
extern float chargerend;
extern int chargertoggle;
extern int ncharger;
extern bool chargecurrentlimit;

// serial canbus expansion
extern unsigned long id;
extern unsigned char dta[8];

// AC current control
extern volatile uint32_t pilottimer;
extern volatile uint16_t timehigh, duration;
extern volatile uint16_t accurlim;
extern volatile int dutycycle;
extern uint16_t chargerpower;
extern bool CPdebug;

// variables
extern int outputstate;
extern int incomingByte;
extern int x;
extern int storagemode;
extern int cellspresent;
extern int dashused;
extern int Charged;
extern int renum;

// Debugging modes
extern int debug;
extern int inputcheck;
extern int outputcheck;
extern int candebug;
extern int gaugedebug;
extern int debugCur;
extern int CSVdebug;
extern int delim;
extern int menuload;
extern int balancecells;
extern int debugdigits;

enum errorType
{
    ERROR_NONE = 0,
    ERROR_BATTERY_COMMS = 1,
    ERROR_VOLTAGE = 2,
    ERROR_CONTACTORS_NOT_CLOSING = 4,
    ERROR_CONTACTORS_OPENED_EMERGENCY = 5,
    ERROR_CURRENT_READING = 6,
    ERROR_OVER_VOLTAGE = 7,
    ERROR_UNDER_VOLTAGE = 8,
    ERROR_OVER_TEMPERATURE = 9,
    ERROR_CAN = 10,
};

extern int testcount;

extern byte bmsstatus;
extern byte bmsError;

extern uint32_t lastUpdate;