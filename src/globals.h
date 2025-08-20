#pragma once 

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

//
// Charger Types
#define NoCharger 0
#define BrusaNLG5 1
#define ChevyVolt 2
#define Eltek 3
#define Elcon 4
#define Victron 5
#define Coda 6
#define VictronHV 7
//


#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))
#define CPU_REBOOT WRITE_RESTART(0x5FA0004)





int Discharge;

// variables for output control
int pulltime = 100;
int contctrl, contstat = 0; // 1 = out 5 high 2 = out 6 high 3 = both high
unsigned long conttimer1, conttimer2, conttimer3, Pretimer, Pretimer1, overtriptimer, undertriptimer, mainconttimer = 0;
uint16_t pwmfreq = 18000; // pwm frequency
int pwmcurmax = 50;       // Max current to be shown with pwm
int pwmcurmid = 50;       // Mid point for pwm dutycycle based on current
int16_t pwmcurmin = 0;    // DONOT fill in, calculated later based on other values

bool OutputEnable = 0; // Request to close contactors
bool CanOnReq = false; // CAN Request to close Contacors
bool CanOnRev = false;

// variables for VE can
uint16_t chargevoltage = 49100; // max charge voltage in mv
uint16_t chargecurrent, tempchargecurrent = 0;
uint16_t disvoltage = 42000; // max discharge voltage in mv
uint16_t discurrent = 0;
int batvcal = 0;

uint16_t SOH = 100; // SOH place holder

unsigned char alarm[4] = {0, 0, 0, 0};
unsigned char warning[4] = {0, 0, 0, 0};
unsigned char mes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char bmsname[8] = {'S', 'I', 'M', 'P', ' ', 'B', 'M', 'S'};
unsigned char bmsmanu[8] = {'S', 'I', 'M', 'P', ' ', 'E', 'C', 'O'};
long unsigned int rxId;
unsigned char len = 0;
byte rxBuf[8];
char msgString[128]; // Array to store serial string
uint32_t inbox;
signed long CANmilliamps;                     // mV
signed long voltage1, voltage2, voltage3 = 0; // mV only with ISAscale sensor
// struct can_frame canMsg;
// MCP2515 CAN1(10); //set CS pin for can controlelr

// variables for current calulation
uint16_t value;
float currentact, RawCur;
float ampsecond;
unsigned long lasttime;
unsigned long nextLoopTime, looptime1, UnderTimer, OverTime, cleartime, baltimer, CanOntimeout = 0; // ms
int currentsense = 14;
int sensor = 1;

// Variables for SOC calc
int SOC = 100; // State of Charge
int SOCset = 0;
int SOCtest = 0;
int SOCmem = 0;
int SOCreset = 0;

/// charger variables
int maxac1 = 16;          // Shore power 16A per charger
int maxac2 = 10;          // Generator Charging
int chargerid1 = 0x618;   // bulk chargers
int chargerid2 = 0x638;   // finishing charger
float chargerendbulk = 0; // V before Charge Voltage to turn off the bulk charger/s
float chargerend = 0;     // V before Charge Voltage to turn off the finishing charger/s
int chargertoggle = 0;
int ncharger = 1; // number of chargers
bool chargecurrentlimit = 0;

// serial canbus expansion
unsigned long id = 0;
unsigned char dta[8];

// AC current control
volatile uint32_t pilottimer = 0;
volatile uint16_t timehigh, duration = 0;
volatile uint16_t accurlim = 0;
volatile int dutycycle = 0;
uint16_t chargerpower = 0;
bool CPdebug = 0;

// variables
int outputstate = 0;
int incomingByte = 0;
int x = 0;
int storagemode = 0;
int cellspresent = 0;
int dashused = 1;
int Charged = 0;
int renum = 0;

// Debugging modes//////////////////
int debug = 1;
int inputcheck = 0;  // read digital inputs
int outputcheck = 0; // check outputs
int candebug = 0;    // view can frames
int gaugedebug = 0;
int debugCur = 0;
int CSVdebug = 0;
int delim = 0;
int menuload = 0;
int balancecells;
int debugdigits = 2; // amount of digits behind decimal for voltage reading

int testcount = 0;

byte bmsstatus = 0;

uint32_t lastUpdate;