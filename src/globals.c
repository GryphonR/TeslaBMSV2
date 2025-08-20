#include "globals.h"

int Discharge;

// variables for output control
int pulltime = 100;
int contctrl = 0, contstat = 0;
unsigned long conttimer1 = 0, conttimer2 = 0, conttimer3 = 0, Pretimer = 0, Pretimer1 = 0, overtriptimer = 0, undertriptimer = 0, mainconttimer = 0;
uint16_t pwmfreq = 18000;
int pwmcurmax = 50;
int pwmcurmid = 50;
int16_t pwmcurmin = 0;

bool OutputEnable = 0;
bool CanOnReq = false;
bool CanOnRev = false;

// variables for VE can
uint16_t chargevoltage = 49100;
uint16_t chargecurrent = 0, tempchargecurrent = 0;
uint16_t disvoltage = 42000;
uint16_t discurrent = 0;
int batvcal = 0;

uint16_t SOH = 100;

unsigned char alarm[4] = {0, 0, 0, 0};
unsigned char warning[4] = {0, 0, 0, 0};
unsigned char mes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned char bmsname[8] = {'S', 'I', 'M', 'P', ' ', 'B', 'M', 'S'};
unsigned char bmsmanu[8] = {'S', 'I', 'M', 'P', ' ', 'E', 'C', 'O'};
long unsigned int rxId = 0;
unsigned char len = 0;
byte rxBuf[8] = {0};
char msgString[128] = {0};
uint32_t inbox = 0;
signed long CANmilliamps = 0;
signed long voltage1 = 0, voltage2 = 0, voltage3 = 0;

// variables for current calculation
uint16_t value = 0;
float currentact = 0, RawCur = 0;
float ampsecond = 0;
unsigned long lasttime = 0;
unsigned long nextLoopTime = 0, looptime1 = 0, UnderTimer = 0, OverTime = 0, cleartime = 0, baltimer = 0, CanOntimeout = 0;
int currentsense = 14;
int sensor = 1;

// Variables for SOC calc
int SOC = 100;
int SOCset = 0;
int SOCtest = 0;
int SOCmem = 0;
int SOCreset = 0;

// charger variables
int maxac1 = 16;
int maxac2 = 10;
int chargerid1 = 0x618;
int chargerid2 = 0x638;
float chargerendbulk = 0;
float chargerend = 0;
int chargertoggle = 0;
int ncharger = 1;
bool chargecurrentlimit = 0;

// serial canbus expansion
unsigned long id = 0;
unsigned char dta[8] = {0};

// AC current control
volatile uint32_t pilottimer = 0;
volatile uint16_t timehigh = 0, duration = 0;
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

// Debugging modes
int debug = 1;
int inputcheck = 0;
int outputcheck = 0;
int candebug = 0;
int gaugedebug = 0;
int debugCur = 0;
int CSVdebug = 0;
int delim = 0;
int menuload = 0;
int balancecells = 0;
int debugdigits = 2;

int testcount = 0;

byte bmsstatus = 0;
byte bmsError = 0;

uint32_t lastUpdate = 0;