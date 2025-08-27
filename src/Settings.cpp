#include "Settings.h"
#include "Logger.h" 
#include "globals.h"

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