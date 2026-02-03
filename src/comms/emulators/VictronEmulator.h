#pragma once

#include "Emulator.h"
#include "config/globals.h"
#include "config/pinouts.h"
#include "battery/BMSModuleManager.h"

/**
 * @brief Emulates a Victron Lynx BMS over CAN bus.
 * 
 * Sends periodic status messages to make this BMS appear as a 
 * Victron-compatible battery to Victron inverters/chargers.
 * 
 * CAN Messages sent:
 * - 0x351: Charge/discharge voltage and current limits
 * - 0x355: SOC and SOH
 * - 0x356: Pack voltage, current, temperature
 * - 0x35A: Alarms and warnings
 * - 0x35E: BMS name
 * - 0x370: Manufacturer name
 * - 0x372: Number of modules
 * - 0x373: Cell voltage and temperature extremes
 * - 0x379: Installed capacity and status
 */
class VictronEmulator : public Emulator
{
public:
    VictronEmulator(SendFunc sendFunc, BMSModuleManager &bmsRef, EEPROMSettings &settingsRef)
        : Emulator(sendFunc), _bms(bmsRef), _settings(settingsRef) {}

    void update() override
    {
        CAN_message_t msg;

        // 0x351: Charge/Discharge limits
        msg.id = 0x351;
        msg.len = 8;
        if (storagemode == 0)
        {
            msg.buf[0] = lowByte(uint16_t((_settings.ChargeVsetpoint * _settings.Scells) * 10));
            msg.buf[1] = highByte(uint16_t((_settings.ChargeVsetpoint * _settings.Scells) * 10));
        }
        else
        {
            msg.buf[0] = lowByte(uint16_t((_settings.StoreVsetpoint * _settings.Scells) * 10));
            msg.buf[1] = highByte(uint16_t((_settings.StoreVsetpoint * _settings.Scells) * 10));
        }
        msg.buf[2] = lowByte(chargecurrent);
        msg.buf[3] = highByte(chargecurrent);
        msg.buf[4] = lowByte(discurrent);
        msg.buf[5] = highByte(discurrent);
        msg.buf[6] = lowByte(uint16_t((_settings.DischVsetpoint * _settings.Scells) * 10));
        msg.buf[7] = highByte(uint16_t((_settings.DischVsetpoint * _settings.Scells) * 10));
        send(msg);

        // 0x355: SOC/SOH
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
        send(msg);

        // 0x356: Pack voltage, current, temp
        msg.id = 0x356;
        msg.len = 8;
        if (_settings.chargertype == VictronHV || _settings.SerialCan == 1)
        {
            msg.buf[0] = lowByte(uint16_t(_bms.getPackVoltage() * 10));
            msg.buf[1] = highByte(uint16_t(_bms.getPackVoltage() * 10));
        }
        else
        {
            msg.buf[0] = lowByte(uint16_t(_bms.getPackVoltage() * 100));
            msg.buf[1] = highByte(uint16_t(_bms.getPackVoltage() * 100));
        }
        msg.buf[2] = lowByte(long(currentact / 100));
        msg.buf[3] = highByte(long(currentact / 100));
        msg.buf[4] = lowByte(int16_t(_bms.getAvgTemperature() * 10));
        msg.buf[5] = highByte(int16_t(_bms.getAvgTemperature() * 10));
        msg.buf[6] = 0;
        msg.buf[7] = 0;
        send(msg);

        // 0x35A: Alarms/Warnings
        msg.id = 0x35A;
        msg.len = 8;
        msg.buf[0] = alarm[0];
        msg.buf[1] = alarm[1];
        msg.buf[2] = alarm[2];
        msg.buf[3] = alarm[3];
        msg.buf[4] = warning[0];
        msg.buf[5] = warning[1];
        msg.buf[6] = warning[2];
        msg.buf[7] = warning[3];
        send(msg);

        // 0x35E: BMS Name
        msg.id = 0x35E;
        msg.len = 8;
        for (int i = 0; i < 8; i++) msg.buf[i] = bmsname[i];
        send(msg);

        // 0x370: Manufacturer
        msg.id = 0x370;
        msg.len = 8;
        for (int i = 0; i < 8; i++) msg.buf[i] = bmsmanu[i];
        send(msg);

        // 0x373: Cell extremes
        msg.id = 0x373;
        msg.len = 8;
        msg.buf[0] = lowByte(uint16_t(_bms.getLowCellVolt() * 1000));
        msg.buf[1] = highByte(uint16_t(_bms.getLowCellVolt() * 1000));
        msg.buf[2] = lowByte(uint16_t(_bms.getHighCellVolt() * 1000));
        msg.buf[3] = highByte(uint16_t(_bms.getHighCellVolt() * 1000));
        msg.buf[4] = lowByte(uint16_t(_bms.getLowTemperature() + 273.15));
        msg.buf[5] = highByte(uint16_t(_bms.getLowTemperature() + 273.15));
        msg.buf[6] = lowByte(uint16_t(_bms.getHighTemperature() + 273.15));
        msg.buf[7] = highByte(uint16_t(_bms.getHighTemperature() + 273.15));
        send(msg);

        // 0x379: Capacity and status
        msg.id = 0x379;
        msg.len = 8;
        msg.buf[0] = lowByte(uint16_t(_settings.Pstrings * _settings.CAP));
        msg.buf[1] = highByte(uint16_t(_settings.Pstrings * _settings.CAP));
        msg.buf[2] = contstat;
        msg.buf[3] = (digitalRead(PIN_OUT1) | (digitalRead(PIN_OUT2) << 1) | 
                      (digitalRead(PIN_OUT3) << 2) | (digitalRead(PIN_OUT4) << 3));
        msg.buf[4] = bmsstatus;
        msg.buf[5] = 0x00;
        msg.buf[6] = 0x00;
        msg.buf[7] = 0x00;
        send(msg);

        // 0x372: Module count
        msg.id = 0x372;
        msg.len = 8;
        msg.buf[0] = lowByte(_bms.getNumModules());
        msg.buf[1] = highByte(_bms.getNumModules());
        msg.buf[2] = 0x00;
        msg.buf[3] = 0x00;
        msg.buf[4] = 0x00;
        msg.buf[5] = 0x00;
        msg.buf[6] = 0x00;
        msg.buf[7] = 0x00;
        send(msg);
    }

private:
    BMSModuleManager &_bms;
    EEPROMSettings &_settings;
};
