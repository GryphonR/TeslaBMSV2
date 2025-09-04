#pragma once

#include "pinouts.h"
#include "Logger.h"
#include "globals.h"
#include "StatusAndLogging.h"

enum ContactorState
{
    OPEN = 0,
    CLOSED = 1,
    CLOSED_PULLIN = 2, // Transient full power state for manual econimising.
    WAITING_TO_OPEN = 3,
    WAITING_TO_OPEN_CURRENT = 4,
    WAITING_TO_CLOSE = 5,
    FAULT = 6,
};

enum ContactorName
{
    POSITIVE = 0,
    NEGATIVE = 1,
    CHARGE = 2,
    PRECHARGE = 3
};

enum Economiser
{
    BuiltIn = 0,
    External = 1,
    None = 2
};

enum ContactorOutput
{
    H1 = 0,
    H2 = 1,
    H3 = 2,
    H4 = 3,
    L5 = 4,
    L6 = 5,
    L7 = 6,
    L8 = 7
};

class BMS_Contactor
{

public:
    BMS_Contactor(ContactorName name, ContactorOutput connection, Economiser economiser);
    void update();
    void close();
    void close(unsigned int delay);
    void open();
    void open(unsigned int delay);
    void open(unsigned int delay, float currentThreshold);
    void set(ContactorState state);
    bool getState();
    bool diagnosticAvailable();
    float getVoltage(bool forceUpdate);
    float getPinCurrent(bool forceUpdate);
    float getPower(bool forceUpdate);
    float getTemperature(bool forceUpdate);
    float getResistance(bool forceUpdate);

private:
    float _gndOffsetV = 0.262; // Measured on bench, nothing connected.
    ContactorState _state;
    ContactorOutput _connection;
    Economiser _economiser;
    ContactorName _name;
    bool _waitingToOpen = false;
    bool _waitingToOpenCurrent = false;
    bool _waitingToClose = false;
    unsigned long _waitingOpenTime = 0;
    float _waitingCurrentThreshold = 0;
    unsigned long _waitingCloseTime = 0;
    unsigned long _economiserStartTime;
    bool _diagnostics = 0;
    float _voltage = -1;
    float _current = -1;
    float _power = -1;
    float _temperature = -1;
    float _resistance = -1;
    uint8_t _pinControl;
    uint8_t _pinEn;
    uint8_t _pinSel0;
    uint8_t _pinSel1;
    uint8_t _pinMultisense;
    void _readVoltage();
    void _readCurrent();
    void _readTemperature();
    char *_connectionToString(ContactorOutput connection);
    char *_contactorNameToString(ContactorName name);
};