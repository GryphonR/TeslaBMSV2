#include "BMS_Contactor.h"

/**
 * STILL TODO!
 *
 * - Set fault reset signal to select fault behaviour between latch off or auto restart
 * - Diagnostics for open/short circuit
 * - Diagnostics for over temperature
 * - Diagnostics for checking if contactor is connected or not by current reading.
 *
 */
BMS_Contactor::BMS_Contactor(ContactorName name, ContactorOutput connection, Economiser economiser)
{
    _connection = connection;
    _economiser = economiser;

    switch (_connection)
    {
    case H1:
        _pinControl = PIN_OUT1;
        break;
    case H2:
        _pinControl = PIN_OUT2;
        break;
    case H3:
        _pinControl = PIN_OUT3;
        break;
    case H4:
        _pinControl = PIN_OUT4;
        break;
    case L5:
        _pinControl = PIN_OUT5;
        break;
    case L6:
        _pinControl = PIN_OUT6;
        break;
    case L7:
        _pinControl = PIN_OUT7;
        break;
    case L8:
        _pinControl = PIN_OUT8;
        break;
    default:
        Logger::error("Invalid ContactorOutput %s", _connectionToString(_connection));
        break;
    }

    pinMode(_pinControl, OUTPUT);

    if (_connection < 4)
    { // High Side Driver
        if (economiser == Economiser::External)
        {
            Logger::warn("External Economiser not supported for High Side Driver %s, Economiser is disabled", _connectionToString(_connection));
            _economiser = Economiser::None;
        }
        _diagnostics = 1;
        if (_connection < 2)
        { // Driver A
            _pinEn = PIN_HSD_SENSE_EN_A;
            _pinSel0 = PIN_HSD_SEL_0_A;
            _pinSel1 = PIN_HSD_SEL_1_A;
            _pinMultisense = PIN_HSD_SEL_MULTISENSE_A;
        }
        else
        { // Driver B
            _pinEn = PIN_HSD_SENSE_EN_B;
            _pinSel0 = PIN_HSD_SEL_0_B;
            _pinSel1 = PIN_HSD_SEL_1_B;
            _pinMultisense = PIN_HSD_SEL_MULTISENSE_B;
        }

        pinMode(_pinEn, OUTPUT);
        pinMode(_pinSel0, OUTPUT);
        pinMode(_pinSel1, OUTPUT);
        pinMode(_pinMultisense, INPUT); // Not actually needed for ADC
    }
}

void BMS_Contactor::update()
{
    // For Manual Economiser, if it is newly closed, this reverts to econiomised state
    if (_state == ContactorState::CLOSED_PULLIN)
    {
        // If we're passed the economiser start time and (double check) the economiser is external
        if (millis() > _economiserStartTime && _economiser == Economiser::External)
        {
            Logger::info("Relaxing %s to econimised state", _connectionToString(_connection));
            analogWrite(_pinControl, settings.conthold);
        }
    }

    // For Timed Functions - check if time periods have elapsed or thresholds are met
    if (_state == ContactorState::WAITING_TO_CLOSE)
    {
        if (millis() > _waitingCloseTime)
        {
            close();
        }
    }
    else if (_state == ContactorState::WAITING_TO_OPEN)
    { // If we're passed the waiting open time, open the contactor. This overrides the current threshold if set
        if (millis() > _waitingOpenTime)
        {
            open();
        }
    }
    else if (_state == ContactorState::WAITING_TO_OPEN_CURRENT)
    { // If the current is less than the waiting current threshold, open the contactor
        if (currentact < _waitingCurrentThreshold)
        {
            Logger::info("Current below threshold %f, opening %s", _waitingCurrentThreshold, _connectionToString(_connection));
            open();
        }
    }

    // Todo - diagnostics - if open, check no current is flowing, if closed, check current is flowing, overTemperature etc.
    // probably in a slow loop - every second?
}

/**
 * @brief Close the contactor and set the state to CLOSED.
 *
 * Checks for potential latch up after setting the control pin high.
 * If the pin is not high, as expected, it logs an error and attempts to return the contactor to a safe (OPEN) state.
 */
void BMS_Contactor::close()
{
    digitalWrite(_pinControl, HIGH);
    _waitingCloseTime = 0;
    delay(1);                      // Catch the unlikely case of latch down
    if (!digitalRead(_pinControl)) // If control pin is not high, as expected
    {
        setBMSstatus(BMS_STATUS_ERROR, ERROR_CONTACTORS_NOT_CLOSING, "Failed to assert contactor close singnal, Pin suck low");
        Logger::error("Potential Latch down detected on %s.", _connectionToString(_connection));
        Logger::error("Attempting to return %s to safe (OPEN) state.", _connectionToString(_connection));
        _state = ContactorState::FAULT;
        open();
    }
    else
    {
        _state = ContactorState::CLOSED;
    }
}

/**
 * @brief Schedules the contactor to close after a specified delay
 *
 * Sets the contactor state to WAITING_TO_CLOSE and sets the waiting close time to the current millis() plus the specified delay.
 * If the contactor is already closed, this function does nothing.
 *
 * @note Relies on the BMS_Contactor::update() function to be called regularly
 *
 * @param delay The delay in milliseconds to wait before closing the contactor
 */
void BMS_Contactor::close(unsigned int delay)
{
    if (_state != ContactorState::CLOSED)
    {
        _state = ContactorState::WAITING_TO_CLOSE;
        _waitingCloseTime = millis() + delay;
        Logger::info("Closing %s in %dms", _contactorNameToString(_name), delay);
    }
}

/**
 * @brief Opens the contactor and sets the state to OPEN
 *
 * Checks for potential latch up after setting the control pin low.
 *
 */
void BMS_Contactor::open()
{
    digitalWrite(_pinControl, LOW);
    Logger::info("Opening %s", _contactorNameToString(_name));
    _waitingOpenTime = 0;
    _waitingCurrentThreshold = 0;
    delay(1);
    // Catch the unlikely case of latch up
    if (digitalRead(_pinControl))
    {
        setBMSstatus(BMS_STATUS_ERROR, ERROR_CONTACTORS_NOT_OPENING, "Failed to assert contactor open singnal, Pin suck high");
        Logger::error("Potential Latch up detected on %s", _contactorNameToString(_name));
        _state = ContactorState::FAULT;
    }
    else
    {
        _state = ContactorState::OPEN;
    }
}

/**
 * @brief Opens the contactor after a specified delay
 *
 * If the contactor is not already open, sets the contactor to the WAITING_TO_OPEN state
 * and sets the waiting open time to the current time plus the specified delay. A debug
 * message is logged to indicate that the contactor is opening with the specified delay.
 *
 * @note Relies on the BMS_Contactor::update() function to be called regularly
 *
 * @param delay The delay in milliseconds to wait before opening the contactor
 */
void BMS_Contactor::open(unsigned int delay)
{
    if (_state != ContactorState::OPEN)
    {
        _state = ContactorState::WAITING_TO_OPEN;
        _waitingOpenTime = millis() + delay;
        Logger::info("Opening %s in %dms", _contactorNameToString(_name), delay);
    }
}

/**
 * @brief Opens the contactor when the current is below the specified threshold or after the specified timeout
 *
 * If the contactor is not already open, sets the contactor to the WAITING_TO_OPEN_CURRENT state
 * and sets the waiting open time to the current time plus the specified timeout. A debug
 * message is logged to indicate that the contactor is opening with the specified delay.
 *
 * @note Relies on the BMS_Contactor::update() function to be called regularly, and current to be updated regularly (Both sub 100ms periodically)
 *
 * @param timeout The delay in milliseconds to wait before opening the contactor
 * @param currentThreshold The current threshold below which the contactor will open
 */
void BMS_Contactor::open(unsigned int timeout, float currentThreshold)
{
    if (_state != ContactorState::OPEN)
    {
        _state = ContactorState::WAITING_TO_OPEN_CURRENT;
        _waitingCurrentThreshold = currentThreshold;
        _waitingOpenTime = millis() + timeout;
        Logger::info("Opening %s when current is below %f OR after %dms", _contactorNameToString(_name), currentThreshold, timeout);
    }
}

/**
 * @brief Sets the contactor state to the specified value
 *
 * This is a simple wrapper around the open() and close() functions, to make it easier to
 * set the contactor state in a single call. If the state is set to OPEN, it calls open() to
 * open the contactor. If the state is set to any other value, it calls close() to close the
 * contactor. If the contactor is already in the requested state, this function does nothing.
 *
 * @param state The desired state for the contactor. Must be either ContactorState::OPEN or
 * ContactorState::CLOSED
 */
void BMS_Contactor::set(ContactorState state)
{
    if (state == ContactorState::OPEN)
    {
        open();
    }
    else
    {
        close();
    }
}

/**
 * @brief Gets the current state of the contactor.
 *
 * @return The current state of the contactor. Will be one of the ContactorState enum values.
 */
bool BMS_Contactor::getState()
{
    return _state;
}

/**
 * @brief Indicates whether diagnostic functions are available for this contactor.
 *
 * @return true if diagnostics are available, false otherwise
 */
bool BMS_Contactor::diagnosticAvailable()
{
    return _diagnostics;
}

/**
 * @brief Reads the voltage across the contactor coil.
 *
 * If the contactor is a high side driver with diagnostic capabilities, this function
 * reads the voltage across the contactor coil. The voltage is read from the ADC and the
 * measured value is returned after being converted to a voltage value using the
 * datasheet voltage sense calculations.
 *
 * @note The voltage sense calculation is based on the VND7050 datasheet and is a rough
 * estimate.
 *
 * @return The voltage across the contactor coil in Volts. If the contactor does not have
 * diagnostic capabilities, the function returns -1.
 */
void BMS_Contactor::_readVoltage()
{
    if (_diagnostics)
    {
        digitalWrite(_pinEn, HIGH);
        digitalWrite(_pinSel0, HIGH);
        digitalWrite(_pinSel1, HIGH);
        delay(2);
        uint16_t adc = analogRead(_pinMultisense);
        float voltage = (((adc * 3.3) / 1023) - _gndOffsetV);// * 8.0; // 3.3 for reference voltage, -0.37 for ground network offset, 8 for VND7050 IC multiplier
        Logger::debug("Voltage Read ADC: %d", adc);
        Logger::debug("Voltage Pre Conversion: %fV", voltage);
        voltage = voltage *8.0;
        Logger::debug("Voltage: %fV", voltage);
        _voltage = voltage;
    }
    else
    {
        _voltage = -1;
    }
}

/**
 * @brief Reads the current through the contactor coil.
 *
 * If the contactor is a high side driver with diagnostic capabilities, this function
 * reads the current through the contactor coil. The current is read from the ADC and the
 * measured value is returned after being converted to a current value using the
 * datasheet current sense resistor calculations.
 *
 * @note The current sense resistor value is hard coded to 680 ohms, which is the
 * value used in the BMS design. The current sense calculation is based on the
 * VND7050 datasheet and is a rough estimate.
 *
 * @return The current through the contactor coil in Amps. If the contactor does
 * not have diagnostic capabilities, the function returns -1.
 */
void BMS_Contactor::_readCurrent()
{
    if (_diagnostics)
    {
        digitalWrite(_pinEn, HIGH);

        if (_connection == H1 || _connection == H3)
        { // table 11, page 20 VND7050 datasheet https://www.lcsc.com/datasheet/C443997.pdf
            digitalWrite(_pinSel0, LOW);
            digitalWrite(_pinSel1, LOW);
        }
        else
        {
            digitalWrite(_pinSel0, HIGH);
            digitalWrite(_pinSel1, LOW);
        }
        delay(2);
        uint16_t adc = analogRead(_pinMultisense);
        Logger::debug("Current Read ADC: %d", adc);
        // calculate voltage on current sense resistor
        float voltage = ((adc * 3.3 ) / 1023) - _gndOffsetV; // 3.3 for reference voltage, -0.37 for ground network offset
        Logger::debug("Current Reading - Voltage %fV", voltage);
        float senseResistorCurrent = voltage / 680;
        const float K = 1400; // Guestimate from page 16 graph VND7050 datasheet https://www.lcsc.com/datasheet/C443997.pdf
        float current = senseResistorCurrent * K;
        Logger::debug("Current: %fA", current);
        _current = current;
    }
    else
    {
        _current = -1;
    }
}

/**
 * @brief Reads the temperature of the driver casing.
 *
 * If the driver has diagnostic capabilities, this function reads the
 * temperature of the driver casing. The temperature is read from the ADC and the
 * measured value is returned after being converted to a temperature value using the
 * datasheet calculations.
 *
 * @note The temperature calculation is based on the VND7050 datasheet and is a rough
 * estimate. The maximum temperature is assumed to be 150 degrees Celsius.
 *
 * @return The temperature of the driver casing in degrees Celsius. If the driver does
 * not have diagnostic capabilities, the function returns -1.
 */
void BMS_Contactor::_readTemperature()
{
    if (_diagnostics)
    {
        digitalWrite(_pinEn, HIGH);
        digitalWrite(_pinSel0, LOW);
        digitalWrite(_pinSel1, HIGH);
        delay(2);
        uint16_t adc = analogRead(_pinMultisense);
        Logger::debug("ADC: %d", adc);
        // mV per degree = -5.5. Therefore Temp = max temp - (mVoltage/5.5) Max Temp isn't clear?
        // 5.5@5v, 3.63@3.3v?
        // trial and error puts it at 13.5!
        
        float voltage = ((adc * (3300)) / 1023) - _gndOffsetV; // 3.3 for reference voltage, -0.37 for ground network offset, 8 for VND7050 IC multiplier
        Logger::debug("Temp Reading Voltage: %fmV", voltage);
        float temp = 150 - (voltage / 13.5); 
        _temperature = temp;
    }
    else
    {
        _temperature = -1;
    }
}

/**
 * @brief Gets the power consumption of the contactor coil.
 *
 * If the driver has diagnostic capabilities, this function returns the power
 * consumption of the contactor coil. If forceUpdate is true, it will first read the
 * latest voltage and current values from the driver. If the driver does not have
 * diagnostic capabilities, the function returns -1.
 *
 * @return The power consumption of the contactor coil in Watts. If the driver does
 * not have diagnostic capabilities, the function returns -1.
 */
float BMS_Contactor::getPower(bool forceUpdate)
{
    if (_diagnostics)
    {
        if (forceUpdate)
        {
            _readVoltage();
            _readCurrent();
        }
        return (_voltage * _current);
    }
    return -1;
}

/**
 * @brief Gets the temperature of the driver casing.
 *
 * If the driver casing driver has diagnostic capabilities, this function returns the
 * temperature of the driver casing. If forceUpdate is true, it will first read the
 * latest temperature value from the driver. If the driver does not have diagnostic
 * capabilities, the function returns -1.
 *
 * @param forceUpdate If true, a fresh temperature value will be read from the
 * driver before being returned.
 *
 * @return The temperature of the driver casing in degrees Celsius. If the driver does
 * not have diagnostic capabilities, the function returns -1.
 */
float BMS_Contactor::getTemperature(bool forceUpdate)
{
    if (_diagnostics)
    {
        if (forceUpdate)
        {
            _readTemperature();
        }
        return _temperature;
    }
    return -1;
}

/**
 * @brief Gets the voltage at the driver.
 *
 *
 * @param forceUpdate If true, the latest voltage values will be read from the
 * driver before returning the value.
 *
 * @return The voltage at the driver in Volts.
 */
float BMS_Contactor::getVoltage(bool forceUpdate)
{
    if (_diagnostics)
    {
        if (forceUpdate)
        {
            _readVoltage();
        }
        return _voltage;
    }
    return -1;
}

/**
 * @brief Gets the current through the contactor coil.
 *
 * If the driver has diagnostic capabilities, this function returns the current
 * through the contactor coil.
 *
 * @param forceUpdate If true, a fresh current value will be read from the
 * driver before being returned.
 *
 * @return The current through the contactor coil in Amps. If the driver does
 * not have diagnostic capabilities, the function returns -1.
 */
float BMS_Contactor::getPinCurrent(bool forceUpdate)
{
    if (_diagnostics)
    {
        if (forceUpdate)
        {
            _readCurrent();
        }
        return _current;
    }
    return -1;
}

/**
 * @brief Gets the resistance of the load connected to the contactor.
 *
 * Calculates the resistance of the load connected to the contactor by dividing the
 * voltage by the current. If forceUpdate is true, it will first read the latest
 * voltage and current values from the contactor.
 *
 * @param forceUpdate If true, fresh voltage and current values will be read
 * from the driver before calculating the resistance.
 *
 * @return The resistance of the load connected to the driver in Ohms.
 */
float BMS_Contactor::getResistance(bool forceUpdate)
{
    if (_diagnostics)
    {
        if (forceUpdate)
        {
            _readVoltage();
            _readCurrent();
        }
        return _voltage / _current;
    }
    return -1;
}

char *BMS_Contactor::_connectionToString(ContactorOutput connection)
{
    switch (connection)
    {
    case H1:
        return (char *)"H1";
    case H2:
        return (char *)"H2";
    case H3:
        return (char *)"H3";
    case H4:
        return (char *)"H4";
    case L5:
        return (char *)"L5";
    case L6:
        return (char *)"L6";
    case L7:
        return (char *)"L7";
    case L8:
        return (char *)"L8";
    default:
        return (char *)"Unknown";
    }
}

char *BMS_Contactor::_contactorNameToString(ContactorName name)
{
    switch (name)
    {
    case POSITIVE:
        return (char *)"Positive Contactor";
    case NEGATIVE:
        return (char *)"Negative Contactor";
    case CHARGE:
        return (char *)"Charge Contactor";
    case PRECHARGE:
        return (char *)"Precharge Contactor";
    default:
        return (char *)"Unknown Contactor";
    }
}
