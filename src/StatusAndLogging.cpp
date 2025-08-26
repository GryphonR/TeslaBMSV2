/**
 * File to contain status management functions, Serial console logging funcions
 * and SD Card logging functions
*/


#include "StatusAndLogging.h"
#include "pinouts.h"

/**
 * @brief Sets the BMS status and logs the change.
 *
 * This function takes an integer representing the current state of the BMS
 * and returns a corresponding string literal. This is useful for logging,
 * debugging, and displaying the status on a user interface.
 *
 * @param status The integer status code of the BMS (e.g., BMS_STATUS_READY, BMS_STATUS_CHARGE).
 * @param message Optional additional message to log when setting the status.
 */
void setBMSstatus(int newStatus, const char *message)
{
    bmsstatus = newStatus;
    if (newStatus == BMS_STATUS_ERROR)
    {
        if (message)
        {
            Logger::error("BMS Status set to %s: %s", getBMSStatusString(newStatus), message);
        }
        else
        {
            Logger::error("BMS Status set to %s", getBMSStatusString(newStatus));
        }
    }
    else
    {
        if (message)
        {
            Logger::debug("BMS Status set to %s: %s", getBMSStatusString(newStatus), message);
        }
        else
        {
            Logger::debug("BMS Status set to %s", getBMSStatusString(newStatus));
        }
    }
}

/**
 * @brief Sets the BMS status and error code, and logs the change.
 *
 * This function takes an integer representing the current state of the BMS
 * and an optional error code. It updates both the BMS status and error code,
 * logging the changes for debugging and monitoring purposes.
 *
 * @param newStatus The integer status code of the BMS (e.g., BMS_STATUS_READY, BMS_STATUS_CHARGE).
 * @param newError Optional integer error code to set (default is 0, meaning no error).
 * @param message Optional additional message to log when setting the status.
 */
void setBMSstatus(int newStatus, int newError, const char *message)
{
    setBMSstatus(newStatus, message);

    if (newError != 0)
    {
        bmsError = newError;
        Logger::error("BMS Error set to %s", getBMSErrorString(bmsError));
    }
}

/**
 * @brief Converts a BMS status code into a human-readable string.
 *
 * This function takes an integer representing the current state of the BMS
 * and returns a corresponding string literal. This is useful for logging,
 * debugging, and displaying the status on a user interface.
 *
 * @param status The integer status code of the BMS (e.g., BMS_STATUS_READY, BMS_STATUS_CHARGE).
 * @return A constant character pointer to a string describing the status.
 *         Returns "Unknown" if the provided status code does not match any known state.
 */
const char *getBMSStatusString(int status)
{
    switch (status)
    {
    case BMS_STATUS_BOOT:
        return "Boot";
    case BMS_STATUS_READY:
        return "Ready";
    case BMS_STATUS_PRECHARGE:
        return "Precharge";
    case BMS_STATUS_DRIVE:
        return "Drive";
    case BMS_STATUS_CHARGE:
        return "Charge";
    case BMS_STATUS_ERROR:
        return "Error";
    default:
        return "Unknown";
    }
}

/**
 * @brief Converts a BMS error code into a human-readable string.
 *
 * This function takes an integer representing the current error state of the BMS
 * and returns a corresponding string literal. This is useful for logging,
 * debugging, and displaying the error on a user interface.
 *
 * @param error The integer error code of the BMS (e.g., ERROR_NONE, ERROR_VOLTAGE).
 * @return A constant character pointer to a string describing the error.
 *         Returns "Unknown Error" if the provided error code does not match any known error.
 */
const char *getBMSErrorString(int error)
{
    switch (error)
    {
    case ERROR_NONE:
        return "No Error";
    case ERROR_CAN:
        return "CAN Error";
    case ERROR_VOLTAGE:
        return "Voltage Error";
    case ERROR_UNDER_VOLTAGE:
        return "Undervoltage Error";
    case ERROR_OVER_VOLTAGE:
        return "Overvoltage Error";
    case ERROR_OVER_TEMPERATURE:
        return "Over Temperature Error";
    case ERROR_UNDER_TEMPERATURE:
        return "Under Temperature Error";
    case ERROR_CURRENT_READING:
        return "Current Sensor Error";
    default:
        return "Unknown Error";
    }
}

/**
 * @brief Updates the alarm and warning status arrays based on current BMS readings and settings.
 *
 * This function checks the current cell voltages and temperatures against the configured setpoints
 * in the settings structure. It sets the appropriate bits in the `alarm` and `warning` arrays to
 * indicate overvoltage, undervoltage, overtemperature, undertemperature, and cell imbalance conditions.
 * These arrays are used for CAN communication and system status reporting.
 *
 * Alarm bits:
 * - alarm[0]:
 *   - 0x04 / 0b00000100: High cell voltage exceeds OverVSetpoint
 *   - 0x10 / 0b00010000: Low cell voltage below UnderVSetpoint
 *   - 0x40 / 0b01000000: High temperature exceeds OverTSetpoint
 * - alarm[1]:
 *   - 0x01: Low temperature below UnderTSetpoint
 * - alarm[3]:
 *   - 0x01: Cell voltage difference exceeds CellGap
 *
 * Warning bits:
 * - warning[0]:
 *   - 0x04: High cell voltage exceeds (OverVSetpoint - WarnOff)
 *   - 0x10: Low cell voltage below (UnderVSetpoint + WarnOff)
 *   - 0x40: High temperature exceeds (OverTSetpoint - WarnToff)
 * - warning[1]:
 *   - 0x01: Low temperature below (UnderTSetpoint + WarnToff)
 *
 * The function resets all alarm and warning bits before evaluating the current status.
 *
 * @note This function should be called periodically to ensure alarms and warnings are up-to-date.
 */
void alarmupdate()
{
    alarm[0] = 0x00;
    if (settings.OverVSetpoint < bms.getHighCellVolt())
    {
        alarm[0] = 0x04;
    }
    if (bms.getLowCellVolt() < settings.UnderVSetpoint)
    {
        alarm[0] |= 0x10;
    }
    if (bms.getHighTemperature() > settings.OverTSetpoint)
    {
        alarm[0] |= 0x40;
    }
    alarm[1] = 0;
    if (bms.getLowTemperature() < settings.UnderTSetpoint)
    {
        alarm[1] = 0x01;
    }
    alarm[3] = 0;
    if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap)
    {
        alarm[3] = 0x01;
    }

    /// warnings///
    warning[0] = 0;

    if (bms.getHighCellVolt() > (settings.OverVSetpoint - settings.WarnOff))
    {
        warning[0] = 0x04;
    }
    if (bms.getLowCellVolt() < (settings.UnderVSetpoint + settings.WarnOff))
    {
        warning[0] |= 0x10;
    }

    if (bms.getHighTemperature() > (settings.OverTSetpoint - settings.WarnToff))
    {
        warning[0] |= 0x40;
    }
    warning[1] = 0;
    if (bms.getLowTemperature() < (settings.UnderTSetpoint + settings.WarnToff))
    {
        warning[1] = 0x01;
    }
}

/**
 * @brief Prints the current BMS status and related information to the serial console.
 *
 * This function outputs various details about the BMS status, including:
 * - Current BMS status (e.g., ESS Mode, Boot, Ready, Precharge, Drive, Charge, Error)
 * - Input states (e.g., AC Present, Key ON)
 * - Balancing status
 * - Cell count
 * - Output states
 * - Contactor control status
 * - Charge and discharge current limits
 * - Charge power and duration if in charge state
 *
 * The output is formatted for readability and includes ANSI escape codes to clear the console screen.
 * This function is typically called periodically to provide real-time updates on the BMS status.
 */
void printbmsstat()
{
    // Serial.print("\033[H\033[J"); // ANSI Escape code to clear screen
    SERIAL_CONSOLE.println();
    SERIAL_CONSOLE.println();
    SERIAL_CONSOLE.println(testcount);
    SERIAL_CONSOLE.print("BMS Status : ");
    if (settings.ESSmode == 1)
    {
        SERIAL_CONSOLE.print("ESS Mode ");

        if (bms.getLowCellVolt() < settings.UnderVSetpoint)
        {
            SERIAL_CONSOLE.print(": UnderVoltage ");
        }
        if (bms.getHighCellVolt() > settings.OverVSetpoint)
        {
            SERIAL_CONSOLE.print(": OverVoltage ");
        }
        if ((bms.getHighCellVolt() - bms.getLowCellVolt()) > settings.CellGap)
        {
            SERIAL_CONSOLE.print(": Cell Imbalance ");
        }
        if (bms.getAvgTemperature() > settings.OverTSetpoint)
        {
            SERIAL_CONSOLE.print(": Over Temp ");
        }
        if (bms.getAvgTemperature() < settings.UnderTSetpoint)
        {
            SERIAL_CONSOLE.print(": Under Temp ");
        }
        if (storagemode == 1)
        {
            if (bms.getLowCellVolt() > settings.StoreVsetpoint)
            {
                SERIAL_CONSOLE.print(": OverVoltage Storage ");
                SERIAL_CONSOLE.print(": UNhappy:");
            }
            else
            {
                SERIAL_CONSOLE.print(": Happy ");
            }
        }
        else
        {
            if (bms.getLowCellVolt() > settings.UnderVSetpoint && bms.getHighCellVolt() < settings.OverVSetpoint)
            {

                if (bmsstatus == BMS_STATUS_ERROR)
                {
                    SERIAL_CONSOLE.print(": UNhappy:");
                }
                else
                {
                    SERIAL_CONSOLE.print(": Happy ");
                }
            }
        }
    }
    else
    {
        SERIAL_CONSOLE.print(bmsstatus);
        switch (bmsstatus)
        {
        case (BMS_STATUS_BOOT):
            SERIAL_CONSOLE.print(" Boot ");
            break;

        case (BMS_STATUS_READY):
            SERIAL_CONSOLE.print(" Ready ");
            break;

        case (BMS_STATUS_PRECHARGE):
            SERIAL_CONSOLE.print(" Precharge ");
            break;

        case (BMS_STATUS_DRIVE):
            SERIAL_CONSOLE.print(" Drive ");
            break;

        case (BMS_STATUS_CHARGE):
            SERIAL_CONSOLE.print(" Charge ");
            break;

        case (BMS_STATUS_ERROR):
            SERIAL_CONSOLE.print(" Error ");
            break;
        }
    }
    SERIAL_CONSOLE.print("  ");
    if (digitalRead(PIN_IN3) == HIGH)
    {
        SERIAL_CONSOLE.print("| AC Present |");
    }
    if (digitalRead(PIN_IN1) == HIGH)
    {
        SERIAL_CONSOLE.print("| Key ON |");
    }
    if (balancecells == 1)
    {
        SERIAL_CONSOLE.print("|Balancing Active");
    }
    SERIAL_CONSOLE.print("  ");
    SERIAL_CONSOLE.print(cellspresent);
    SERIAL_CONSOLE.println();
    SERIAL_CONSOLE.print("Out:");
    SERIAL_CONSOLE.print(digitalRead(PIN_OUT1));
    SERIAL_CONSOLE.print(digitalRead(PIN_OUT2));
    SERIAL_CONSOLE.print(digitalRead(PIN_OUT3));
    SERIAL_CONSOLE.print(digitalRead(PIN_OUT4));
    SERIAL_CONSOLE.print(" Cont:");
    if ((contstat & 1) == 1)
    {
        SERIAL_CONSOLE.print("1");
    }
    else
    {
        SERIAL_CONSOLE.print("0");
    }
    if ((contstat & 2) == 2)
    {
        SERIAL_CONSOLE.print("1");
    }
    else
    {
        SERIAL_CONSOLE.print("0");
    }
    if ((contstat & 4) == 4)
    {
        SERIAL_CONSOLE.print("1");
    }
    else
    {
        SERIAL_CONSOLE.print("0");
    }
    if ((contstat & 8) == 8)
    {
        SERIAL_CONSOLE.print("1");
    }
    else
    {
        SERIAL_CONSOLE.print("0");
    }
    SERIAL_CONSOLE.print(" In:");
    SERIAL_CONSOLE.print(digitalRead(PIN_IN1));
    SERIAL_CONSOLE.print(digitalRead(PIN_IN2));
    SERIAL_CONSOLE.print(digitalRead(PIN_IN3));
    SERIAL_CONSOLE.print(digitalRead(PIN_IN4));

    SERIAL_CONSOLE.print(" Charge Current Limit : ");
    SERIAL_CONSOLE.print(chargecurrent * 0.1, 0);
    SERIAL_CONSOLE.print(" A DisCharge Current Limit : ");
    SERIAL_CONSOLE.print(discurrent * 0.1, 0);
    SERIAL_CONSOLE.print(" A");

    if (bmsstatus == BMS_STATUS_CHARGE || accurlim > 0)
    {
        Serial.print("  CP AC Current Limit: ");
        Serial.print(accurlim);
        Serial.print(" A");
    }

    if (bmsstatus == BMS_STATUS_CHARGE && CPdebug == 1)
    {
        Serial.print("A  CP Dur: ");
        Serial.print(duration);
        Serial.print("  Charge Power : ");
        Serial.print(chargerpower);
        if (chargecurrentlimit == false)
        {
            SERIAL_CONSOLE.print("  No Charge Current Limit");
        }
        else
        {
            SERIAL_CONSOLE.print("  Charge Current Limit Active");
        }
    }
}
