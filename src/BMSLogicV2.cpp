/**
 * @file BMSLogic.cpp
 * @brief Re-Write of the operational Logic for BMS Comms Board functions.
 *
 */

#include "BMSLogicV2.h"
#include "globals.h"
#include "Settings.h"
#include "TeslaBMSV2.h"
#include "pinouts.h"
#include "Logger.h"

/**
 * @brief Periodic loop for BMS functions
 * Separated into Fast (Comms) and Slow (Data/Safety) loops
 */
void bmsLoop()
{
    // // -------------------------
    // // Communication loop
    // // -------------------------
    // static unsigned long nextChargerCommsLoop = 0;
    // if (millis() > nextChargerCommsLoop)
    // {
    //     nextChargerCommsLoop = millis() + settings.chargerspd;

    //     // 1. Victron / External Comms
    //     VEcan(); 

    //     // 2. Charger Control Comms
    //     // Only talk to charger if we are actually in a state that allows charging
    //     // OR if we are in ESS mode (always connected)
    //     if (settings.ESSmode == 1 || bmsstatus == BMS_STATUS_CHARGE)
    //     {
    //         chargercomms();
    //     }
    // }

    if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL || settings.cursens == CURR_SENSE_ANALOGUE_GUESSING) {
        getcurrent();
    }

    // -------------------------
    // BMS logic loop
    // -------------------------
    if (millis() > nextLoopTime)
    {
        nextLoopTime = millis() + 500;

        // 1. PAUSE Balancing to get a clean reading
        // (We don't want the voltage drop from the bleed resistors affecting the reading)
        Logger::debug("Stop balancing");
        bms.StopBalancing();

        Logger::debug("Reading volt and temp");
        // 2. Read Data
        bms.getAllVoltTemp();
        
        Logger::debug("Checking voltages and temps");
        // 3. Checks
        checkCellVoltages();
        checkCellTemps();
        
        Logger::debug("Updating SOC");
        // 4. Update Logic
        updateSOC();
        Logger::debug("Updating current limit");
        currentlimit();
        
        // 5. RESUME Balancing (if required)
        // The 'balancing()' function checks the settings and voltages 
        // and re-enables balancing if needed.
        Logger::debug("Enable balancing");
        balancing(); 

        // 6. Debugging & Housekeeping
        if (debugMode != 0) {
            printbmsstat();
            bms.printPackDetails(debugdigits);
        }
        if (CSVdebug != 0) {
            bms.printAllCSV(millis(), currentact, SOC, delim);
        }
        if(gaugeEnabled == 1) {
            gaugeUpdate();
        }
        
        // 7. Module Management
        // Check if cell count matches expected
        if (bms.seriescells() != settings.Scells)
        {
             setBMSstatus(BMS_STATUS_ERROR, ERROR_BATTERY_COMMS, "Cell Count Mismatch");
        }

        alarmupdate(); // Update the alarm flags for CAN transmission
        
        // clear modules every 5 seconds to ensure we aren't reading stale data
        static unsigned long cleartime = 0;
        if (millis() - cleartime > 5000)
        {
            bms.clearmodules();
            cleartime = millis();
        }
    }
}

/**
 * @brief Checks for Undervoltage and Overvoltage events
 */
void checkCellVoltages()
{
    // -- UNDERVOLTAGE CHECK --
    if (bms.getLowCellVolt() < settings.UnderVSetpoint)
    {
        if (underVoltTripTimer > millis()) // Check if timer has expired
        {
            // Create a safe buffer for the error message
            char errBuf[64];
            snprintf(errBuf, sizeof(errBuf), "Undervoltage: %.3fV", bms.getLowCellVolt());
            
            setBMSstatus(BMS_STATUS_ERROR, ERROR_VOLTAGE, errBuf);
        }
    }
    else
    {
        // Reset the trip timer if voltage is OK
        underVoltTripTimer = millis() + settings.triptime;
    }

    // -- OVERVOLTAGE CHECK --
    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
        if (overVoltTripTimer > millis()) 
        {
            char errBuf[64];
            snprintf(errBuf, sizeof(errBuf), "Overvoltage: %.3fV", bms.getHighCellVolt());

            setBMSstatus(BMS_STATUS_ERROR, ERROR_VOLTAGE, errBuf);
        }
    }
    else
    {
        overVoltTripTimer = millis() + settings.triptime;
    }
}

/**
 * @brief Checks for Over Temperature events
 */
void checkCellTemps()
{   
    float avg_temp = bms.getAvgTemperature();
    float max_temp = bms.getHighTemperature();
    float min_temp = bms.getLowTemperature();
    Logger::debug("Avg temp: %f", avg_temp); // TODO: this is another completely stupid thing, temps (also high & low) are only processed in this function call
    Logger::debug("Low temp: %f", min_temp);
    Logger::debug("High temp: %f", max_temp);

    if (max_temp> settings.OverTSetpoint)
    {
        char errBuf[64];
        snprintf(errBuf, sizeof(errBuf), "Over Temp: %.1fC", max_temp);
        
        setBMSstatus(BMS_STATUS_ERROR, ERROR_OVER_TEMPERATURE, errBuf);
    }
    else if(min_temp < settings.UnderTSetpoint){
        char errBuf[64];
        snprintf(errBuf, sizeof(errBuf), "Under Temp: %.1fC", min_temp);
        setBMSstatus(BMS_STATUS_ERROR, ERROR_UNDER_TEMPERATURE, errBuf);
    }
}

/**
 * @brief Checks for Overcurrent Conditions (Software Fuse)
 */
void checkCurrent()
{
    // 1. DISCHARGE Overcurrent Check
    if (Discharge == 1)
    {
        // HARD Limit (Instant Trip) - 2x discharge current
        if (currentact > (settings.discurrentmax * 2)) 
        {
             setBMSstatus(BMS_STATUS_ERROR, ERROR_DISCHARGE_CURRENT, "Hard Overcurrent Trip!");
             return;
        }

        // SOFT Limit (Timed Trip)
        // If we are exceeding the rated max for longer than 'triptime'
        if (currentact > settings.discurrentmax)
        {
            if (overCurrentTripTimer > millis()) 
            {
                 char errBuf[64];
                 snprintf(errBuf, sizeof(errBuf), "Overcurrent: %.1fA. Exceeded %ims trip time", currentact, settings.triptime);
                 setBMSstatus(BMS_STATUS_ERROR, ERROR_DISCHARGE_CURRENT, errBuf);
            }
        }
        else
        {
            // Reset timer if current returns to normal
            overCurrentTripTimer = millis() + settings.triptime; 
        }
    }

    // 2. CHARGE Overcurrent Check
    // Note: 'currentact' is often negative during charge depending on sensor orientation.
    // You generally want to use 'abs(currentact)' or handle the sign correctly.
    // Assuming currentact is Positive = Discharge, Negative = Charge here:
    
    if (currentact < 0 && abs(currentact) > settings.chargecurrentmax)
    {
         if (overCurrentTripTimer > millis()) 
        {
             char errBuf[64];
             snprintf(errBuf, sizeof(errBuf), "Charge Overcurrent: %.1fA", currentact);
             setBMSstatus(BMS_STATUS_ERROR, ERROR_CHARGE_CURRENT, errBuf);
        }
    }
}

/**
 * @brief Manage the output of the BMS. This function is called from the main loop
 *
 * This function is responsible for managing the output of the BMS. It controls the charging and discharging of the battery, as well as the balancing of the cells. It also handles the error state of the BMS.
 *
 * In ESS mode, this function is responsible for controlling the contactors and chargers. It will also turn on the discharge contactor if the key is turned on and the voltage is above the setpoint.
 *
 * In READY, PRECHARGE, DRIVE, CHARGE and ERROR states, this function is responsible for controlling the chargers and contactors. It will also turn on the discharge contactor if the key is turned on and the voltage is above the setpoint.
 *
 * @param[in] None
 * @return None
 */
/**
 * @brief Revised Output Check using BMS_Contactor objects
 */
void outputCheck()
{
    Logger::debug("Entering Output Check Loop");

    // 1. Tick the Contactor State Machines
    // This handles economizers, delayed closing/opening, etc.
    updateContactors();

    // 2. Global Error Checks (Overrides everything)
    if (bmsstatus == BMS_STATUS_ERROR)
    {
        openAllContactors();
        
        // Attempt Recovery if conditions allow (Key must be OFF)
        if (digitalRead(PIN_IGNITION) == LOW) 
        {
            if (bms.getLowCellVolt() >= settings.UnderVSetpoint && 
                bms.getHighCellVolt() <= settings.OverVSetpoint)
            {
                setBMSstatus(BMS_STATUS_READY, "Conditions returned to normal");
                bmsError = ERROR_NONE;
            }
        }
        return; // Exit immediately
    }

    // 3. Main State Machine
    switch (bmsstatus)
    {
        case BMS_STATUS_BOOT:
            openAllContactors();
            setBMSstatus(BMS_STATUS_READY, "Boot complete");
            break;

        case BMS_STATUS_READY:
            openAllContactors();
            
            // -- ESS Mode Start Logic --
            if (settings.ESSmode == 1)
            {
                bool essEnabled = (settings.ChargerDirect == 1) || (digitalRead(PIN_IN2) == HIGH) || (CanOnReq == true);
                
                if (essEnabled && bms.getHighCellVolt() < settings.OverVSetpoint && bms.getLowCellVolt() > settings.UnderVSetpoint)
                {
                    setBMSstatus(BMS_STATUS_PRECHARGE, "ESS Enable Detected");
                    Pretimer = millis(); // Start the precharge clock
                }
            }
            // -- Vehicle Mode Start Logic --
            else 
            {
                if (bms.getLowCellVolt() > settings.DischVsetpoint) // && digitalRead(PIN_IGNITION) == HIGH)  //ADD BACK IN
                {
                    setBMSstatus(BMS_STATUS_PRECHARGE, "Key ON detected");
                    Pretimer = millis();
                }
                else if (digitalRead(PIN_CHARGE) == HIGH && bms.getHighCellVolt() < settings.ChargeVsetpoint)
                {
                     setBMSstatus(BMS_STATUS_CHARGE, "AC detected");
                     Pretimer = millis();
                }
            }
            break;

        case BMS_STATUS_PRECHARGE:
            Discharge = 0;
            
            // 1. Always close Negative first (if not already)
            if(contactors.negative.isOpen()) {
                contactors.negative.close();
            }

            // Note: You could use .close(delay) here if you wanted to stagger them
            if(contactors.precharge.isOpen()) {
                contactors.precharge.close();
            }

            // 3. Check Precharge Completion Conditions
            // Time passed AND Current is low enough (Capacitors charged)
            if (millis() > (Pretimer + settings.Pretime)) //ADD BACK IN && abs(currentact) < settings.Precurrent)
            {
                // Close Main Positive
                contactors.positive.close();

                // We wait for the contactor object to confirm it is CLOSED (it might have a mechanical delay)
                if (contactors.positive.isClosed())
                {
                    contactors.precharge.open();
                    setBMSstatus(BMS_STATUS_DRIVE, "Precharge Complete");
                }
            }
            
            // Timeout Safety (Optional but recommended)
            if (millis() > (Pretimer + settings.Pretime + 5000)) {
                 setBMSstatus(BMS_STATUS_ERROR, ERROR_PRECHARGE_TIMEOUT, "Precharge Timeout - Voltage didn't rise?");
            }
            break;

        case BMS_STATUS_DRIVE:
            Discharge = 1;
            
            // Ensure Main Contactors are closed (in case we came from a weird state)
            // The class ignores .close() if it's already closed, so this is safe.
            contactors.negative.close();
            contactors.positive.close();

            // -- VEHICLE MODE --
            if (settings.ESSmode == 0)
            {
                // if (digitalRead(PIN_IGNITION) == LOW) {  //ADD BACK IN
                //     setBMSstatus(BMS_STATUS_READY, "Key OFF detected");
                // }
                if (digitalRead(PIN_CHARGE) == HIGH) {
                    setBMSstatus(BMS_STATUS_CHARGE, "AC detected during Drive");
                }
            }
            // -- ESS MODE --
            else 
            {                
                bool essEnabled = (settings.ChargerDirect == 1) || (digitalRead(PIN_IN2) == HIGH) || (CanOnReq == true);
                if (!essEnabled) {
                     setBMSstatus(BMS_STATUS_READY, "ESS Disable Request");
                }
            }
            break;

        case BMS_STATUS_CHARGE:
            Discharge = 0;

            // If charging is provded by a seperate charge contactor then
            // change over to the charge contactor
            if(settings.ChargerDirect == 0){
                contactors.positive.open(); // Ensure drive contactors open
                contactors.charge.close(); // Close dedicated Charge Relay
            }
            else{
                contactors.negative.close();
                contactors.positive.close();
            }
            
            
            if (bms.getHighCellVolt() > settings.ChargeVsetpoint || bms.getHighTemperature() > settings.OverTSetpoint)
            {
                contactors.charge.open();
                contactors.positive.open();

                // Check the average cell voltage level to determine the SOC
                if (bms.getAvgCellVolt() > (settings.ChargeVsetpoint - settings.ChargeHys))
                {
                    // 100% SOC
                    SOCcharged(2);
                }
                else
                {
                    // 95% SOC
                    SOCcharged(1);
                }

                setBMSstatus(BMS_STATUS_READY, "Charge Complete");
            }
            
            if (digitalRead(PIN_CHARGE) == LOW && settings.ESSmode == 0) {
                setBMSstatus(BMS_STATUS_READY, "AC Removed");
            }
            break;
    }
}

void updateContactors()
{
    contactors.positive.update();
    contactors.precharge.update();
    contactors.charge.update();
    contactors.negative.update();
    contactors.trip.update();
}

void openAllContactors()
{
    contactors.positive.open();
    contactors.precharge.open();
    contactors.charge.open();
    contactors.negative.open();
    contactors.trip.open(); // Does this need to be inverted?
}

/**
 * @brief Manages cell balancing based on the balancecells setting.
 *
 * This function checks the `balancecells` setting to determine whether
 * cell balancing should be enabled or disabled. If balancing is enabled
 * (i.e., `balancecells` is set to 1), it calls the `balanceCells` method
 * of the BMS object with the specified balance duty cycle. If balancing
 * is disabled, it stops the balancing process by calling the `StopBalancing`
 * method of the BMS object.
 *
 * The function includes a debug mode that can be activated by setting
 * the `debug` variable to 1, although in this implementation, both
 * debug and non-debug modes perform the same action.
 */
void balancing()
{
    if (balancecells == 1)
    {
        bms.balanceCells(settings.balanceDuty, debugMode);
    }
    else
    {
        bms.StopBalancing();
    }
}
