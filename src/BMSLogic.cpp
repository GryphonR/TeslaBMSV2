/**
 * @file BMSLogic.cpp
 * @brief Operational Logic for BMS Comms Board functions.
 *
 */

#include "BMSLogic.h"
#include "globals.h"
#include "Settings.h"
#include "TeslaBMSV2.h"
#include "pinouts.h"
#include "Logger.h"

/**
 * @brief Periodic loop for BMS functions
 *
 * This function is called every 500ms to update BMS data, check for errors, and perform other
 * necessary functions. It checks for undervoltage and overvoltage conditions in both ESS and
 * vehicle modes, and sets the appropriate alarm states. It also updates the SOC and current
 * limits, and checks for other errors.
 *
 * @note This function should be called periodically to ensure proper BMS function.
 */
void bmsLoop()
{
    if (millis() > nextLoopTime)
    {
        nextLoopTime = millis() + 500; // Every 500ms
        Logger::debug("Before bms.getAllVoltTemp()");

        // Update BMS data
        bms.getAllVoltTemp();

        Logger::debug("After bms.getAllVoltTemp()");
        // Cell Voltage  check
        if (settings.ESSmode == 1) // ESS Mode
        {
            Logger::debug("Entering ESS Mode Loop");
            // Check if Lowest cell or highest cell is below the Undervoltage setpoint. (Why lowest and highest??)
            if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getHighCellVolt() < settings.UnderVSetpoint)
            {
                if (undertriptimer > millis()) // check is last time not undervoltage is longer than UnderDur ago
                {
                    // bmsstatus = BMS_STATUS_ERROR;
                    setBMSstatus(BMS_STATUS_ERROR, "Undervoltage detected in ESS mode");
                    bmsError = ERROR_VOLTAGE; // TODO - refine error state
                }
            }
            else
            {
                undertriptimer = millis() + settings.triptime;
            }

            if (bms.getLowCellVolt() > settings.OverVSetpoint || bms.getHighCellVolt() > settings.OverVSetpoint)
            {
                if (overtriptimer > millis()) // check is last time not undervoltage is longer thatn UnderDur ago
                {
                    // bmsstatus = BMS_STATUS_ERROR;
                    setBMSstatus(BMS_STATUS_ERROR, "Overvoltage detected in ESS mode");
                    bmsError = ERROR_VOLTAGE; // TODO - refine error state
                }
            }
            else
            {
                overtriptimer = millis() + settings.triptime;
            }
        }
        else // In 'vehicle' mode
        {
            if (bms.getLowCellVolt() < settings.UnderVSetpoint)
            {
                if (UnderTimer < millis()) // check is last time not undervoltage is longer thatn UnderDur ago
                {
                    // bmsstatus = BMS_STATUS_ERROR;
                    setBMSstatus(BMS_STATUS_ERROR, "Undervoltage detected in vehicle mode");
                    bmsError = ERROR_VOLTAGE; // TODO - refine error state
                }
            }
            else
            {
                UnderTimer = millis() + settings.triptime;
            }

            if (bms.getHighCellVolt() < settings.UnderVSetpoint || bms.getHighTemperature() > settings.OverTSetpoint)
            {
                // bmsstatus = BMS_STATUS_ERROR;
                setBMSstatus(BMS_STATUS_ERROR, "Undervoltage or Overtemperature detected in vehicle mode");
                bmsError = ERROR_VOLTAGE; // TODO - refine error state
            }

            if (bms.getHighCellVolt() > settings.OverVSetpoint)
            {
                if (OverTime < millis()) // check is last time not undervoltage is longer thatn UnderDur ago
                {
                    bmsstatus = BMS_STATUS_ERROR;
                    setBMSstatus(BMS_STATUS_ERROR, "Overvoltage detected in vehicle mode");
                    bmsError = ERROR_VOLTAGE; // TODO - refine error state
                }
            }
            else
            {
                OverTime = millis() + settings.triptime;
            }
        }

        balancing();

        if (debug != 0)
        {
            printbmsstat();
            bms.printPackDetails(debugdigits);
        }
        if (CSVdebug != 0)
        {
            bms.printAllCSV(millis(), currentact, SOC, delim);
        }
        if (inputcheck != 0)
        {
            inputdebug();
        }

        if (outputcheck != 0)
        {
            outputdebug();
        }
        else
        {
            gaugeUpdate();
        }
        updateSOC();
        currentlimit();

        Logger::debug("ChargerDirect: %d, canOnRev: %d, CanOnReq: %d", settings.ChargerDirect, CanOnRev, CanOnReq);

        if (settings.ESSmode == 1 && settings.ChargerDirect == 0 && CanOnRev == true)
        {
            if ((millis() - CanOntimeout) > 5000)
            {
                Serial.println();
                Serial.println("0x309 Can On Request Missing");
                CanOnReq = false;
            }
        }

        Logger::debug("BMS Status: %d, SOCset: %d, cellspresent: %d", bmsstatus, SOCset, cellspresent);
        if (cellspresent == 0 && SOCset == 1)
        {
            cellspresent = bms.seriescells();
            bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
        }
        else
        {
            if (cellspresent != bms.seriescells() || cellspresent != (settings.Scells * settings.Pstrings)) // detect a fault in cells detected
            {
                if (debug != 0)
                {
                    SERIAL_CONSOLE.println("  ");
                    SERIAL_CONSOLE.print("   !!! Series Cells Fault !!!");
                    SERIAL_CONSOLE.println("  ");
                    bmsstatus = BMS_STATUS_ERROR;
                    setBMSstatus(BMS_STATUS_ERROR, "Series Cells count fault detected");
                    bmsError = ERROR_VOLTAGE; // TODO - refine error state
                }
            }
        }
        alarmupdate();
        if (CSVdebug != 1)
        {
            Logger::debug("BeforeDash Update");
            // dashupdate(); //TODO - Hanging in Dash Update!
        }
        Logger::debug("After Dash Update");

    } // end of 500ms loop

// Why are modules cleared every 5 seconds?
    if (millis() - cleartime > 5000)
    {
        Logger::debug("Clearing modules");
        cleartime = millis();
        bms.clearmodules();
        Logger::debug("Cleared modules");
    }

    static unsigned long nextChargerCommsLoop = millis(); // prevents rapid iterations until the increment has caught up with millis();
    if (millis() > nextChargerCommsLoop)
    {
        Logger::debug("Entering Charger Comms");
        nextChargerCommsLoop += settings.chargerspd;
        VEcan();
        if (settings.ESSmode == 1)
        {
            chargercomms();
        }
        else
        {
            if (bmsstatus == BMS_STATUS_CHARGE)
            {
                chargercomms();
            }
        }
        Logger::debug("Leaving Charger Comms");
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
void outputCheck()
{   
    Logger::debug("Entering Output Check Loop");
    if (outputcheck != 1)
    {   

        contactorControl();
        if (settings.ESSmode == 1)
        {
            Logger::debug("Entering ESS Mode Loop");
            if (settings.ChargerDirect == 1)
            {
                OutputEnable = 1;
            }
            else
            {
                if (digitalRead(PIN_IN2) == HIGH || CanOnReq == true)
                {
                    OutputEnable = 1;
                    // Serial.println(CanOnReq);
                }
                else
                {
                    OutputEnable = 0;
                }
            }
            // Status not error or boot, outputCheck = 1
            if (bmsstatus != BMS_STATUS_ERROR && bmsstatus != BMS_STATUS_BOOT && OutputEnable == 1)
            {
                contctrl = contctrl | 4; // turn on negative contactor
                if (settings.tripcont != 0)
                {
                    if (bms.getLowCellVolt() > settings.UnderVSetpoint && bms.getHighCellVolt() < settings.OverVSetpoint)
                    {
                        if (digitalRead(PIN_OUT2) == LOW && digitalRead(PIN_OUT4) == LOW)
                        {
                            mainconttimer = millis();
                            digitalWrite(PIN_OUT4, HIGH); // Precharge start
                            Serial.println();
                            Serial.println("Precharge!!!");
                            Serial.printf("Conditions: Output2: %d, output4: %d, bmsStatus: %d\n", digitalRead(PIN_OUT2), digitalRead(PIN_OUT4), bmsstatus);
                            Serial.println();
                            Serial.println(mainconttimer);
                            Serial.println();
                        }
                        if (mainconttimer + settings.Pretime < millis() && digitalRead(PIN_OUT2) == LOW && abs(currentact) < settings.Precurrent)
                        {
                            digitalWrite(PIN_OUT2, HIGH); // turn on contactor
                            contctrl = contctrl | 2;      // turn on contactor
                            Serial.println();
                            Serial.println("Main On!!!");
                            Serial.println();
                            mainconttimer = millis() + settings.Pretime;
                        }
                        if (mainconttimer + settings.Pretime + 1000 < millis())
                        {
                            digitalWrite(PIN_OUT4, LOW); // ensure precharge is low
                        }
                    }
                    else
                    {
                        digitalWrite(PIN_OUT4, LOW); // ensure precharge is low
                        mainconttimer = 0;
                    }
                }
                if (digitalRead(PIN_IN1) == LOW) // Key OFF
                {
                    if (storagemode == 1)
                    {
                        storagemode = 0;
                    }
                }
                else
                {
                    if (storagemode == 0)
                    {
                        storagemode = 1;
                    }
                }
                if (bms.getHighCellVolt() > settings.balanceVoltage && bms.getHighCellVolt() > bms.getLowCellVolt() + settings.balanceHyst)
                {
                    balancecells = 1;
                }
                else
                {
                    balancecells = 0;
                }

                // Pretimer + settings.Pretime > millis();

                if (storagemode == 1)
                {
                    if (bms.getHighCellVolt() > settings.StoreVsetpoint || chargecurrent == 0)
                    {
                        digitalWrite(PIN_OUT3, LOW); // turn off charger
                        // contctrl = contctrl & 253;
                        // Pretimer = millis();
                        Charged = 1;
                        SOCcharged(2);
                    }
                    else
                    {
                        if (Charged == 1)
                        {
                            if (bms.getHighCellVolt() < (settings.StoreVsetpoint - settings.ChargeHys))
                            {
                                Charged = 0;
                                digitalWrite(PIN_OUT3, HIGH); // turn on charger
                                                              /*
                                                                if (Pretimer + settings.Pretime < millis())
                                                                {
                                                                contctrl = contctrl | 2;
                                                                Pretimer = 0;
                                                                }
                                                              */
                            }
                        }
                        else
                        {
                            digitalWrite(PIN_OUT3, HIGH); // turn on charger
                                                          /*
                                                            if (Pretimer + settings.Pretime < millis())
                                                            {
                                                            contctrl = contctrl | 2;
                                                            Pretimer = 0;
                                                            }
                                                          */
                        }
                    }
                }
                else
                {
                    if (bms.getHighCellVolt() > settings.OverVSetpoint || bms.getHighCellVolt() > settings.ChargeVsetpoint || chargecurrent == 0)
                    {
                        if ((millis() - overtriptimer) > settings.triptime)
                        {
                            if (digitalRead(PIN_OUT3) == 1)
                            {
                                Serial.println();
                                Serial.println("Over Voltage Trip");
                                digitalWrite(PIN_OUT3, LOW); // turn off charger
                                // contctrl = contctrl & 253;
                                // Pretimer = millis();
                                Charged = 1;
                                SOCcharged(2);
                            }
                        }
                    }
                    else
                    {
                        overtriptimer = millis();
                        if (Charged == 1)
                        {

                            if (bms.getHighCellVolt() < (settings.ChargeVsetpoint - settings.ChargeHys))
                            {
                                if (digitalRead(PIN_OUT3) == 0)
                                {
                                    Serial.println();
                                    Serial.println("Reset Over Voltage Trip Not Charged");
                                    Charged = 0;
                                    digitalWrite(PIN_OUT3, HIGH); // turn on charger
                                }
                                /*
                                  if (Pretimer + settings.Pretime < millis())
                                  {
                                  // Serial.println();
                                  //Serial.print(Pretimer);
                                  contctrl = contctrl | 2;
                                  }*/
                            }
                        }
                        else
                        {
                            if (digitalRead(PIN_OUT3) == 0)
                            {
                                Serial.println();
                                Serial.println("Reset Over Voltage Trip Not Charged");
                                digitalWrite(PIN_OUT3, HIGH); // turn on charger
                            }
                            /*
                              if (Pretimer + settings.Pretime < millis())
                              {
                              // Serial.println();
                              //Serial.print(Pretimer);
                              contctrl = contctrl | 2;
                              }*/
                        }
                    }
                }

                if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getLowCellVolt() < settings.DischVsetpoint)
                {
                    if (digitalRead(PIN_OUT1) == 1)
                    {

                        if ((millis() - undertriptimer) > settings.triptime)
                        {
                            Serial.println();
                            Serial.println("Under Voltage Trip");
                            digitalWrite(PIN_OUT1, LOW); // turn off discharge
                                                         // contctrl = contctrl & 254;
                                                         // Pretimer1 = millis();
                        }
                    }
                }
                else
                {
                    undertriptimer = millis();

                    if (bms.getLowCellVolt() > settings.DischVsetpoint + settings.DischHys)
                    {
                        if (digitalRead(PIN_OUT1) == 0)
                        {
                            Serial.println();
                            Serial.println("Reset Under Voltage Trip");
                            digitalWrite(PIN_OUT1, HIGH); // turn on discharge
                        }
                        /*
                          if (Pretimer1 + settings.Pretime < millis())
                          {
                          contctrl = contctrl | 1;
                          }*/
                    }
                }

                if (SOCset == 1)
                {
                    if (settings.tripcont == 0)
                    {
                        if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getHighCellVolt() > settings.OverVSetpoint || bms.getHighTemperature() > settings.OverTSetpoint)
                        {
                            digitalWrite(PIN_OUT2, HIGH); // trip breaker
                            // bmsstatus = BMS_STATUS_ERROR;
                            setBMSstatus(BMS_STATUS_ERROR, "Voltage or Temperature fault in ESS mode with tripcont=0");
                            bmsError = ERROR_VOLTAGE; // TODO - refine error state
                                                      // TODO - break out the above if statement to assign correct error
                        }
                        else
                        {
                            digitalWrite(PIN_OUT2, LOW); // trip breaker
                        }
                    }
                    else
                    {
                        if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getHighCellVolt() > settings.OverVSetpoint || bms.getHighTemperature() > settings.OverTSetpoint)
                        {
                            digitalWrite(PIN_OUT2, LOW); // turn off contactor
                            contctrl = contctrl & 253;   // turn off contactor
                            digitalWrite(PIN_OUT4, LOW); // ensure precharge is low
                            // bmsstatus = BMS_STATUS_ERROR;
                            setBMSstatus(BMS_STATUS_ERROR, "Voltage or Temperature fault in ESS mode with tripcont=1");
                            bmsError = ERROR_VOLTAGE; // TODO - refine error state
                        }
                    }
                }
            }
            else
            {
                // digitalWrite(OUT2, HIGH);//trip breaker
                // Logger::debug("ESS Mode - All outputs turning off - In Error, Boot or Output not enabled state");
                Discharge = 0;
                digitalWrite(PIN_OUT4, LOW);
                digitalWrite(PIN_OUT3, LOW); // turn off charger
                digitalWrite(PIN_OUT2, LOW);
                digitalWrite(PIN_OUT1, LOW); // turn off discharge
                contctrl = 0;                // turn off out 5 and 6

                if (SOCset == 1)
                {
                    if (settings.tripcont == 0)
                    {

                        digitalWrite(PIN_OUT2, HIGH); // trip breaker
                    }
                    else
                    {
                        digitalWrite(PIN_OUT2, LOW); // turn off contactor
                        digitalWrite(PIN_OUT4, LOW); // ensure precharge is low
                    }

                    if (bms.getLowCellVolt() > settings.UnderVSetpoint && bms.getHighCellVolt() < settings.OverVSetpoint && bms.getHighTemperature() < settings.OverTSetpoint && cellspresent == bms.seriescells() && cellspresent == (settings.Scells * settings.Pstrings))
                    {
                        // bmsstatus = BMS_STATUS_READY;
                        setBMSstatus(BMS_STATUS_READY, "Voltage and Temperature normal in ESS mode");
                        bmsError = ERROR_NONE;
                    }
                }
            }
            // pwmcomms();
        }
        else
        {
            switch (bmsstatus)
            {
            case (BMS_STATUS_BOOT):
                Discharge = 0;
                digitalWrite(PIN_OUT4, LOW);
                digitalWrite(PIN_OUT3, LOW); // turn off charger
                digitalWrite(PIN_OUT2, LOW);
                digitalWrite(PIN_OUT1, LOW); // turn off discharge
                contctrl = 0;
                // bmsstatus = BMS_STATUS_READY;
                setBMSstatus(BMS_STATUS_READY, "Boot complete");
                break;

            case (BMS_STATUS_READY):
                Discharge = 0;
                digitalWrite(PIN_OUT4, LOW);
                digitalWrite(PIN_OUT3, LOW); // turn off charger
                digitalWrite(PIN_OUT2, LOW);
                digitalWrite(PIN_OUT1, LOW); // turn off discharge
                contctrl = 0;                // turn off out 5 and 6
                accurlim = 0;
                if (bms.getHighCellVolt() > settings.balanceVoltage && bms.getHighCellVolt() > bms.getLowCellVolt() + settings.balanceHyst)
                {
                    // bms.balanceCells();
                    balancecells = 1;
                }
                else
                {
                    balancecells = 0;
                }
                if (digitalRead(PIN_IN3) == HIGH && (bms.getHighCellVolt() < (settings.ChargeVsetpoint - settings.ChargeHys)) && bms.getHighTemperature() < (settings.OverTSetpoint - settings.WarnToff)) // detect AC present for charging and check not balancing
                {
                    if (settings.ChargerDirect == 1)
                    {
                        // bmsstatus = BMS_STATUS_CHARGE;
                        setBMSstatus(BMS_STATUS_CHARGE, "AC detected in READY state with ChargerDirect=1");
                    }
                    else
                    {
                        // bmsstatus = BMS_STATUS_PRECHARGE;
                        setBMSstatus(BMS_STATUS_PRECHARGE, "AC detected in READY state with ChargerDirect=0");
                        Pretimer = millis();
                    }
                }
                if (digitalRead(PIN_IN1) == HIGH && bms.getLowCellVolt() > settings.DischVsetpoint) // detect Key ON
                {
                    // bmsstatus = BMS_STATUS_PRECHARGE;
                    setBMSstatus(BMS_STATUS_PRECHARGE, "Key ON detected in READY state");
                    Pretimer = millis();
                }

                break;

            case (BMS_STATUS_PRECHARGE):
                Discharge = 0;
                Prechargecon();
                break;

            case (BMS_STATUS_DRIVE):
                Discharge = 1;
                accurlim = 0;
                if (digitalRead(PIN_IN1) == LOW) // Key OFF
                {
                    // bmsstatus = BMS_STATUS_READY;
                    setBMSstatus(BMS_STATUS_READY, "Key OFF detected in DRIVE state");
                }
                if (digitalRead(PIN_IN3) == HIGH && (bms.getHighCellVolt() < (settings.ChargeVsetpoint - settings.ChargeHys)) && bms.getHighTemperature() < (settings.OverTSetpoint - settings.WarnToff)) // detect AC present for charging and check not balancing
                {
                    // bmsstatus = BMS_STATUS_CHARGE;
                    setBMSstatus(BMS_STATUS_CHARGE, "AC detected in DRIVE state");
                }

                break;

            case (BMS_STATUS_CHARGE):
                if (settings.ChargerDirect > 0)
                {
                    Discharge = 0;
                    digitalWrite(PIN_OUT4, LOW);
                    digitalWrite(PIN_OUT2, LOW);
                    digitalWrite(PIN_OUT1, LOW); // turn off discharge
                    contctrl = 0;                // turn off out 5 and 6
                }
                Discharge = 0;
                if (digitalRead(PIN_IN2) == HIGH)
                {
                    chargecurrentlimit = true;
                }
                else
                {
                    chargecurrentlimit = false;
                }
                digitalWrite(PIN_OUT3, HIGH); // enable charger
                if (bms.getHighCellVolt() > settings.balanceVoltage)
                {
                    // bms.balanceCells();
                    balancecells = 1;
                }
                else
                {
                    balancecells = 0;
                }
                if (bms.getHighCellVolt() > settings.ChargeVsetpoint || bms.getHighTemperature() > settings.OverTSetpoint)
                {
                    if (bms.getAvgCellVolt() > (settings.ChargeVsetpoint - settings.ChargeHys))
                    {
                        SOCcharged(2);
                    }
                    else
                    {
                        SOCcharged(1);
                    }
                    digitalWrite(PIN_OUT3, LOW); // turn off charger
                    // bmsstatus = BMS_STATUS_READY;
                    setBMSstatus(BMS_STATUS_READY, "Charge or Temperature setpoint reached in CHARGE state");
                }
                if (digitalRead(PIN_IN3) == LOW) // detect AC not present for charging
                {
                    // bmsstatus = BMS_STATUS_READY;
                    setBMSstatus(BMS_STATUS_READY, "AC not detected in CHARGE state");
                }
                break;

            case (BMS_STATUS_ERROR):
                Discharge = 0;
                digitalWrite(PIN_OUT4, LOW);
                digitalWrite(PIN_OUT3, LOW); // turn off charger
                digitalWrite(PIN_OUT2, LOW);
                digitalWrite(PIN_OUT1, LOW); // turn off discharge
                contctrl = 0;                // turn off out 5 and 6
                /*
                          if (digitalRead(IN3) == HIGH) //detect AC present for charging
                          {
                            bmsstatus = Charge;
                          }
                */
                if (bms.getLowCellVolt() >= settings.UnderVSetpoint && bms.getHighCellVolt() <= settings.OverVSetpoint && digitalRead(PIN_IN1) == LOW)
                {
                    // bmsstatus = BMS_STATUS_READY;
                    setBMSstatus(BMS_STATUS_READY, "Voltage and Key OFF normal in ERROR state");
                }

                break;
            }
        }
        if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL || settings.cursens == CURR_SENSE_ANALOGUE_GUESSING)
        {
            getcurrent();
        }
    }
    Logger::debug("Leaving Output Check Loop");
}
