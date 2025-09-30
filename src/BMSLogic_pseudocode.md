This file comprises of three loops.

## BMSLoop (function)
This function updates the BMS Status and Error conditions, calls the balancing function, some diagnostics,

#### Every 500ms
- Updates Voltage and Temperature data from the ModuleManager

    IF ESS Mode
            
        Checks if Lowest cell OR highest cell is below the Undervoltage Setpoint
            YES - checks if underTripTimer millis has elapsed since all voltages were ok
                    YES - Sets BMS Status Error
                        - DOESN'T trip anything??
            NO - Updates the underV trip Timer to millis() + triptime

        Checks if Lowest cell or highest cell is above the OverVoltage Setpoint
            YES - checks if overTripTimer millis has elapsed since all voltages were ok
                    YES - Sets BMS Status Error
                        - DOESN'T trip anything??
            NO - Updates the overV trip Timer to millis() + triptime

    IF Vehicle mode

        Checks if Lowest cell below the Undervoltage Setpoint
            YES - checks if underTripTimer millis has elapsed since all voltages were ok
                    YES - Sets BMS Status Error
                        - DOESN'T trip anything??
            NO - Updates the underV trip Timer to millis() + triptime

        Checks if highest cell is above the OverVoltage Setpoint
            YES - checks if overTripTimer millis has elapsed since all voltages were ok
                    YES - Sets BMS Status Error
                        - DOESN'T trip anything??
            NO - Updates the overV trip Timer to millis() + triptime

        Checks if highest cell voltage is below the Undervoltage Setpoint OR Highest Temperature is over teme Setpoint
            YES - Set Error Status

    Calls balancing function

    Checks Debug Flags
        debug? Print bms stat and pack details to serial
        CSVdebug? Print csv values to serial
        inputCheck? input debug function
        outputCheck? Output debug function ELSE gaugeUpdate

    Update SOC Func.

    Current Limit Func.

    