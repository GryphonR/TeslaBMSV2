#include "SerialMenu.h"
#include "config.h"
#include "globals.h"
#include "pinouts.h"
#include "CurrentSensing.h"
#include "PhysicalGauges.h"
#include "Settings.h"

#include <EEPROM.h>

// Settings menu
void menu()
{

    incomingByte = Serial.read(); // read the incoming byte:
    if (menuload == 4)
    {
        switch (incomingByte)
        {

        case '1':
            menuload = 1;
            candebug = !candebug;
            incomingByte = 'd';
            break;

        case '2':
            menuload = 1;
            debugCur = !debugCur;
            incomingByte = 'd';
            break;

        case '3':
            menuload = 1;
            outputcheck = !outputcheck;
            if (outputcheck == 0)
            {
                contctrl = 0;
                digitalWrite(PIN_OUT1, LOW);
                digitalWrite(PIN_OUT2, LOW);
                digitalWrite(PIN_OUT3, LOW);
                digitalWrite(PIN_OUT4, LOW);
            }
            incomingByte = 'd';
            break;

        case '4':
            menuload = 1;
            inputcheck = !inputcheck;
            incomingByte = 'd';
            break;

        case '5':
            menuload = 1;
            settings.ESSmode = !settings.ESSmode;
            incomingByte = 'd';
            break;

        case '6':
            menuload = 1;
            cellspresent = bms.seriescells();
            incomingByte = 'd';
            break;

        case '7':
            menuload = 1;
            gaugedebug = !gaugedebug;
            incomingByte = 'd';
            break;

        case '8':
            menuload = 1;
            CSVdebug = !CSVdebug;
            incomingByte = 'd';
            break;

        case '9':
            menuload = 1;
            if (Serial.available() > 0)
            {
                debugdigits = Serial.parseInt();
            }
            if (debugdigits > 4)
            {
                debugdigits = 2;
            }
            incomingByte = 'd';
            break;

        case '0':
            menuload = 1;
            CPdebug = !CPdebug;
            incomingByte = 'd';
            break;

        case 'd':
            menuload = 1;
            if (Serial.available() > 0)
            {
                delim = Serial.parseInt();
            }
            if (delim > 1)
            {
                delim = 0;
            }
            incomingByte = 'd';
            break;

        case 113: // q for quite menu

            menuload = 0;
            incomingByte = 115;
            break;

        default:
            // if nothing else matches, do the default
            // default is optional
            break;
        }
    }

    if (menuload == 9)
    {
        if (settings.ExpMess > 1)
        {
            settings.ExpMess = 0;
        }
        switch (incomingByte)
        {

        case '1':
            menuload = 1;
            settings.ExpMess = !settings.ExpMess;
            incomingByte = 'x';
            break;
        case 113: // q to go back to main menu
            menuload = 0;
            incomingByte = 115;
            break;
        }
    }

    if (menuload == 2)
    {
        switch (incomingByte)
        {

        case 99: // c for calibrate zero offset

            calcur();
            break;

        case '1':
            menuload = 1;
            settings.invertcur = !settings.invertcur;
            incomingByte = 'c';
            break;

        case '2':
            menuload = 1;
            settings.voltsoc = !settings.voltsoc;
            incomingByte = 'c';
            break;

        case '3':
            menuload = 1;
            if (Serial.available() > 0)
            {
                settings.ncur = Serial.parseInt();
            }
            menuload = 1;
            incomingByte = 'c';
            break;

        case '8':
            menuload = 1;
            if (Serial.available() > 0)
            {
                settings.changecur = Serial.parseInt();
            }
            menuload = 1;
            incomingByte = 'c';
            break;

        case '4':
            menuload = 1;
            if (Serial.available() > 0)
            {
                settings.convlow = Serial.parseInt();
            }
            incomingByte = 'c';
            break;

        case '5':
            menuload = 1;
            if (Serial.available() > 0)
            {
                settings.convhigh = Serial.parseInt();
            }
            incomingByte = 'c';
            break;

        case '6':
            menuload = 1;
            if (Serial.available() > 0)
            {
                settings.CurDead = Serial.parseInt();
            }
            incomingByte = 'c';
            break;

        case 113: // q for quite menu

            menuload = 0;
            incomingByte = 115;
            break;

        case 115: // s for switch sensor
            settings.cursens++;
            if (settings.cursens > 3)
            {
                settings.cursens = 0;
            }
            /*
              if (settings.cursens == Analoguedual)
              {
                settings.cursens = Canbus;
                SERIALCONSOLE.println("  ");
                SERIALCONSOLE.print(" CANbus Current Sensor ");
                SERIALCONSOLE.println("  ");
              }
              else
              {
                settings.cursens = Analoguedual;
                SERIALCONSOLE.println("  ");
                SERIALCONSOLE.print(" Analogue Current Sensor ");
                SERIALCONSOLE.println("  ");
              }
            */
            menuload = 1;
            incomingByte = 'c';
            break;

        case '7': // s for switch sensor
            settings.curcan++;
            if (settings.curcan > CurCanMax)
            {
                settings.curcan = 1;
            }
            menuload = 1;
            incomingByte = 'c';
            break;

        default:
            // if nothing else matches, do the default
            // default is optional
            break;
        }
    }

    if (menuload == 8)
    {
        switch (incomingByte)
        {
        case '1': // e dispaly settings
            if (Serial.available() > 0)
            {
                settings.IgnoreTemp = Serial.parseInt();
            }
            if (settings.IgnoreTemp > 2)
            {
                settings.IgnoreTemp = 0;
            }
            bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
            menuload = 1;
            incomingByte = 'i';
            break;

        case '2':
            if (Serial.available() > 0)
            {
                settings.IgnoreVolt = Serial.parseInt();
                settings.IgnoreVolt = settings.IgnoreVolt * 0.001;
                bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);
                // Serial.println(settings.IgnoreVolt);
                menuload = 1;
                incomingByte = 'i';
            }
            break;

        case 113: // q to go back to main menu

            menuload = 0;
            incomingByte = 115;
            break;
        }
    }

    if (menuload == 7)
    {
        switch (incomingByte)
        {
        case '1':
            if (Serial.available() > 0)
            {
                settings.WarnOff = Serial.parseInt();
                settings.WarnOff = settings.WarnOff * 0.001;
                menuload = 1;
                incomingByte = 'a';
            }
            break;

        case '2':
            if (Serial.available() > 0)
            {
                settings.CellGap = Serial.parseInt();
                settings.CellGap = settings.CellGap * 0.001;
                menuload = 1;
                incomingByte = 'a';
            }
            break;

        case '3':
            if (Serial.available() > 0)
            {
                settings.WarnToff = Serial.parseInt();
                menuload = 1;
                incomingByte = 'a';
            }
            break;

        case '4':
            if (Serial.available() > 0)
            {
                settings.triptime = Serial.parseInt();
                menuload = 1;
                incomingByte = 'a';
            }
            break;

        case 113: // q to go back to main menu
            menuload = 0;
            incomingByte = 115;
            break;
        }
    }

    if (menuload == 6) // Charging settings
    {
        switch (incomingByte)
        {

        case 113: // q to go back to main menu

            menuload = 0;
            incomingByte = 115;
            break;

        case '1':
            if (Serial.available() > 0)
            {
                settings.ChargeVsetpoint = Serial.parseInt();
                settings.ChargeVsetpoint = settings.ChargeVsetpoint / 1000;
                menuload = 1;
                incomingByte = 'e';
            }
            break;

        case '2':
            if (Serial.available() > 0)
            {
                settings.ChargeHys = Serial.parseInt();
                settings.ChargeHys = settings.ChargeHys / 1000;
                menuload = 1;
                incomingByte = 'e';
            }
            break;

        case '4':
            if (Serial.available() > 0)
            {
                settings.chargecurrentend = Serial.parseInt() * 10;
                menuload = 1;
                incomingByte = 'e';
            }
            break;

        case '3':
            if (Serial.available() > 0)
            {
                settings.chargecurrentmax = Serial.parseInt() * 10;
                menuload = 1;
                incomingByte = 'e';
            }
            break;

        case 'a':
            if (Serial.available() > 0)
            {
                settings.chargecurrent2max = Serial.parseInt() * 10;
                menuload = 1;
                incomingByte = 'e';
            }
            break;

        case '5': // 1 Over Voltage Setpoint
            settings.chargertype = settings.chargertype + 1;
            if (settings.chargertype > 7)
            {
                settings.chargertype = 0;
            }
            menuload = 1;
            incomingByte = 'e';
            break;

        case '6':
            if (Serial.available() > 0)
            {
                settings.chargerspd = Serial.parseInt();
                menuload = 1;
                incomingByte = 'e';
            }
            break;

        case '7':
            if (Serial.available() > 0)
            {
                settings.canSpeed = Serial.parseInt() * 1000;
                // Can1.end(); //FlexCAN_T4 has no end function TODO
                Can1.begin();
                Can1.setBaudRate(settings.canSpeed);
                menuload = 1;
                incomingByte = 'e';
            }
            break;

        case '8':
            if (settings.ChargerDirect == 1)
            {
                settings.ChargerDirect = 0;
                menuload = 1;
                incomingByte = 'e';
            }
            else
            {
                settings.ChargerDirect = 1;
                menuload = 1;
                incomingByte = 'e';
            }
            break;

        case '9':
            if (Serial.available() > 0)
            {
                settings.ChargeTSetpoint = Serial.parseInt();
                menuload = 1;
                incomingByte = 'e';
            }
            break;

        case 'b':
            if (Serial.available() > 0)
            {
                settings.chargereff = Serial.parseInt();
                menuload = 1;
                incomingByte = 'e';
            }
            break;

        case 'c':
            if (Serial.available() > 0)
            {
                settings.chargerACv = Serial.parseInt();
                menuload = 1;
                incomingByte = 'e';
            }
            break;

        case 'd':
            if (Serial.available() > 0)
            {
                if (settings.SerialCan == 0)
                {
                    settings.SerialCan = 1;
                }
                else
                {
                    settings.SerialCan = 0;
                }
                menuload = 1;
                incomingByte = 'e';
            }
            break;
        }
    }

    if (menuload == 5)
    {
        switch (incomingByte)
        {
        case '1':
            if (Serial.available() > 0)
            {
                settings.Pretime = Serial.parseInt();
                menuload = 1;
                incomingByte = 'k';
            }
            break;

        case '2':
            if (Serial.available() > 0)
            {
                settings.Precurrent = Serial.parseInt();
                menuload = 1;
                incomingByte = 'k';
            }
            break;

        case '3':
            if (Serial.available() > 0)
            {
                settings.conthold = Serial.parseInt();
                menuload = 1;
                incomingByte = 'k';
            }
            break;

        case '4':
            if (Serial.available() > 0)
            {
                settings.gaugelow = Serial.parseInt();
                gaugedebug = 2;
                gaugeUpdate();
                menuload = 1;
                incomingByte = 'k';
            }
            break;

        case '5':
            if (Serial.available() > 0)
            {
                settings.gaugehigh = Serial.parseInt();
                gaugedebug = 3;
                gaugeUpdate();
                menuload = 1;
                incomingByte = 'k';
            }
            break;

        case '6':
            settings.tripcont = !settings.tripcont;
            if (settings.tripcont > 1)
            {
                settings.tripcont = 0;
            }
            menuload = 1;
            incomingByte = 'k';
            break;

        case '7':
            if (settings.ChargerDirect == 1)
            {
                settings.ChargerDirect = 0;
            }
            else
            {
                settings.ChargerDirect = 1;
            }
            menuload = 1;
            incomingByte = 'k';
            break;

        case 113: // q to go back to main menu
            gaugedebug = 0;
            menuload = 0;
            incomingByte = 115;
            break;
        }
    }

    if (menuload == 3)
    {
        switch (incomingByte)
        {
        case 113: // q to go back to main menu

            menuload = 0;
            incomingByte = 115;
            break;

        case 'b':
            if (Serial.available() > 0)
            {
                settings.socvolt[0] = Serial.parseInt();
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case 'f': // f factory settings
            loadSettings();
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.println(" Coded Settings Loaded ");
            SERIAL_CONSOLE.println("  ");
            menuload = 1;
            incomingByte = 'b';
            break;

        case 'r': // r for reset
            SOCreset = 1;
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print(" mAh Reset ");
            SERIAL_CONSOLE.println("  ");
            menuload = 1;
            incomingByte = 'b';
            break;

        case '1': // 1 Over Voltage Setpoint
            if (Serial.available() > 0)
            {
                settings.OverVSetpoint = Serial.parseInt();
                settings.OverVSetpoint = settings.OverVSetpoint / 1000;
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case 'g':
            if (Serial.available() > 0)
            {
                settings.StoreVsetpoint = Serial.parseInt();
                settings.StoreVsetpoint = settings.StoreVsetpoint / 1000;
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case 'h':
            if (Serial.available() > 0)
            {
                settings.DisTaper = Serial.parseInt();
                settings.DisTaper = settings.DisTaper / 1000;
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case 'j':
            if (Serial.available() > 0)
            {
                settings.DisTSetpoint = Serial.parseInt();
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case 'c':
            if (Serial.available() > 0)
            {
                settings.socvolt[1] = Serial.parseInt();
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case 'd':
            if (Serial.available() > 0)
            {
                settings.socvolt[2] = Serial.parseInt();
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case 'e':
            if (Serial.available() > 0)
            {
                settings.socvolt[3] = Serial.parseInt();
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case '9': // Discharge Voltage Setpoint
            if (Serial.available() > 0)
            {
                settings.DischVsetpoint = Serial.parseInt();
                settings.DischVsetpoint = settings.DischVsetpoint / 1000;
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case 'k': // Discharge Voltage hysteresis
            if (Serial.available() > 0)
            {
                settings.DischHys = Serial.parseInt();
                settings.DischHys = settings.DischHys / 1000;
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case '0': // c Pstrings
            if (Serial.available() > 0)
            {
                settings.Pstrings = Serial.parseInt();
                menuload = 1;
                incomingByte = 'b';
                bms.setPstrings(settings.Pstrings);
            }
            break;

        case 'a': //
            if (Serial.available() > 0)
            {
                settings.Scells = Serial.parseInt();
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case '2': // 2 Under Voltage Setpoint
            if (Serial.available() > 0)
            {
                settings.UnderVSetpoint = Serial.parseInt();
                settings.UnderVSetpoint = settings.UnderVSetpoint / 1000;
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case '3': // 3 Over Temperature Setpoint
            if (Serial.available() > 0)
            {
                settings.OverTSetpoint = Serial.parseInt();
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case '4': // 4 Udner Temperature Setpoint
            if (Serial.available() > 0)
            {
                settings.UnderTSetpoint = Serial.parseInt();
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case '5': // 5 Balance Voltage Setpoint
            if (Serial.available() > 0)
            {
                settings.balanceVoltage = Serial.parseInt();
                settings.balanceVoltage = settings.balanceVoltage / 1000;
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case '6': // 6 Balance Voltage Hystersis
            if (Serial.available() > 0)
            {
                settings.balanceHyst = Serial.parseInt();
                settings.balanceHyst = settings.balanceHyst / 1000;
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case '7': // 7 Battery Capacity inAh
            if (Serial.available() > 0)
            {
                settings.CAP = Serial.parseInt();
                menuload = 1;
                incomingByte = 'b';
            }
            break;

        case '8': // discurrent in A
            if (Serial.available() > 0)
            {
                settings.discurrentmax = Serial.parseInt() * 10;
                menuload = 1;
                incomingByte = 'b';
            }
            break;
        }
    }

    if (menuload == 1)
    {
        switch (incomingByte)
        {
        case 'R': // restart
            CPU_REBOOT;
            break;

        case 'x': // Ignore Value Settings
            while (Serial.available())
            {
                Serial.read();
            }
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println("Experimental Settings");
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println("Do not use unless you know what it does!!!!!");
            SERIAL_CONSOLE.print("1 - Sending Experimental Victron CAN:");
            SERIAL_CONSOLE.println(settings.ExpMess);

            SERIAL_CONSOLE.println("q - Go back to menu");
            menuload = 9;
            break;

        case 'i': // Ignore Value Settings
            while (Serial.available())
            {
                Serial.read();
            }
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println("Ignore Value Settings");
            SERIAL_CONSOLE.print("1 - Temp Sensor Setting:");
            SERIAL_CONSOLE.println(settings.IgnoreTemp);
            SERIAL_CONSOLE.print("2 - Voltage Under Which To Ignore Cells:");
            SERIAL_CONSOLE.print(settings.IgnoreVolt * 1000, 0);
            SERIAL_CONSOLE.println("mV");
            SERIAL_CONSOLE.println("q - Go back to menu");
            menuload = 8;
            break;

        case 'e': // Charging settings
            while (Serial.available())
            {
                Serial.read();
            }
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println("Charging Settings");
            SERIAL_CONSOLE.print("1 - Cell Charge Voltage Limit Setpoint: ");
            SERIAL_CONSOLE.print(settings.ChargeVsetpoint * 1000, 0);
            SERIAL_CONSOLE.println("mV");
            SERIAL_CONSOLE.print("2 - Charge Hystersis: ");
            SERIAL_CONSOLE.print(settings.ChargeHys * 1000, 0);
            SERIAL_CONSOLE.println("mV");
            if (settings.chargertype > 0)
            {
                SERIAL_CONSOLE.print("3 - Pack Max Charge Current: ");
                SERIAL_CONSOLE.print(settings.chargecurrentmax * 0.1);
                SERIAL_CONSOLE.println("A");
                SERIAL_CONSOLE.print("4- Pack End of Charge Current: ");
                SERIAL_CONSOLE.print(settings.chargecurrentend * 0.1);
                SERIAL_CONSOLE.println("A");
            }
            SERIAL_CONSOLE.print("5- Charger Type: ");
            switch (settings.chargertype)
            {
            case 0:
                SERIAL_CONSOLE.print("Relay Control");
                break;
            case 1:
                SERIAL_CONSOLE.print("Brusa NLG5xx");
                break;
            case 2:
                SERIAL_CONSOLE.print("Volt Charger");
                break;
            case 3:
                SERIAL_CONSOLE.print("Eltek Charger");
                break;
            case 4:
                SERIAL_CONSOLE.print("Elcon Charger");
                break;
            case 5:
                SERIAL_CONSOLE.print("Victron/SMA");
                break;
            case 6:
                SERIAL_CONSOLE.print("Coda");
                break;
            case 7:
                SERIAL_CONSOLE.print("Victron HV Spec");
                break;
            }
            SERIAL_CONSOLE.println();
            if (settings.chargertype > 0)
            {
                SERIAL_CONSOLE.print("6- Charger Can Msg Spd: ");
                SERIAL_CONSOLE.print(settings.chargerspd);
                SERIAL_CONSOLE.println("mS");
                SERIAL_CONSOLE.print("7- Can Baudrate: ");
                SERIAL_CONSOLE.print(settings.canSpeed * 0.001, 0);
                SERIAL_CONSOLE.println("kbps");
            }
            SERIAL_CONSOLE.print("8 - Charger HV Connection: ");
            switch (settings.ChargerDirect)
            {
            case 0:
                SERIAL_CONSOLE.print(" Behind Contactors");
                break;
            case 1:
                SERIAL_CONSOLE.print("Direct To Battery HV");
                break;
            }
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.print("9 - Charge Current derate Low: ");
            SERIAL_CONSOLE.print(settings.ChargeTSetpoint);
            SERIAL_CONSOLE.println(" C");
            if (settings.chargertype > 0)
            {
                SERIAL_CONSOLE.print("a - Alternate Pack Max Charge Current: ");
                SERIAL_CONSOLE.print(settings.chargecurrent2max * 0.1);
                SERIAL_CONSOLE.println("A");
                SERIAL_CONSOLE.print("b - Charger AC to DC effiecency: ");
                SERIAL_CONSOLE.print(settings.chargereff);
                SERIAL_CONSOLE.println("%");
                SERIAL_CONSOLE.print("c - Charger AC Voltage: ");
                SERIAL_CONSOLE.print(settings.chargerACv);
                SERIAL_CONSOLE.println("VAC");
            }

            SERIAL_CONSOLE.print("d - Standard Can Voltage Scale: ");
            if (settings.SerialCan == 0)
            {
                SERIAL_CONSOLE.print("0.01");
            }
            else if (settings.SerialCan == 1)
            {
                SERIAL_CONSOLE.print("0.1");
            }
            SERIAL_CONSOLE.println();

            SERIAL_CONSOLE.println("q - Go back to menu");
            menuload = 6;
            break;

        case 'a': // Alarm and Warning settings
            while (Serial.available())
            {
                Serial.read();
            }
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println("Alarm and Warning Settings Menu");
            SERIAL_CONSOLE.print("1 - Voltage Warning Offset: ");
            SERIAL_CONSOLE.print(settings.WarnOff * 1000, 0);
            SERIAL_CONSOLE.println("mV");
            SERIAL_CONSOLE.print("2 - Cell Voltage Difference Alarm: ");
            SERIAL_CONSOLE.print(settings.CellGap * 1000, 0);
            SERIAL_CONSOLE.println("mV");
            SERIAL_CONSOLE.print("3 - Temp Warning Offset: ");
            SERIAL_CONSOLE.print(settings.WarnToff);
            SERIAL_CONSOLE.println(" C");
            // SERIALCONSOLE.print("4 - Temp Warning delay: ");
            // SERIALCONSOLE.print(settings.UnderDur);
            // SERIALCONSOLE.println(" mS");
            SERIAL_CONSOLE.print("4 - Over and Under Voltage Delay: ");
            SERIAL_CONSOLE.print(settings.triptime);
            SERIAL_CONSOLE.println(" mS");

            menuload = 7;
            break;

        case 'k': // contactor settings
            while (Serial.available())
            {
                Serial.read();
            }
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println("Contactor and Gauge Settings Menu");
            SERIAL_CONSOLE.print("1 - PreCharge Timer: ");
            SERIAL_CONSOLE.print(settings.Pretime);
            SERIAL_CONSOLE.println("mS");
            SERIAL_CONSOLE.print("2 - PreCharge Finish Current: ");
            SERIAL_CONSOLE.print(settings.Precurrent);
            SERIAL_CONSOLE.println(" mA");
            SERIAL_CONSOLE.print("3 - PWM contactor Hold 0 - 255 : ");
            SERIAL_CONSOLE.println(settings.conthold);
            SERIAL_CONSOLE.print("4 - PWM for Gauge Low 0 - 255 : ");
            SERIAL_CONSOLE.println(settings.gaugelow);
            SERIAL_CONSOLE.print("5 - PWM for Gauge High 0 - 255 : ");
            SERIAL_CONSOLE.println(settings.gaugehigh);
            if (settings.ESSmode == 1)
            {
                SERIAL_CONSOLE.print("6 - ESS Main Contactor or Trip : ");
                if (settings.tripcont == 0)
                {
                    SERIAL_CONSOLE.println("Trip Shunt");
                }
                else
                {
                    SERIAL_CONSOLE.println("Main Contactor and Precharge");
                }
                SERIAL_CONSOLE.print("7 - External Battery Enable : ");
                switch (settings.ChargerDirect)
                {
                case 0:
                    SERIAL_CONSOLE.print(" Enable In2");
                    break;
                case 1:
                    SERIAL_CONSOLE.print("Auto Start");
                    break;
                }
            }

            menuload = 5;
            break;

        case 113:                    // q to go back to main menu
            EEPROM.put(0, settings); // save all change to eeprom
            menuload = 0;
            debug = 1;
            break;
        case 'd': // d for debug settings
            while (Serial.available())
            {
                Serial.read();
            }
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println("Debug Settings Menu");
            SERIAL_CONSOLE.println("Toggle on / off");
            SERIAL_CONSOLE.print("1 - Can Debug : ");
            SERIAL_CONSOLE.println(candebug);
            SERIAL_CONSOLE.print("2 - Current Debug : ");
            SERIAL_CONSOLE.println(debugCur);
            SERIAL_CONSOLE.print("3 - Output Check : ");
            SERIAL_CONSOLE.println(outputcheck);
            SERIAL_CONSOLE.print("4 - Input Check : ");
            SERIAL_CONSOLE.println(inputcheck);
            SERIAL_CONSOLE.print("5 - ESS mode : ");
            SERIAL_CONSOLE.println(settings.ESSmode);
            SERIAL_CONSOLE.print("6 - Cells Present Reset : ");
            SERIAL_CONSOLE.println(cellspresent);
            SERIAL_CONSOLE.print("7 - Gauge Debug : ");
            SERIAL_CONSOLE.println(gaugedebug);
            SERIAL_CONSOLE.print("8 - CSV Output : ");
            SERIAL_CONSOLE.println(CSVdebug);
            SERIAL_CONSOLE.print("9 - Decimal Places to Show : ");
            SERIAL_CONSOLE.println(debugdigits);
            SERIAL_CONSOLE.print("d - CSV Delimiter : ");
            if (delim == 1)
            {
                SERIAL_CONSOLE.println("Space");
            }
            else
            {
                SERIAL_CONSOLE.println("Comma");
            }
            SERIAL_CONSOLE.println("q - Go back to menu");
            menuload = 4;
            break;

        case 99: // c for calibrate zero offset
            while (Serial.available())
            {
                Serial.read();
            }
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println("Current Sensor Calibration Menu");
            SERIAL_CONSOLE.println("c - To calibrate sensor offset");
            SERIAL_CONSOLE.print("s - Current Sensor Type : ");
            switch (settings.cursens)
            {
            case CURR_SENSE_ANALOGUE_DUAL:
                SERIAL_CONSOLE.println(" Analogue Dual Current Sensor ");
                break;
            case CURR_SENSE_ANALOGUE_GUESSING:
                SERIAL_CONSOLE.println(" Analogue Single Current Sensor ");
                break;
            case CURR_SENSE_CANBUS:
                SERIAL_CONSOLE.println(" Canbus Current Sensor ");
                break;
            default:
                SERIAL_CONSOLE.println("Undefined");
                break;
            }
            SERIAL_CONSOLE.print("1 - invert current : ");
            SERIAL_CONSOLE.println(settings.invertcur);
            SERIAL_CONSOLE.print("2 - Pure Voltage based SOC : ");
            SERIAL_CONSOLE.println(settings.voltsoc);
            SERIAL_CONSOLE.print("3 - Current Multiplication : ");
            SERIAL_CONSOLE.println(settings.ncur);
            if (settings.cursens == CURR_SENSE_ANALOGUE_GUESSING || settings.cursens == CURR_SENSE_ANALOGUE_DUAL)
            {
                SERIAL_CONSOLE.print("4 - Analogue Low Range Conv : ");
                SERIAL_CONSOLE.print(settings.convlow * 0.01, 2);
                SERIAL_CONSOLE.println(" mV / A");
            }
            if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL)
            {
                SERIAL_CONSOLE.print("5 - Analogue High Range Conv : ");
                SERIAL_CONSOLE.print(settings.convhigh * 0.01, 2);
                SERIAL_CONSOLE.println(" mV / A");
            }
            if (settings.cursens == CURR_SENSE_ANALOGUE_GUESSING || settings.cursens == CURR_SENSE_ANALOGUE_DUAL)
            {
                SERIAL_CONSOLE.print("6 - Current Sensor Deadband : ");
                SERIAL_CONSOLE.print(settings.CurDead);
                SERIAL_CONSOLE.println(" mV");
            }
            if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL)
            {

                SERIAL_CONSOLE.print("8 - Current Channel ChangeOver : ");
                SERIAL_CONSOLE.print(settings.changecur * 0.001);
                SERIAL_CONSOLE.println(" A");
            }

            if (settings.cursens == CURR_SENSE_CANBUS)
            {
                SERIAL_CONSOLE.print("7 - Can Current Sensor : ");
                if (settings.curcan == LemCAB300)
                {
                    SERIAL_CONSOLE.println(" LEM CAB300 / 500 series ");
                }
                else if (settings.curcan == LemCAB500)
                {
                    SERIAL_CONSOLE.println(" LEM CAB500 Special ");
                }
                else if (settings.curcan == IsaScale)
                {
                    SERIAL_CONSOLE.println(" IsaScale IVT - S ");
                }
                else if (settings.curcan == VictronLynx)
                {
                    SERIAL_CONSOLE.println(" Victron Lynx VE.CAN Shunt");
                }
            }
            SERIAL_CONSOLE.println("q - Go back to menu");
            menuload = 2;
            break;

        case 98: // c for calibrate zero offset
            while (Serial.available())
            {
                Serial.read();
            }
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println("Battery Settings Menu");
            SERIAL_CONSOLE.println("r - Reset AH counter");
            SERIAL_CONSOLE.println("f - Reset to Coded Settings");
            SERIAL_CONSOLE.println("q - Go back to menu");
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.println();
            SERIAL_CONSOLE.print("1 - Cell Over Voltage Setpoint : ");
            SERIAL_CONSOLE.print(settings.OverVSetpoint * 1000, 0);
            SERIAL_CONSOLE.print("mV");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("2 - Cell Under Voltage Setpoint : ");
            SERIAL_CONSOLE.print(settings.UnderVSetpoint * 1000, 0);
            SERIAL_CONSOLE.print("mV");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("3 - Over Temperature Setpoint : ");
            SERIAL_CONSOLE.print(settings.OverTSetpoint);
            SERIAL_CONSOLE.print("C");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("4 - Under Temperature Setpoint : ");
            SERIAL_CONSOLE.print(settings.UnderTSetpoint);
            SERIAL_CONSOLE.print("C");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("5 - Cell Balance Voltage Setpoint : ");
            SERIAL_CONSOLE.print(settings.balanceVoltage * 1000, 0);
            SERIAL_CONSOLE.print("mV");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("6 - Balance Voltage Hystersis : ");
            SERIAL_CONSOLE.print(settings.balanceHyst * 1000, 0);
            SERIAL_CONSOLE.print("mV");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("7 - Ah Battery Capacity : ");
            SERIAL_CONSOLE.print(settings.CAP);
            SERIAL_CONSOLE.print("Ah");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("8 - Pack Max Discharge : ");
            SERIAL_CONSOLE.print(settings.discurrentmax * 0.1);
            SERIAL_CONSOLE.print("A");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("9 - Cell Discharge Voltage Limit Setpoint : ");
            SERIAL_CONSOLE.print(settings.DischVsetpoint * 1000, 0);
            SERIAL_CONSOLE.print("mV");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("0 - Slave strings in parallel : ");
            SERIAL_CONSOLE.print(settings.Pstrings);
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("a - Cells in Series per String : ");
            SERIAL_CONSOLE.print(settings.Scells);
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("b - setpoint 1 : ");
            SERIAL_CONSOLE.print(settings.socvolt[0]);
            SERIAL_CONSOLE.print("mV");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("c - SOC setpoint 1 : ");
            SERIAL_CONSOLE.print(settings.socvolt[1]);
            SERIAL_CONSOLE.print(" % ");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("d - setpoint 2 : ");
            SERIAL_CONSOLE.print(settings.socvolt[2]);
            SERIAL_CONSOLE.print("mV");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("e - SOC setpoint 2 : ");
            SERIAL_CONSOLE.print(settings.socvolt[3]);
            SERIAL_CONSOLE.print(" % ");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("g - Storage Setpoint : ");
            SERIAL_CONSOLE.print(settings.StoreVsetpoint * 1000, 0);
            SERIAL_CONSOLE.print("mV");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("h - Discharge Current Taper Offset : ");
            SERIAL_CONSOLE.print(settings.DisTaper * 1000, 0);
            SERIAL_CONSOLE.print("mV");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("j - Discharge Current Temperature Derate : ");
            SERIAL_CONSOLE.print(settings.DisTSetpoint);
            SERIAL_CONSOLE.print("C");
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("k - Cell Discharge Voltage Hysteresis : ");
            SERIAL_CONSOLE.print(settings.DischHys * 1000, 0);
            SERIAL_CONSOLE.print("mV");
            SERIAL_CONSOLE.println("  ");

            SERIAL_CONSOLE.println();
            menuload = 3;
            break;

        default:
            // if nothing else matches, do the default
            // default is optional
            break;
        }
    }

    if (incomingByte == 115 && menuload == 0)
    {
        SERIAL_CONSOLE.println();
        SERIAL_CONSOLE.println("MENU");
        SERIAL_CONSOLE.println("Debugging Paused");
        SERIAL_CONSOLE.print("Firmware Version : ");
        SERIAL_CONSOLE.println(firmver);
        SERIAL_CONSOLE.println("b - Battery Settings");
        SERIAL_CONSOLE.println("a - Alarm and Warning Settings");
        SERIAL_CONSOLE.println("e - Charging Settings");
        SERIAL_CONSOLE.println("c - Current Sensor Calibration");
        SERIAL_CONSOLE.println("k - Contactor and Gauge Settings");
        SERIAL_CONSOLE.println("i - Ignore Value Settings");
        SERIAL_CONSOLE.println("d - Debug Settings");
        SERIAL_CONSOLE.println("x - Experimental Settings");
        SERIAL_CONSOLE.println("R - Restart BMS");
        SERIAL_CONSOLE.println("q - exit menu");
        debug = 0;
        menuload = 1;
    }
}