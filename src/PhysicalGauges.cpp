#include "PhysicalGauges.h"
#include "globals.h"
#include "pinouts.h"


/*
 *@brief Updates a physical gauge output based on the state of charge (SOC) and debug settings.
 */
void gaugeUpdate()
{
    if (gaugedebug == 1)
    {
        SOCtest = SOCtest + 10;
        if (SOCtest > 1000)
        {
            SOCtest = 0;
        }
        analogWrite(PIN_OUT8, map(SOCtest * 0.1, 0, 100, settings.gaugelow, settings.gaugehigh));
        if (debug != 0)
        {
            SERIAL_CONSOLE.println("  ");
            SERIAL_CONSOLE.print("SOC : ");
            SERIAL_CONSOLE.print(SOCtest * 0.1);
            SERIAL_CONSOLE.print("  fuel pwm : ");
            SERIAL_CONSOLE.print(map(SOCtest * 0.1, 0, 100, settings.gaugelow, settings.gaugehigh));
            SERIAL_CONSOLE.println("  ");
        }
    }
    if (gaugedebug == 2)
    {
        SOCtest = 0;
        analogWrite(PIN_OUT8, map(SOCtest * 0.1, 0, 100, settings.gaugelow, settings.gaugehigh));
    }
    if (gaugedebug == 3)
    {
        SOCtest = 1000;
        analogWrite(PIN_OUT8, map(SOCtest * 0.1, 0, 100, settings.gaugelow, settings.gaugehigh));
    }
    if (gaugedebug == 0)
    {
        analogWrite(PIN_OUT8, map(SOC, 0, 100, settings.gaugelow, settings.gaugehigh));
    }
}