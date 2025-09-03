// This files contains functions related to ANALOGUE current sensing. 
// If a CAN based current sensor is used, it's handled in the CAN files

#include "CurrentSensing.h"
#include "globals.h"
#include "pinouts.h"

#include <Filters.h> //https://github.com/JonHub/Filters

// Curent filter//
float filterFrequency = 5.0;
FilterOnePole lowpassFilter(LOWPASS, filterFrequency);

void getcurrent()
{
    if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL || settings.cursens == CURR_SENSE_ANALOGUE_GUESSING)
    {
        if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL)
        {
            if (currentact < settings.changecur && currentact > (settings.changecur * -1))
            {
                sensor = 1;
                adc->adc0->startContinuous(PIN_ACUR_1);
            }
            else
            {
                sensor = 2;
                adc->adc0->startContinuous(PIN_ACUR_2);
            }
        }
        else
        {
            sensor = 1;
            adc->adc0->startContinuous(PIN_ACUR_1);
        }
        if (sensor == 1)
        {
            if (debugCur)
            {
                SERIAL_CONSOLE.println();
                if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL)
                {
                    SERIAL_CONSOLE.print("Low Range: ");
                }
                else
                {
                    SERIAL_CONSOLE.print("Single In: ");
                }
                SERIAL_CONSOLE.print("Value ADC0: ");
            }
            value = (uint16_t)adc->adc0->analogReadContinuous(); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
            if (debugCur)
            {
                SERIAL_CONSOLE.print(value * 3300 / adc->adc0->getMaxValue()); //- settings.offset1)
                SERIAL_CONSOLE.print(" ");
                SERIAL_CONSOLE.print(settings.offset1);
            }
            RawCur = int16_t((value * 3300 / adc->adc0->getMaxValue()) - settings.offset1) / (settings.convlow * 0.0000066);

            if (abs((int16_t(value * 3300 / adc->adc0->getMaxValue()) - settings.offset1)) < settings.CurDead)
            {
                RawCur = 0;
            }
            if (debugCur != 0)
            {
                SERIAL_CONSOLE.print("  ");
                SERIAL_CONSOLE.print(int16_t(value * 3300 / adc->adc0->getMaxValue()) - settings.offset1);
                SERIAL_CONSOLE.print("  ");
                SERIAL_CONSOLE.print(RawCur);
                SERIAL_CONSOLE.print(" mA");
                SERIAL_CONSOLE.print("  ");
            }
        }
        else
        {
            if (debugCur != 0)
            {
                SERIAL_CONSOLE.println();
                SERIAL_CONSOLE.print("High Range: ");
                SERIAL_CONSOLE.print("Value ADC0: ");
            }
            value = (uint16_t)adc->adc0->analogReadContinuous(); // the unsigned is necessary for 16 bits, otherwise values larger than 3.3/2 V are negative!
            if (debugCur != 0)
            {
                SERIAL_CONSOLE.print(value * 3300 / adc->adc0->getMaxValue()); //- settings.offset2)
                SERIAL_CONSOLE.print("  ");
                SERIAL_CONSOLE.print(settings.offset2);
            }
            RawCur = int16_t((value * 3300 / adc->adc0->getMaxValue()) - settings.offset2) / (settings.convhigh * 0.0000066);
            if (value < 100 || value > (adc->adc0->getMaxValue() - 100))
            {
                RawCur = 0;
            }
            if (debugCur != 0)
            {
                SERIAL_CONSOLE.print("  ");
                SERIAL_CONSOLE.print((float(value * 3300 / adc->adc0->getMaxValue()) - settings.offset2));
                SERIAL_CONSOLE.print("  ");
                SERIAL_CONSOLE.print(RawCur);
                SERIAL_CONSOLE.print("mA");
                SERIAL_CONSOLE.print("  ");
            }
        }
    }

    if (settings.invertcur == 1)
    {
        RawCur = RawCur * -1;
    }

    lowpassFilter.input(RawCur);
    if (debugCur != 0)
    {
        SERIAL_CONSOLE.print(lowpassFilter.output());
        SERIAL_CONSOLE.print(" | ");
        SERIAL_CONSOLE.print(settings.changecur);
        SERIAL_CONSOLE.print(" | ");
    }

    currentact = lowpassFilter.output();

    if (debugCur != 0)
    {
        SERIAL_CONSOLE.print(currentact);
        SERIAL_CONSOLE.print("mA  ");
    }

    if (settings.cursens == CURR_SENSE_ANALOGUE_DUAL)
    {
        if (sensor == 1)
        {
            if (currentact > 500 || currentact < -500)
            {
                ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
                lasttime = millis();
            }
            else
            {
                lasttime = millis();
            }
        }
        if (sensor == 2)
        {
            if (currentact > settings.changecur || currentact < (settings.changecur * -1))
            {
                ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
                lasttime = millis();
            }
            else
            {
                lasttime = millis();
            }
        }
    }
    else
    {
        if (currentact > 500 || currentact < -500)
        {
            ampsecond = ampsecond + ((currentact * (millis() - lasttime) / 1000) / 1000);
            lasttime = millis();
        }
        else
        {
            lasttime = millis();
        }
    }
    currentact = settings.ncur * currentact;
    RawCur = 0;
    /*
      AverageCurrentTotal = AverageCurrentTotal - RunningAverageBuffer[NextRunningAverage];

      RunningAverageBuffer[NextRunningAverage] = currentact;

      if (debugCur != 0)
      {
        SERIAL_CONSOLE.print(" | ");
        SERIAL_CONSOLE.print(AverageCurrentTotal);
        SERIAL_CONSOLE.print(" | ");
        SERIAL_CONSOLE.print(RunningAverageBuffer[NextRunningAverage]);
        SERIAL_CONSOLE.print(" | ");
      }
      AverageCurrentTotal = AverageCurrentTotal + RunningAverageBuffer[NextRunningAverage];
      if (debugCur != 0)
      {
        SERIAL_CONSOLE.print(" | ");
        SERIAL_CONSOLE.print(AverageCurrentTotal);
        SERIAL_CONSOLE.print(" | ");
      }

      NextRunningAverage = NextRunningAverage + 1;

      if (NextRunningAverage > RunningAverageCount)
      {
        NextRunningAverage = 0;
      }

      AverageCurrent = AverageCurrentTotal / (RunningAverageCount + 1);

      if (debugCur != 0)
      {
        SERIAL_CONSOLE.print(AverageCurrent);
        SERIAL_CONSOLE.print(" | ");
        SERIAL_CONSOLE.print(AverageCurrentTotal);
        SERIAL_CONSOLE.print(" | ");
        SERIAL_CONSOLE.print(NextRunningAverage);
      }
    */
}


void calcur()
{
    adc->startContinuous(PIN_ACUR_1, ADC_0);
    sensor = 1;
    x = 0;
    SERIAL_CONSOLE.print(" Calibrating Current Offset ::::: ");
    while (x < 20)
    {
        settings.offset1 = settings.offset1 + ((uint16_t)adc->adc0->analogReadContinuous() * 3300 / adc->adc0->getMaxValue());
        SERIAL_CONSOLE.print(".");
        delay(100);
        x++;
    }
    settings.offset1 = settings.offset1 / 21;
    SERIAL_CONSOLE.print(settings.offset1);
    SERIAL_CONSOLE.print(" current offset 1 calibrated ");
    SERIAL_CONSOLE.println("  ");
    x = 0;
    adc->adc0->startContinuous(PIN_ACUR_2);
    sensor = 2;
    SERIAL_CONSOLE.print(" Calibrating Current Offset ::::: ");
    while (x < 20)
    {
        settings.offset2 = settings.offset2 + ((uint16_t)adc->adc0->analogReadContinuous() * 3300 / adc->adc0->getMaxValue());
        SERIAL_CONSOLE.print(".");
        delay(100);
        x++;
    }
    settings.offset2 = settings.offset2 / 21;
    SERIAL_CONSOLE.print(settings.offset2);
    SERIAL_CONSOLE.print(" current offset 2 calibrated ");
    SERIAL_CONSOLE.println("  ");
}