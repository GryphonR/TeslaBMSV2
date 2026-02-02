#include "config/globals.h"
#include "storage/Logger.h"
#include <Arduino.h>

/*
  Moved from main/TeslaBMSV2.cpp — calculates and sets
  `discurrent` and `chargecurrent` (in tenths of an amp).
*/
void currentlimit()
{
  // Current limit calculation
  // `discurrent` and `chargecurrent` are stored in tenths of amps (0.1 A units).
  // settings.discurrentmax / settings.chargecurrentmax etc. are in tenths as well.
  // Derates use Arduino `map()` for linear interpolation:
  //   map(x, in_min, in_max, out_min, out_max) -> interpolated value
  // AC pilot (`accurlim`) is in amps (A) from the J1772 pilot duty cycle.
  // We convert AC power to a DC current limit with:
  //   chargerpower = accurlim * settings.chargerACv * settings.chargereff * 0.01; // watts
  //   tempchargecurrent = (chargerpower * 10) / (avgCellVolt * settings.Scells); // tenths of amps
  Logger::debug("Entering Current Limit");
  if (bmsstatus == BMS_STATUS_ERROR)
  {
    discurrent = 0;
    chargecurrent = 0;
  }
  else
  {

    /// Start at no derating///
    discurrent = settings.discurrentmax;

    if (chargecurrentlimit == false)
    {
      chargecurrent = settings.chargecurrentmax;
    }
    else
    {
      chargecurrent = settings.chargecurrent2max;
    }

    ///////All hard limits to into zeros
    if (bms.getLowTemperature() < settings.UnderTSetpoint)
    {
      // discurrent = 0; Request Daniel
      chargecurrent = 0;
    }
    if (bms.getHighTemperature() > settings.OverTSetpoint)
    {
      discurrent = 0;
      chargecurrent = 0;
    }
    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      chargecurrent = 0;
    }
    if (bms.getHighCellVolt() > settings.OverVSetpoint)
    {
      chargecurrent = 0;
    }
    if (bms.getLowCellVolt() < settings.UnderVSetpoint || bms.getLowCellVolt() < settings.DischVsetpoint)
    {
      discurrent = 0;
    }

    // Modifying discharge current///

    if (discurrent > 0)
    {
      // Temperature based///

      if (bms.getHighTemperature() > settings.DisTSetpoint)
      {
        discurrent = discurrent - map(bms.getHighTemperature(), settings.DisTSetpoint, settings.OverTSetpoint, 0, settings.discurrentmax);
      }
      // Voltagee based///
      if (bms.getLowCellVolt() < (settings.DischVsetpoint + settings.DisTaper))
      {
        discurrent = discurrent - map(bms.getLowCellVolt(), settings.DischVsetpoint, (settings.DischVsetpoint + settings.DisTaper), settings.discurrentmax, 0);
      }
    }

    // Modifying Charge current///

    if (chargecurrent > 0)
    {
      if (chargecurrentlimit == false)
      {
        // Temperature based///
        if (bms.getLowTemperature() < settings.ChargeTSetpoint)
        {
          chargecurrent = chargecurrent - map(bms.getLowTemperature(), settings.UnderTSetpoint, settings.ChargeTSetpoint, settings.chargecurrentmax, 0);
        }
        // Voltagee based///
        if (storagemode == 1)
        {
          if (bms.getHighCellVolt() > (settings.StoreVsetpoint - settings.ChargeHys))
          {
            chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.StoreVsetpoint - settings.ChargeHys), settings.StoreVsetpoint, settings.chargecurrentend, settings.chargecurrentmax);
          }
        }
        else
        {
          if (bms.getHighCellVolt() > (settings.ChargeVsetpoint - settings.ChargeHys))
          {
            chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.ChargeVsetpoint - settings.ChargeHys), settings.ChargeVsetpoint, 0, (settings.chargecurrentmax - settings.chargecurrentend));
          }
        }
      }
      else
      {
        // Temperature based///
        if (bms.getLowTemperature() < settings.ChargeTSetpoint)
        {
          chargecurrent = chargecurrent - map(bms.getLowTemperature(), settings.UnderTSetpoint, settings.ChargeTSetpoint, settings.chargecurrent2max, 0);
        }
        // Voltagee based///
        if (storagemode == 1)
        {
          if (bms.getHighCellVolt() > (settings.StoreVsetpoint - settings.ChargeHys))
          {
            chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.StoreVsetpoint - settings.ChargeHys), settings.StoreVsetpoint, settings.chargecurrentend, settings.chargecurrent2max);
          }
        }
        else
        {
          if (bms.getHighCellVolt() > (settings.ChargeVsetpoint - settings.ChargeHys))
          {
            chargecurrent = chargecurrent - map(bms.getHighCellVolt(), (settings.ChargeVsetpoint - settings.ChargeHys), settings.ChargeVsetpoint, 0, (settings.chargecurrent2max - settings.chargecurrentend));
          }
        }
      }
    }
  }
  /// No negative currents///

  if (discurrent < 0)
  {
    discurrent = 0;
  }
  if (chargecurrent < 0)
  {
    chargecurrent = 0;
  }

  // Charge current derate for Control Pilot AC limit

  if (accurlim > 0)
  {
    // `accurlim` is A available from EVSE pilot. chargerACv is AC voltage (V),
    // chargereff is efficiency in percent. chargerpower is in watts.
    chargerpower = accurlim * settings.chargerACv * settings.chargereff * 0.01;
    // Convert available AC power to a DC current limit in tenths of amps:
    // tempchargecurrent = (W * 10) / packVoltage
    tempchargecurrent = (chargerpower * 10) / (bms.getAvgCellVolt() * settings.Scells);

    if (chargecurrent > tempchargecurrent)
    {
      chargecurrent = tempchargecurrent;
    }
  }
  Logger::debug("Exiting Current Limit");
}
