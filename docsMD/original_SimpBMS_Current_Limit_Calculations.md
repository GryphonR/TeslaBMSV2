# original SimpBMS Current Limit Calculations

## Summary

This document explains where the BMS generates charge and discharge current limits, how they are calculated, and gives worked numeric examples.

## Where in the code

- `currentlimit()` — calculates `discurrent` and `chargecurrent`: [src/main/TeslaBMSV2.cpp](src/main/TeslaBMSV2.cpp#L1035-L1190)
- `VEcan()` — sends limits to Victron (CAN ID 0x351, bytes 2–5): [src/main/TeslaBMSV2.cpp](src/main/TeslaBMSV2.cpp#L540-L610)
- `isrCP()` — reads J1772 control-pilot and sets `accurlim` (AC current available): [src/main/TeslaBMSV2.cpp](src/main/TeslaBMSV2.cpp#L1400-L1480)
- Settings and units: EEPROM struct and defaults: [src/config/config.h](src/config/config.h#L1-L120) and [src/config/Settings.h](src/config/Settings.h#L48-L66)

## High-level algorithm

1. Start from configured maxima:
   - `discurrent = settings.discurrentmax`
   - `chargecurrent = settings.chargecurrentmax` (or `settings.chargecurrent2max` when `chargecurrentlimit == true`)
2. Apply hard cutoffs to zero for unsafe conditions (over-temp, over-voltage, under-voltage/temperature).
3. Derate discharge current by:
   - Temperature: linear interpolation between `DisTSetpoint` and `OverTSetpoint` down to 0.
   - Low cell voltage taper between `DischVsetpoint` and `DischVsetpoint + DisTaper`.
4. Derate charge current by:
   - Low temperature: linear interpolation between `UnderTSetpoint` and `ChargeTSetpoint`.
   - High cell voltage: taper between `(ChargeVsetpoint - ChargeHys)` and `ChargeVsetpoint` (or storage values when `storagemode==1`), interpolating toward `chargecurrentend`.
5. Clamp negatives to zero.
6. If an AC pilot limit (`accurlim`) is present, convert available AC power to an equivalent maximum DC charge current and enforce it.

## Units and encoding

- `chargecurrent` and `discurrent` are used and sent as integer values representing 0.1 A (tenths of an amp). The code prints values as `settings.chargecurrentmax * 0.1` in the serial menu and multiplies/parses user input by 10.
- `VEcan()` sends `chargecurrent` in bytes 2–3 and `discurrent` in bytes 4–5 of CAN ID 0x351.

## Key code formulas

- AC power available (watts):

$$\text{chargerpower} = \text{accurlim} \times \text{settings.chargerACv} \times \text{settings.chargereff} \times 0.01$$

(implemented in code as: `chargerpower = accurlim * settings.chargerACv * settings.chargereff * 0.01`)

- Convert AC power to maximum (tenths of amp) DC charge current:

$$\text{tempchargecurrent} = \frac{\text{chargerpower} \times 10}{\text{packVoltage}}$$

where $$\text{packVoltage} = bms.getAvgCellVolt() \times settings.Scells$$

(code: `tempchargecurrent = (chargerpower * 10) / (bms.getAvgCellVolt() * settings.Scells);`)

- Linear derates use Arduino-style `map()` (i.e. linear interpolation). Example pattern used:

`chargecurrent = chargecurrent - map(value, in_min, in_max, out_min, out_max);`

## Worked numeric examples

Example 1 — AC pilot limiting (typical EVSE):

- Inputs:
  - `accurlim = 16` A (from J1772 pilot)
  - `settings.chargerACv = 240` V
  - `settings.chargereff = 85` (%)
  - Pack: `bms.getAvgCellVolt() = 3.7` V, `settings.Scells = 12` (packVoltage = 44.4 V)
  - `settings.chargecurrentmax = 1000` (100.0 A in tenths)

- Steps:
  1. Charger power: \(3256\mathrm{W}\) computed as

\[\text{chargerpower} = 16 \times 240 \times 0.85 = 3264\ \mathrm{W}\]

  2. Convert to tenths-of-amp DC limit:

\[\text{tempchargecurrent} = \frac{3264 \times 10}{44.4} \approx 735\ \text{(tenths)}\]\

  3. Which equals \(73.5\ \mathrm{A}\). The code then enforces `chargecurrent = min(chargecurrent, tempchargecurrent)`.

Example 2 — Voltage taper on charging:

- Inputs:
  - `settings.chargecurrentmax = 1000` (100.0 A)
  - `settings.chargecurrentend = 20` (2.0 A)
  - `settings.ChargeVsetpoint = 3.9` V
  - `settings.ChargeHys = 0.05` V
  - Observed `bms.getHighCellVolt() = 3.88` V (in the hysteresis window)

- Range: from `in_min = 3.85` (3.9 - 0.05) to `in_max = 3.9`.

- The code subtracts a mapped reduction computed as:

\[\text{reduction} = \mathrm{map}(3.88, 3.85, 3.9, 0, 1000 - 20) \approx 0.6 \times 980 \approx 588\ \text{(tenths)}\]

- New `chargecurrent = 1000 - 588 = 412` (i.e. 41.2 A).

Example 3 — Discharge temperature derate:

- Inputs:
  - `settings.discurrentmax = 4500` (450.0 A)
  - `settings.DisTSetpoint = 40` °C
  - `settings.OverTSetpoint = 65` °C
  - Observed `bms.getHighTemperature() = 52.5` °C

- Fraction through the derate window: \((52.5 - 40) / (65 - 40) = 12.5/25 = 0.5\).
- Reduction (tenths): \(0.5 \times 4500 = 2250\) (i.e. 225.0 A)
- Resulting `discurrent = 4500 - 2250 = 2250` (225.0 A)

## Notes and caveats

- The code uses integer `map()` operations; small truncation can occur.
- There are a few inconsistencies in defaults vs. menu scaling (defaults in `Settings.h` vs. the Serial menu multiply/divide by 10). Practically, the code treats stored current settings as tenths of amps — follow the Serial menu I/O behavior when interacting at runtime.
- The `map()` function does not clamp by default — values outside the input range can extrapolate; the code relies on prior hard cutoffs to avoid unsafe extrapolations.