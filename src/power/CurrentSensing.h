#pragma once
#include "stdint.h"

/**
 * @brief Reads and processes the current sensor value.
 *
 * This function acquires the current measurement from the sensor,
 * applies any necessary filtering or calibration, and updates
 * the relevant global variables with the latest current value.
 */
void getcurrent();

int32_t readAdcAsmA(uint8_t pin, uint16_t offset, float convFactor);
void updateCoulombCounter(float currentMa);
void processCurrentValue(int32_t rawInput);

uint16_t performCalibration(uint8_t pin);
void calcur();
