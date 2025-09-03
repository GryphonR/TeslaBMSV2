#pragma once

/**
 * @brief Reads and processes the current sensor value.
 *
 * This function acquires the current measurement from the sensor,
 * applies any necessary filtering or calibration, and updates
 * the relevant global variables with the latest current value.
 */
void getcurrent();

/**
 * @brief Calibrates the current sensor offsets for both analogue channels.
 *
 * This function performs calibration for two current sensing channels by averaging
 * multiple ADC readings for each channel to determine the offset values. The offsets
 * are stored in the global settings structure as `offset1` and `offset2`.
 *
 * The function first calibrates channel 1 (PIN_ACUR_1), then channel 2 (PIN_ACUR_2),
 * printing progress and results to the serial console. Each calibration consists of
 * 20 readings, each separated by a 100 ms delay, and the average is computed and stored.
 *
 * Global variables affected:
 * - settings.offset1: Set to the average ADC value for channel 1.
 * - settings.offset2: Set to the average ADC value for channel 2.
 * - sensor: Indicates which channel is being calibrated.
 *
 * Serial output is used to indicate calibration progress and results.
 */
void calcur();