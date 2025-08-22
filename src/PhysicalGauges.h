#pragma once

/**
 * @brief Updates the physical gauge output based on SOC and debug settings.
 *
 * This function sets the output for a physical gauge (e.g., fuel gauge)
 * according to the current state of charge (SOC) or debug/test values.
 */
void gaugeUpdate();