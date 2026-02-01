#pragma once

/**
 * @brief Updates the Nextion display with current battery status information.
 *
 * This function sends various battery status parameters to a Nextion display
 * via Serial2. It updates fields such as state of charge (SOC), current,
 * temperature, voltage, cell balancing status, and firmware version.
 *
 * The function formats the data according to the Nextion display's command
 * structure, ensuring that each command is terminated with three 0xFF bytes
 * as required by the Nextion protocol.
 *
 * The display is updated based on the current BMS status, which can vary
 * depending on whether the system is in ESS mode or not.
 */
void dashupdate();