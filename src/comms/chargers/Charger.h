#pragma once

#include "../CanListener.h"
#include "../CanBusManager.h"

/**
 * @brief Abstract base class for Chargers.
 * 
 * Chargers are unique because they need to both LISTEN for status updates
 * and SEND control commands.
 */
class Charger : public CanListener
{
public:
    virtual ~Charger() {}

    /**
     * @brief Send control message to the charger.
     * @param voltage Max Charge Voltage (V)
     * @param current Max Charge Current (A)
     */
    virtual void sendControlMessage(float voltage, float current) = 0;
};
