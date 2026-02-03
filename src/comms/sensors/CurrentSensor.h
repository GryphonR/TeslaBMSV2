#pragma once

#include "../CanListener.h"

/**
 * @brief Abstract base class for all current sensors.
 */
class CurrentSensor : public CanListener
{
public:
    virtual ~CurrentSensor() {}

    /**
     * @brief Get the latest current reading in Amps.
     */
    virtual float getCurrent() = 0;
};
