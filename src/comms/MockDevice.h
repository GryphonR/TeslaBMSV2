#pragma once

#include "comms/CanListener.h"
#include <Arduino.h>

class MockCurrentSensor : public CanListener
{
public:
    MockCurrentSensor(uint32_t id) : _id(id) {}

    bool onReceive(const CAN_message_t &msg) override
    {
        if (msg.id == _id)
        {
            Serial.print("MockSensor Received: ");
            Serial.println(msg.id, HEX);
            // Parse data...
            return true;
        }
        return false;
    }

private:
    uint32_t _id;
};
