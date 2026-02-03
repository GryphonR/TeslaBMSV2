#pragma once

#include "Charger.h"
#include <Arduino.h>

class BrusaCharger : public Charger
{
public:
    BrusaCharger(std::function<void(const CAN_message_t &)> sendCallback) 
        : _sendCallback(sendCallback) 
    {}

    bool onReceive(const CAN_message_t &msg) override
    {
        return false; 
    }

    void sendControlMessage(float voltage, float current) override
    {
        // TODO: Brusa logic is complex and relies on External Inputs (PIN_IN2 for generator).
        // For a modular class, we should inject these dependencies or read them.
        // For this PoC, I will stub it out or implement a simplified version.
        // To properly implement, I would need to pass in 'isGeneratorMode' or similar.
    }

private:
   std::function<void(const CAN_message_t &)> _sendCallback;
};
