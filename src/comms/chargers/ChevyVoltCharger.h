#pragma once

#include "Charger.h"
#include <Arduino.h>

/**
 * @brief Implementation for Chevy Volt Charger.
 */
class ChevyVoltCharger : public Charger
{
public:
    ChevyVoltCharger(std::function<void(const CAN_message_t &)> sendCallback) 
        : _sendCallback(sendCallback) 
    {}

    bool onReceive(const CAN_message_t &msg) override
    {
        // Chevy Volt logic currently doesn't read specific messages in the original code,
        // but if it did, we'd handle them here.
        return false; 
    }

    void sendControlMessage(float voltage, float current) override
    {
        CAN_message_t msg;
        
        // 1. Send Command 0x30E
        msg.id = 0x30E;
        msg.len = 1;
        msg.buf[0] = 0x02; // only HV charging
        _sendCallback(msg);

        // 2. Send Command 0x304
        msg.id = 0x304;
        msg.len = 4;
        msg.buf[0] = 0x40; // fixed

        // Current Calculation
        uint16_t currentByte = (uint16_t)(current * 2); // Scale? Original code: chargecurrent * 2
        if (currentByte > 255) currentByte = 255;
        msg.buf[1] = (uint8_t)currentByte;

        // Voltage Calculation
        // Original: if ((settings.ChargeVsetpoint * settings.Scells) > 200) ...
        // We expect 'voltage' to be the full pack voltage target passed in.
        uint16_t voltageTarget = (uint16_t)voltage; 
        
        // Original logic checked > 200 (presumably volts?)
        // and sent highByte/lowByte of voltage * 2?
        // Let's replicate original logic:
        // settings.ChargeVsetpoint is per cell? Yes. scells is count.
        // Passed 'voltage' should be settings.ChargeVsetpoint * settings.Scells * (something?)
        // Wait, sendControlMessage takes 'voltage'. 
        // In TeslaBMSV2.cpp, we'll pass (settings.ChargeVsetpoint * settings.Scells).
        
        if (voltageTarget > 200)
        {
             uint16_t vVal = voltageTarget * 2;
             msg.buf[2] = highByte(vVal);
             msg.buf[3] = lowByte(vVal);
        }
        else
        {
             // Fallback from original code
             msg.buf[2] = highByte(400); 
             msg.buf[3] = lowByte(400);
        }
        
        _sendCallback(msg);
    }

private:
   std::function<void(const CAN_message_t &)> _sendCallback;
};
