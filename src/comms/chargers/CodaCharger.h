#pragma once

#include "Charger.h"
#include <Arduino.h>

class CodaCharger : public Charger
{
public:
    CodaCharger(std::function<void(const CAN_message_t &)> sendCallback) 
        : _sendCallback(sendCallback) 
    {}

    bool onReceive(const CAN_message_t &msg) override
    {
        return false; 
    }

    void sendControlMessage(float voltage, float current) override
    {
        CAN_message_t msg;
        msg.id = 0x050;
        msg.len = 8;
        msg.buf[0] = 0x00;
        msg.buf[1] = 0xDC;
        
        uint16_t vTag = (uint16_t)(voltage * 10);
        if (vTag > 200) { // Legacy check: (settings.ChargeVsetpoint * settings.Scells) > 200
             msg.buf[2] = highByte(vTag);
             msg.buf[3] = lowByte(vTag);
        } else {
             msg.buf[2] = highByte(400);
             msg.buf[3] = lowByte(400);
        }

        msg.buf[4] = 0x00;
        
        // Power Calc: Voltage * Current < 3300?
        if ((voltage * current) < 3300) {
            // (Voltage * Current) / 240
             uint16_t val = (uint16_t)((voltage * current) / 240);
             msg.buf[5] = highByte(val);
             msg.buf[6] = highByte(val); // Wait, legacy code says highByte for BOTH? 
             // msg.buf[5] = highByte(...); msg.buf[6] = highByte(...);
             // That looks like a bug in legacy code or very weird protocol.
             // I will reproduce legacy exactly for now.
        } else {
             msg.buf[5] = 0x00;
             msg.buf[6] = 0x96;
        }

        msg.buf[7] = 0x01; // HV charging
        _sendCallback(msg);
    }

private:
   std::function<void(const CAN_message_t &)> _sendCallback;
};
