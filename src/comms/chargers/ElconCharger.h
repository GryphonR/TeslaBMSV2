#pragma once

#include "Charger.h"
#include <Arduino.h>

/**
 * @brief Implementation for Elcon Charger.
 */
class ElconCharger : public Charger
{
public:
    ElconCharger(std::function<void(const CAN_message_t &)> sendCallback) 
        : _sendCallback(sendCallback) 
    {}

    bool onReceive(const CAN_message_t &msg) override
    {
        return false; 
    }

    void sendControlMessage(float voltage, float current) override
    {
        CAN_message_t msg;
        msg.id = 0x1806E5F4;
        msg.len = 8;
        msg.flags.extended = 1;
        
        uint16_t voltInt = (uint16_t)(voltage * 10);
        uint16_t currInt = (uint16_t)(current * 10);

        msg.buf[0] = highByte(voltInt);
        msg.buf[1] = lowByte(voltInt);
        msg.buf[2] = highByte(currInt);
        msg.buf[3] = lowByte(currInt);
        msg.buf[4] = 0x00;
        msg.buf[5] = 0x00;
        msg.buf[6] = 0x00;
        msg.buf[7] = 0x00;

        _sendCallback(msg);
    }

private:
   std::function<void(const CAN_message_t &)> _sendCallback;
};
