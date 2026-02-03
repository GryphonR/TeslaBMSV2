#pragma once

#include "CurrentSensor.h"
#include <Arduino.h>
#include <functional>

#define CAB300_OFFSET 0x800000

class CAB300 : public CurrentSensor
{
public:
    CAB300(std::function<void(float)> onUpdate, uint32_t id1 = 0x3C0, uint32_t id2 = 0x3C1, uint32_t id3 = 0x3C2)
        : _onUpdate(onUpdate), _id1(id1), _id2(id2), _id3(id3), _current(0.0f) {}

    bool onReceive(const CAN_message_t &msg) override
    {
        if (msg.id == _id1 || msg.id == _id2 || msg.id == _id3)
        {
            // Combine 3 bytes into a 32-bit integer (Byte 1, 2, 3) - NOT 0,1,2 like CAB500
            // Based on verified legacy code:
            // int32_t rawCan = (inMsg.buf[1] << 16) | (inMsg.buf[2] << 8) | inMsg.buf[3];
            int32_t rawCan = (msg.buf[1] << 16) | (msg.buf[2] << 8) | msg.buf[3];
            
            int32_t milliAmps = rawCan - CAB300_OFFSET;
            
            _current = (float)milliAmps / 1000.0f; // Store as Amps

            if (_onUpdate) _onUpdate(_current);
            return true;
        }
        return false;
    }

    float getCurrent() override
    {
        return _current;
    }

private:
    std::function<void(float)> _onUpdate;
    uint32_t _id1, _id2, _id3;
    float _current;
};
