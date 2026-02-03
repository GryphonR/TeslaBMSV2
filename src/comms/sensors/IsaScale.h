#pragma once

#include "CurrentSensor.h"
#include <Arduino.h>
#include <functional>

class IsaScaleSensor : public CurrentSensor
{
public:
    IsaScaleSensor(std::function<void(float)> onUpdate)
        : _onUpdate(onUpdate), _current(0.0f) {}

    bool onReceive(const CAN_message_t &msg) override
    {
        // 1. Standard ISA Scale Current (0x521)
        if (msg.id == 0x521) 
        {
             int32_t milliAmps = (int32_t)((msg.buf[2] << 24) | (msg.buf[3] << 16) | (msg.buf[4] << 8) | msg.buf[5]);
             _current = (float)milliAmps / 1000.0f;
             if (_onUpdate) _onUpdate(_current);
             return true; 
        }

        // 2. Jaguar I-Pace ISA Shunt (0x3C3)
        if (msg.id == 0x3C3)
        {
             int32_t milliAmps = (int32_t)((msg.buf[2] << 24) | (msg.buf[3] << 16) | (msg.buf[4] << 8) | msg.buf[5]);
             _current = (float)milliAmps / 1000.0f;
             if (_onUpdate) _onUpdate(_current);
             return true;
        }

        // 3. Voltage messages (0x522, 0x523) - consume them
        if (msg.id == 0x522 || msg.id == 0x523) {
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
    float _current;
};
