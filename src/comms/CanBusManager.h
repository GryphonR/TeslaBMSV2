#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <vector>
#include "CanListener.h"

/**
 * @brief A simple wrapper that polls a CAN bus and dispatches messages to listeners.
 * 
 * This is a non-templated class that uses function pointers for flexibility.
 */
class CanBusManager
{
public:
    using ReadFunc = bool(*)(CAN_message_t &msg);
    using WriteFunc = int(*)(const CAN_message_t &msg);

    CanBusManager(ReadFunc readFunc, WriteFunc writeFunc) 
        : _read(readFunc), _write(writeFunc) {}

    void registerListener(CanListener *listener)
    {
        _listeners.push_back(listener);
    }

    void poll()
    {
        CAN_message_t msg;
        while (_read(msg))
        {
            for (auto listener : _listeners)
            {
                if (listener->onReceive(msg))
                {
                    break; 
                }
            }
        }
    }

    void send(const CAN_message_t &msg)
    {
        _write(msg);
    }

private:
    ReadFunc _read;
    WriteFunc _write;
    std::vector<CanListener *> _listeners;
};
